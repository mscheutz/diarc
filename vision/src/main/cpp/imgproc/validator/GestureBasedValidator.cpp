/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "GestureBasedValidator.hpp"
#include "common/notification/MemoryObjectNotification.hpp"
#include "capture/util/CaptureUtilities.hpp"
#include "display/Display.hpp"
#include "stm/util/StmUtilities.hpp"
#include <pcl/registration/distances.h>

using namespace ade::stm;

GestureBasedValidator::GestureBasedValidator(const long long &processorId, const int imgWidth, const int imgHeight)
        : ObjectValidator(processorId, imgWidth, imgHeight) {
  trackedObjects = TrackedObjects::getInstance();
  visionProcessName = "GestureBasedValidator";
  logger = log4cxx::Logger::getLogger("ade.imgproc.validator.GestureBasedValidator");
}

GestureBasedValidator::~GestureBasedValidator() {
}

void GestureBasedValidator::loadConfig(const std::string &configFile) {
}

void GestureBasedValidator::handleFrameCompletionNotification(FrameCompletionNotification::ConstPtr notification) {
  LOG4CXX_TRACE(logger, "[handleFrameCompletionNotification] method entered.");

  // this detector can currently only be used by a single typeId
  long long thisTypeId = getTypes()->begin()->first;

  // process notification
  boost::unordered_set<long long>::const_iterator typeIds_itr;
  for (typeIds_itr = notification->typeIds.begin(); typeIds_itr != notification->typeIds.end(); ++typeIds_itr) {
    long long notificationTypeId = *typeIds_itr;

    LOG4CXX_DEBUG(logger, boost::format("[handleFrameCompletionNotification] processing notification for typeId: %lld.")
                          % notificationTypeId);

    // this check prevents keeping track of frame completion from own tracker which can
    // happen when sub-type(s) are using same tracker instance as this detector
    if (thisTypeId != notificationTypeId) {
      lastFrameCompletedByType[notificationTypeId] = notification->frameNumber;
    }
  }

  // if have notifications from both sub-types -- send frameCompletion to next stage of pipeline
  if (lastFrameCompletedByType.size() == 2) {
    if (getTypes()->empty()) {
      LOG4CXX_ERROR(logger, "[handleFrameCompletionNotification] not processing any types.");
      return;
    }

    // have been notified by both referent and relatum searches, can now send along frame completion notification
    LOG4CXX_TRACE(logger, boost::format("[handleFrameCompletionNotification] sending notification for typeId: %lld.") % thisTypeId);
    FrameCompletionNotification::Ptr new_fcn(
            new FrameCompletionNotification(shared_from_this(), thisTypeId, notification->frameNumber));
    sendNotifications(new_fcn);
  } else {
    LOG4CXX_TRACE(logger,
                  "[handleFrameCompletionNotification] haven't received notification from both referent and relatum.");
  }
}

void GestureBasedValidator::handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification) {
  LOG4CXX_TRACE(logger, boost::format("[handleMemoryObjectNotification] method entered with typeId: %lld.")
                        % notification->object->getTypeId());

  DescriptorsByTypeConstPtr types = getTypes();
  if (types->size() > 2) {
    LOG4CXX_ERROR(logger, "A single GestureBasedValidator can not be used across more than one search!");
    return;
  } else if (types->empty()) {
    // no types to process
    return;
  }

  MemoryObject::Ptr object = notification->object;
  DescriptorsByType::const_iterator types_itr = types->begin();
  PredicateHelper::Set::const_iterator descriptors_itr;
  for (descriptors_itr = types_itr->second.begin(); descriptors_itr != types_itr->second.end(); ++descriptors_itr) {
    const std::string &referentVar = descriptors_itr->getArg(0); // object/part var
    const std::string &relatumVar = descriptors_itr->getArg(1); // gesture var

    // search MO tree for variables matching referent or relatum
    MemoryObject::Vec possibleReferents = object->getMemoryObjects(referentVar);
    MemoryObject::Vec possibleRelatums = object->getMemoryObjects(relatumVar);

    // try and find matches for these (root of scene graph containing referent/relatum)
    MemoryObject::Ptr referentRoot;
    MemoryObject::Ptr relatumRoot;

    // check if notification object is referent or relatum
    if (!possibleReferents.empty()) {
      LOG4CXX_DEBUG(logger, "[handleMemoryObjectNotification] have referent...looking for relatum.");
      referentRoot = object;

      // update last seen (root) MO
      lastSeenTokenIds[referentVar] = referentRoot->getId();

      // object notification is referent, see if we already know about a relatum
      boost::unordered_map<std::string, long long>::const_iterator lastSeen_itr = lastSeenTokenIds.find(relatumVar);
      if (lastSeen_itr != lastSeenTokenIds.end()) {
        // we have seen a relatum, get the corresponding MO
        relatumRoot = trackedObjects->getById(lastSeen_itr->second);
      } else {
        LOG4CXX_DEBUG(logger, "[handleMemoryObjectNotification] have yet to be notified of a relatum.");
      }

    } else if (!possibleRelatums.empty()) {
      LOG4CXX_DEBUG(logger, "[handleMemoryObjectNotification] have relatum...looking for referent.");
      relatumRoot = object;

      // update last seen (root) MO
      lastSeenTokenIds[relatumVar] = relatumRoot->getId();

      // object notification is relatum, see if we already know about a referent
      boost::unordered_map<std::string, long long>::const_iterator lastSeen_itr = lastSeenTokenIds.find(referentVar);
      if (lastSeen_itr != lastSeenTokenIds.end()) {
        // we have seen a referent, get the corresponding MO
        referentRoot = trackedObjects->getById(lastSeen_itr->second);
      } else {
        LOG4CXX_DEBUG(logger, "[handleMemoryObjectNotification] have yet to be notified of a referent.");
      }
    }

    //
    if (referentRoot && relatumRoot) {
      LOG4CXX_DEBUG(logger, "[handleMemoryObjectNotification] have both referent and relatum.");

      if (findReferencedObject(*descriptors_itr, referentRoot, relatumRoot)) {
        LOG4CXX_DEBUG(logger, "Gesture based detection made.");
        sendValidationNotifications(referentRoot);
      } else {
        LOG4CXX_DEBUG(logger, "NO gesture based detection made.");
      }

    } else {
      LOG4CXX_DEBUG(logger, "[handleMemoryObjectNotification] don't have both referent and relatum.");
    }

  }

}

bool GestureBasedValidator::findReferencedObject(const PredicateHelper &relation,
                                                const ade::stm::MemoryObject::Ptr &referent,
                                                const ade::stm::MemoryObject::Ptr &relatum) {

  LOG4CXX_DEBUG(logger, "[findReferencedObject] method entered.");

  // extract "point" gesture(s) from relatum (i.e., person MO) -- use highest conf if both hands
  PredicateHelper pointDescriptor("point(" + relation.getArg(1) + ")");
  MemoryObject::Ptr leftHand = relatum->getChildren()[1];
  MemoryObject::Ptr rightHand = relatum->getChildren()[2];
  float leftHandPointingConf = leftHand->getValidationResults().getConfidence(pointDescriptor);
  float rightHandPointingConf = rightHand->getValidationResults().getConfidence(pointDescriptor);

  LOG4CXX_DEBUG(logger, "[findReferencedObject] looking for pointing gesture.");

  MemoryObject::Ptr pointingMO;
  if (leftHandPointingConf > 0.0f && rightHandPointingConf > 0.0f) {
    // use highest confidence pointing gesture
    if (leftHandPointingConf > rightHandPointingConf) {
      pointingMO = leftHand;
    } else {
      pointingMO = rightHand;
    }
  } else if(leftHandPointingConf > 0.0f) {
    pointingMO = leftHand;
  } else if(rightHandPointingConf > 0.0f) {
    pointingMO = rightHand;
  } else {
    LOG4CXX_DEBUG(logger, "[findReferencedObject] no pointing gesture found."
                          " leftHand: " + leftHand->getValidationResultsString()
                          + " rightHand: " + rightHand->getValidationResultsString());
    return false;
  }

  LOG4CXX_DEBUG(logger, "[findReferencedObject] calculating gesture target point in cloud.");

  // use calcPointTarget to find where point gesture intersects point cloud
  pcl::PointXYZ targetPoint;
  if (!calcTargetPoint(relatum, pointingMO, targetPoint)) {
    LOG4CXX_DEBUG(logger, "[findReferencedObject] no target point found.");
  }

  LOG4CXX_DEBUG(logger, "[findReferencedObject] finding object that was pointed to.");

  // check the intersection point against the referent (or referent parts)
  MemoryObject::Ptr targetObject;
  std::string referentVar = relation.getArg(0);
  MemoryObject::Vec referentCandidates = referent->getMemoryObjects(referentVar);
  MemoryObject::Vec::const_iterator candidate_itr;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr sceneCloud = referent->getCaptureData()->cloud;
  pcl::PointXYZ scenePoint;
  int sceneCloudIndex;
  float distThresh = 0.05; //meters
  for (candidate_itr = referentCandidates.begin(); candidate_itr != referentCandidates.end(); ++candidate_itr) {
    const std::vector<int>& objectIndices = (*candidate_itr)->getDetectionMask()->getIndicesMask();
    std::vector<int>::const_iterator indices_itr;
    for (indices_itr = objectIndices.begin(); indices_itr != objectIndices.end(); ++indices_itr) {
      sceneCloudIndex = *indices_itr;
      scenePoint = sceneCloud->at(sceneCloudIndex);

      float distance = static_cast<float> (pcl::distances::l2(scenePoint.getVector4fMap(), targetPoint.getVector4fMap()));
      if (distance < distThresh) {
        // found intersection
        targetObject = *candidate_itr;
        break;
      }
    }

    if (targetObject) {
      break;
    }
  }

  // if pointing gesture intersects referent, add point(Y) MO child to referent, along with a
  // referencedBy(X,Y) relation between object(X)/part(X) and point(Y)
  if (targetObject) {
    std::vector<int> pointIndicies = {sceneCloudIndex};
    // TODO: this is technically the right typeId ... does this matter?
    MemoryObject::Ptr point(new MemoryObject(relatum->getTypeId(), relation.getArg(1), referent->getCaptureData(),
                                             MemoryObjectMask::Ptr(new MemoryObjectMask(referent->getCaptureData(), pointIndicies))));
    point->addValidationResult(0.9f, pointDescriptor);
    point->addRelation(0.9f, relation, targetObject);
    referent->addChild(point);

    return true;
  } else {
    return false;
  }
}



// calculate where the agent is pointing. Currently uses base of the index finger to the
// second to tip joint for close range point calculation (within .5-.6m) whereas
// uses elbow to wrist for longer range points (defaults to finger if these joints
// aren't available). So essentially close range use your index finger and long range
// point with a straight arm.
//An Opencv frame will pop up for any pointing hand, blue colored pixels show points in close
// proximity to pointing vector while red dot corresponds to calculated pointing location.
bool GestureBasedValidator::calcTargetPoint(const MemoryObject::Ptr &personRoot,
                                           const MemoryObject::Ptr &handRoot,
                                           pcl::PointXYZ &targetPoint) {
  LOG4CXX_DEBUG(logger, "[calcPointTarget] Entering method.");

  //get hand and body handJoints, init vars
  cv::Point3d elbowLoc = cv::Point3d(-1000,0,0);
  cv::Point3d wristLoc = cv::Point3d(-1000,0,0);
  cv::Point3d baseLoc = cv::Point3d(-1000,0,0);
  cv::Point3d midLoc = cv::Point3d(-1000,0,0);
  cv::Point3d endLoc = cv::Point3d(-1000,0,0);

  //iterate through hand children
  MemoryObject::Vec handJoints = handRoot->getChildren();
  MemoryObject::Vec::iterator joint_itr;
  for (joint_itr = handJoints.begin(); joint_itr != handJoints.end(); ++joint_itr) {
    //parse off hand and variable, just care about jointName
    std::string jointString = (*joint_itr)->getValidationResults().getDescriptors().begin()->getName();

    //get hand joint locations
    if (jointString.find("Radius") != std::string::npos) {
      wristLoc = (*joint_itr)->getDetectionMask()->getLocation();
    } else if (jointString.find("IndexCMC") != std::string::npos) {
      baseLoc = (*joint_itr)->getDetectionMask()->getLocation();
    } else if (jointString.find("IndexMCP") != std::string::npos) {
      midLoc = (*joint_itr)->getDetectionMask()->getLocation();
    } else if (jointString.find("IndexPIP") != std::string::npos) {
      endLoc = (*joint_itr)->getDetectionMask()->getLocation();
    }
  }

  //get elbow location for longer distance calc
  MemoryObject::Vec bodyJoints = personRoot->getChildren()[0]->getChildren();
  for (joint_itr = bodyJoints.begin(); joint_itr != bodyJoints.end(); ++joint_itr) {
    //parse off hand and variable, just care about jointName
    std::string jointString = (*joint_itr)->getValidationResults().getDescriptors().begin()->getName();
    if (jointString.find("Elbow") != std::string::npos) {
      elbowLoc = (*joint_itr)->getDetectionMask()->getLocation();
    }
  }

  //bools for if it's possible to compute close or far dist calc based on available handJoints
  bool close = true;
  bool far = true;
  cv::Point3d closeVec;
  cv::Point3d closeNorm;
  cv::Point3d farVec;
  cv::Point3d farNorm;
  double closeMag;
  double farMag;

  //If close or far calculations are possible, compute corresponding point vector
  if (elbowLoc.x == -1000 || wristLoc.x == -1000) {
    far == false;
  } else {
    farVec = wristLoc - elbowLoc;
    farMag = std::sqrt(farVec.x * farVec.x + farVec.y * farVec.y + farVec.z * farVec.z);
    farNorm = farVec / farMag;
  }

  if (baseLoc.x == -1000 || endLoc.x == -1000) {
    close == false;
  } else {
    //necessary?
    if (midLoc.x == -1000) {
      midLoc = baseLoc;
    }
    closeVec = endLoc - baseLoc;
    closeMag = std::sqrt(closeVec.x * closeVec.x + closeVec.y * closeVec.y + closeVec.z * closeVec.z);
    closeNorm = closeVec / closeMag;
  }
  // LOG4CXX_DEBUG(logger, boost::format("closeVec: (%d,%d,%d)") % closeVec.x % closeVec.y % closeVec.z);
  // LOG4CXX_DEBUG(logger, boost::format("closeNorm: (%d,%d,%d)") % closeNorm.x % closeNorm.y % closeNorm.z);
  // LOG4CXX_DEBUG(logger, boost::format("closeMag: %d") % closeMag);
  // LOG4CXX_DEBUG(logger, boost::format("farVec: (%d,%d,%d)") % farVec.x % farVec.y % farVec.z);
  // LOG4CXX_DEBUG(logger, boost::format("farNorm: (%d,%d,%d)") % farNorm.x % farNorm.y % farNorm.z);
  // LOG4CXX_DEBUG(logger, boost::format("farMag: %d") % farMag);

  cv::Mat frame = personRoot->getCaptureData()->frame.clone();
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud = personRoot->getCaptureData()->cloud;
  cv::Point3d baseToObjVec;
  double baseToObjDist;
  cv::Point3d proxPt, pointVec, pointNorm, proxVec, difVec;
  double proxThresh, proxMag, difMag;
  pcl::PointXYZ cloudPoint;
  double minDif = 1000;
  double minMag = 1000;
  int minI = cloud->width;
  int minJ = cloud->height;
  bool longDist = true;
  for (int i = 0; i < cloud->width; i++) {
    for (int j = 0; j < cloud->height; j++) {
      cloudPoint = cloud->at(i,j);
      //have handJoints for close, determine if should be used
      if (close) {
        baseToObjVec = cv::Point3d(cloudPoint.x - baseLoc.x,
                                   cloudPoint.y - baseLoc.y,
                                   cloudPoint.z - baseLoc.z);
        baseToObjDist = std::sqrt(baseToObjVec.x * baseToObjVec.x + baseToObjVec.y * baseToObjVec.y + baseToObjVec.z * baseToObjVec.z);
        //if close enough for finger handJoints to be accurate representation, use
        //or if we can only use close
        if (baseToObjDist < 0.6 || !far) {
          // LOG4CXX_INFO(logger, "close");
          proxPt = midLoc;
          pointVec = closeVec;
          pointNorm = closeNorm;
          proxThresh = 0.1;
          longDist = false;
        } else {
          // if farther away want to use arm handJoints for more stable point vector
          longDist = true;
        }
      }

      if (far && longDist) {
        baseToObjVec = cv::Point3d(cloudPoint.x - elbowLoc.x,
                                   cloudPoint.y - elbowLoc.y,
                                   cloudPoint.z - elbowLoc.z);
        baseToObjDist = std::sqrt(baseToObjVec.x * baseToObjVec.x + baseToObjVec.y * baseToObjVec.y + baseToObjVec.z * baseToObjVec.z);
        proxPt = elbowLoc;
        pointVec = farVec;
        pointNorm = farNorm;
        proxThresh = 0.6; //1.0
      }
        //awkward see if can change... if neither close or far work
      else if (!close) {
        LOG4CXX_INFO(logger, "Not enough handJoints for point location calc");
        return false;
      }

      //now calculate point independent of close/far considerations

      //prevent points on finger from being counted
      proxVec = cv::Point3d(cloudPoint.x - proxPt.x,
                            cloudPoint.y - proxPt.y,
                            cloudPoint.z - proxPt.z);
      proxMag = std::sqrt(proxVec.x * proxVec.x + proxVec.y * proxVec.y + proxVec.z * proxVec.z);

      // angle = std::acos(v1.dot(v2) / (mag1*mag2));

      difVec = (pointNorm * baseToObjDist) - baseToObjVec;
      difMag = std::sqrt(difVec.x * difVec.x + difVec.y * difVec.y + difVec.z * difVec.z);

      // if (angle < 0.1 && mag3 > 0.055) {
      if (difMag < 0.02 && proxMag > proxThresh) {
        frame.at<cv::Vec3b>(j,i)[0] = 255;
        frame.at<cv::Vec3b>(j,i)[1] = 0;
        frame.at<cv::Vec3b>(j,i)[2] = 0;
        if (difMag < minDif || (difMag == minDif && baseToObjDist < minMag)) {
          minDif = difMag;
          minI = i;
          minJ = j;
          minMag = baseToObjDist;
        }
      }
    }
  }

  // check if intersection point was found, and set display and return info
  if (minI >= 0 && minI < cloud->width && minJ >= 0 && minJ < cloud->height) {
    targetPoint = cloud->at(minI, minJ);

    if (getDisplayFlag()) {
      frame.at<cv::Vec3b>(minJ, minI)[0] = 0;
      frame.at<cv::Vec3b>(minJ, minI)[1] = 0;
      frame.at<cv::Vec3b>(minJ, minI)[2] = 255;

      ade::Display::createWindowIfDoesNotExist("Pointing Gesture");
      ade::Display::displayFrame(frame, "Pointing Gesture");
    }

    return true;
  } else {
    return false;
  }
}
