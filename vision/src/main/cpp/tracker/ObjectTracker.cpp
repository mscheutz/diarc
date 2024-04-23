/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "ObjectTracker.hpp"
#include "common/notification/FrameCompletionNotification.hpp"
#include "common/notification/MemoryObjectNotification.hpp"
#include "capture/Capture.hpp"
#include "detector/ObjectDetector.hpp"
#include "display/Display.hpp"

//derived classes includes
#include "GenericTracker.hpp"
#include "CMTTracker.hpp"
#include "RelationTracker.hpp"

#ifdef OPENCV_TRACKING
#if CV_VERSION_MINOR >= 2
#include "TLDTracker.hpp"
#include "KCFTracker.hpp"
#endif
#endif //OPENCV_TRACKING

#ifdef USE_V4R

#include "V4RTracker.hpp"

#endif  //USE_V4R

#include <boost/unordered/unordered_set.hpp>

using namespace ade::stm;

// NOTE: not named "ade.tracker.ObjectTracker" because there could be non-static
// loggers named "ade.tracker.ObjectTracker"
log4cxx::LoggerPtr ObjectTracker::factoryLogger = log4cxx::Logger::getLogger("ade.tracker.ObjectTracker.Factory");

ObjectTracker::Ptr
ObjectTracker::get(const TrackerType type, const long long &processorId, const int imgWidth, const int imgHeight) {
  switch (type) {
    case GENERIC:return GenericTracker::Ptr(new GenericTracker(processorId, imgWidth, imgHeight));
#ifdef OPENCV_TRACKING
#if CV_VERSION_MINOR >= 2
    case TLD:return TLDTracker::Ptr(new TLDTracker(processorId, imgWidth, imgHeight));
    case KCF:return KCFTracker::Ptr(new KCFTracker(processorId, imgWidth, imgHeight));
#endif
#endif //OPENCV_TRACKING
    case CMT:return CMTTracker::Ptr(new CMTTracker(processorId, imgWidth, imgHeight));
    case RELATION:return RelationTracker::Ptr(new RelationTracker(processorId, imgWidth, imgHeight));
    case V4R:
#ifdef USE_V4R
      return V4RTracker::Ptr(new V4RTracker(processorId, imgWidth, imgHeight));
#else
    LOG4CXX_ERROR(factoryLogger, "V4RTracker not available. Did you compile with V4R?");
#endif
      break;
  }

  LOG4CXX_ERROR(factoryLogger, "empty ObjectTracker::Ptr being returned");
  return ObjectTracker::Ptr();
}

ObjectTracker::ObjectTracker(const long long &processorId, const int imgWidth, const int imgHeight)
    : VisionProcess(processorId, imgWidth, imgHeight),
      trackingConfidenceThreshold(0.4),
      detectionConfidenceThreshold(0.0),
      dataProcessed_(false),
      objectsTracked_(false),
      numAddedObjects_(0),
      trackedObjects(NULL),
      publishToROS(true) {
  trackedObjects = TrackedObjects::getInstance();
  visionProcessName = "ObjectTracker";
  logger = log4cxx::Logger::getLogger("ade.tracker.ObjectTracker");
}

ObjectTracker::~ObjectTracker() {
}

void ObjectTracker::init() {
  lastFrameCompletedByType.clear();
  numFramesCompletedByType.clear();
  ade::capture::Capture::registerForNotification(shared_from_this());

  // LOG4CXX_DEBUG(logger, "In objectTracker init - starting ROS.");
  // std::vector<std::pair<std::string, std::string> > remapping;
  // ros::init(remapping, "ade_vision_cap");
  // n_ = new ros::NodeHandle();
  // pub = n_->advertise<c_her::TrackedObjects>("/ade_tracked_objects", 1);
}

void ObjectTracker::cleanup() {
  ade::capture::Capture::unregisterForNotification(shared_from_this());
}

//TODO: what to do with this??
// print tracking info
//      if (logger->isDebugEnabled()) {
//        DescriptorsByTypeConstPtr descriptorsByType = getTypes();
//        DescriptorsByType::const_iterator descriptorsByType_iter;
//        for (descriptorsByType_iter = descriptorsByType->begin(); descriptorsByType_iter != descriptorsByType->end(); ++descriptorsByType_iter) {
//          LOG4CXX_DEBUG(logger, boost::format("Currently tracking %d objects of typeId %lld.")
//                  % trackedObjects->getByTypeId(descriptorsByType_iter->first).size() % descriptorsByType_iter->first);
//        }
//      }

bool ObjectTracker::addProcessingDescriptor(const std::string &descriptor, const long long &typeId) {
  LOG4CXX_DEBUG(logger, boost::format("[addProcessingDescriptor]adding %d.") % descriptor);

  // lock
  boost::lock_guard<boost::mutex> lock(perform_mutex);

  // if typeId is not currently being tracked, clear the last frame completed
  // in case it was previously tracked
  DescriptorsByTypeConstPtr descriptorsByType = getTypes();
  if (descriptorsByType->find(typeId) == descriptorsByType->end()) {
    lastFrameCompletedByType.erase(typeId);
    numFramesCompletedByType.erase(typeId);
  }

  // call base class method explicitly to actually add the descriptor
  return VisionProcess::addProcessingDescriptor(descriptor, typeId);
}

bool ObjectTracker::removeProcessingDescriptor(const std::string &descriptor, const long long &typeId) {
  LOG4CXX_DEBUG(logger, boost::format("[removeProcessingDescriptor]removing %d.") % descriptor);

  //lock
  boost::lock_guard<boost::mutex> lock(perform_mutex);

  //call base class method explicitly
  bool removed = VisionProcess::removeProcessingDescriptor(descriptor, typeId);

  DescriptorsByTypeConstPtr descriptorsByType = getTypes();
  if (descriptorsByType->find(typeId) == descriptorsByType->end()) {
    //if all descriptors of type have been removed, remove tracked objects of specified SearchType
    LOG4CXX_DEBUG(logger, "[removeProcessingDescriptor] calling stopTracking.");
    stopTrackingType(typeId);
  } else if (logger->isDebugEnabled()) {
    DescriptorsByType::const_iterator itr = descriptorsByType->find(typeId);
    LOG4CXX_DEBUG(logger, boost::format("[removeProcessingDescriptor] %d left.") % itr->second.size());
  }

  return removed;
}

bool ObjectTracker::hasIterationCompleted(const long long &typeId) const {
  boost::unique_lock<boost::mutex> fc_lock(frameCompletion_mutex);
  boost::unordered_map<long long, unsigned long long>::const_iterator itr = lastFrameCompletedByType.find(typeId);
  if (itr != lastFrameCompletedByType.end()) {
    return true;
  } else {
    return false;
  }
}

long ObjectTracker::numIterationsCompleted(const long long &typeId) const {
  boost::unique_lock<boost::mutex> fc_lock(frameCompletion_mutex);
  boost::unordered_map<long long, long>::const_iterator itr = numFramesCompletedByType.find(typeId);
  if (itr != numFramesCompletedByType.end()) {
    return itr->second;
  } else {
    return 0;
  }
}

void ObjectTracker::handleCaptureNotification(CaptureNotification::ConstPtr notification) {
  // if (publishToROS)
  // publishTrackingResultsToROS(); 
  if (currentlyTrackingObjects()) {
    // do tracking iteration
    haveNewImage(notification);

    // clean up "dead" MemoryObjects
    removeLowConfidenceObjects();

    // display tracking results
    if (getDisplayFlag()) {
      displayResults(notification->captureData);
    }
  }
}

// void ObjectTracker::publishTrackingResultsToROS() {
//   // TODO: Allow for config to determine both whether we want to publish results to ROS or not
//   //        and what typeIds we want to publish results of
//   // MemoryObjects_HashedById_Set trackedObjectsByTypeId;
//   // tracked_objects_msg_->trackedObjects.clear();
//   // for typeId in typeIds:
//   MemoryObjects_HashedById_Set trackedObjectsByTypeId = trackedObjects->getByTypeId(0);
//   c_her::TrackedObjects tracked_objects_msg;
//   if (trackedObjectsByTypeId.size() > 0) {
//     MemoryObjects_HashedById_Set::const_iterator trackedObjectsByTypeIdIter;
//     for (trackedObjectsByTypeIdIter = trackedObjectsByTypeId.begin();
//          trackedObjectsByTypeIdIter != trackedObjectsByTypeId.end(); ++trackedObjectsByTypeIdIter) {
//       // extract relevant info from trackedObject
//       c_her::TrackedObject tracked_object_msg;
//       geometry_msgs::Point obj_location_msg;
//       // TODO: grasp_pose if desired, have to check if grasp child is present
//       // geometry_msgs::Pose grasp_pose_msg;

//       const MemoryObjectMask::ConstPtr obj_mask = (*trackedObjectsByTypeIdIter)->getTrackingMask();

//       obj_location_msg.x = obj_mask->getLocation().x;
//       obj_location_msg.y = obj_mask->getLocation().y;
//       obj_location_msg.z = obj_mask->getLocation().z;

//       tracked_object_msg.location = obj_location_msg;
//       tracked_object_msg.objectRef = (*trackedObjectsByTypeIdIter)->getVariableName();
//       tracked_object_msg.typeId = (*trackedObjectsByTypeIdIter)->getId();
//       tracked_object_msg.tokenId = (*trackedObjectsByTypeIdIter)->getTypeId();

//       tracked_objects_msg.trackedObjects.push_back(tracked_object_msg);
//     }
//   }
//   // pub message to node
//   LOG4CXX_DEBUG(logger, "[publishTrackingResultsToROS] publishing message");
//   pub.publish(tracked_objects_msg);
//   ros::spinOnce();
// }

void ObjectTracker::handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification) {
  LOG4CXX_TRACE(logger, boost::format("[handleMemoryObjectNotification] method entered. Tracking summary: %s.") %
      getTrackingSummary());

  if (!(notification->object)) {
    LOG4CXX_ERROR(logger, "[handleMemoryObjectNotification] MemoryObject NULL.");
    return;
  } else {
    LOG4CXX_TRACE(logger, boost::format("[handleMemoryObjectNotification] MO with validation: %s.") %
        notification->object->getValidationResultsString());
    if (notification->object->isTracked()) {
      //if already tracked, no need to add to tracker again
      LOG4CXX_TRACE(logger, "[handleMemoryObjectNotification] calling haveTrackedMemoryObject method.");
      haveTrackedMemoryObject(notification->object);
    } else {
      LOG4CXX_TRACE(logger, "[handleMemoryObjectNotification] calling haveNewMemoryObject method.");
      haveNewMemoryObject(notification->object);
    }
  }

  LOG4CXX_TRACE(logger, boost::format("[handleMemoryObjectNotification] method complete. Tracking summary: %s.") %
      getTrackingSummary());
}

void ObjectTracker::handleFrameCompletionNotification(FrameCompletionNotification::ConstPtr notification) {
  DescriptorsByTypeConstPtr types = getTypes(); //using getter so we don't have to lock

  // pass along notification for that particular search typeId (only if this VP is processing that typeId).
  // most trackers don't care about frame completions from the capture proc (i.e., typeId == -1), as most trackers get
  // continual capture notifications regardless of whether or not a detection iteration has happened
  boost::unordered_set<long long>::const_iterator typeIds_itr;
  for (typeIds_itr = notification->typeIds.begin(); typeIds_itr != notification->typeIds.end(); ++typeIds_itr) {
    long long typeId = *typeIds_itr;

    LOG4CXX_TRACE(logger, boost::format(
        "[handleFrameCompletionNotification] handling notification for typeId: %lld. Tracking summary: %s.")
        % typeId % getTrackingSummary());

    if (typeId != -1 && types->find(typeId) != types->end()) {

      // update local lastFrameCompletedByType
      boost::unique_lock<boost::mutex> lock(frameCompletion_mutex);
      if (lastFrameCompletedByType[typeId] < notification->frameNumber) {
        lastFrameCompletedByType[typeId] = notification->frameNumber;
        if (numFramesCompletedByType.find(typeId) != numFramesCompletedByType.end()) {
          numFramesCompletedByType[typeId] = numFramesCompletedByType[typeId] + 1;
        } else {
          numFramesCompletedByType[typeId] = 1;
        }
      }

      // pass along notification to rest of vision pipeline
      FrameCompletionNotification::Ptr new_fcn(
          new FrameCompletionNotification(shared_from_this(), notification->frameNumber, typeId));
      sendNotifications(new_fcn);
    }
  }
}

void ObjectTracker::haveTrackedMemoryObject(MemoryObject::Ptr trackedObject) {
  //get types that tracker should be processing
  DescriptorsByTypeConstPtr descriptorsByType = getTypes();
  DescriptorsByType::const_iterator descriptorsByType_iter;

  //TODO: check if trackedObject should be tracked, and remove it if it shouldn't? or in handleMemoryObjectNotification?

  //re-send notifications that trackedObject was updated so that it continues
  //to be updated by the validators
  sendTrackingNotifications(trackedObject);
}

void ObjectTracker::haveNewMemoryObject(MemoryObject::Ptr newObject) {
  LOG4CXX_TRACE(logger, boost::format("[ObjectTracker::haveNewMemoryObject] with frame number: %llu") %
      newObject->getDetectionMask()->getFrameNumber());

  //  SurfaceObject::Ptr surfaceObject = boost::dynamic_pointer_cast<SurfaceObject>(newObject);
  //  if (surfaceObject) {
  //    ade::Display::createWindowIfDoesNotExist("potential tracked object");
  //    ade::Display::displayFrame(surfaceObject->getObjectImage(), "potential tracked object");
  //  }

  int localAddedObjects = mergeObject(newObject);
  numAddedObjects_ += localAddedObjects;

  //  if (surfaceObject) {
  //    LOG4CXX_DEBUG(logger, boost::format("[ObjectTracker::haveNewMemoryObject] tracked: %d. VALIDATION: %s\n.") % localAddedObjects % newObject->getValidationResultsString());
  //    sleep(1);
  //  }
  //
  //  if (surfaceObject && localAddedObjects >= 1) {
  //    ade::Display::createWindowIfDoesNotExist("new tracked object");
  //    ade::Display::displayFrame(surfaceObject->getObjectImage(), "new tracked object");
  //    sleep(1);
  //  }

  //EAK: should we send notifications here too? should we only send if localAddedObjects > 0?
  //send notifications that newObject was updated
  if (localAddedObjects > 0) {
    sendTrackingNotifications(newObject);
  }
}

void ObjectTracker::sendTrackingNotifications(MemoryObject::VecPtr newTrackedObjects) {
  LOG4CXX_TRACE(logger,
                boost::format("[sendTrackingNotifications] num tracked objects: %d.") % newTrackedObjects->size());
  if (newTrackedObjects->size() > 0) {
    for (size_t i = 0; i < newTrackedObjects->size(); i++) {
      MemoryObjectNotification::Ptr n(new MemoryObjectNotification(shared_from_this(),
                                                                   (*newTrackedObjects)[i]->getTypeId(),
                                                                   (*newTrackedObjects)[i]->getTrackingMask()->getFrameNumber(),
                                                                   (*newTrackedObjects)[i]));
      sendNotifications(n);
    }
  }
}

void ObjectTracker::sendTrackingNotifications(MemoryObject::Ptr trackedObject) {
  LOG4CXX_TRACE(logger, "[sendTrackingNotifications] single tracked object.");
  if (trackedObject) {
    MemoryObjectNotification::Ptr n(new MemoryObjectNotification(shared_from_this(),
                                                                 trackedObject->getTypeId(),
                                                                 trackedObject->getTrackingMask()->getFrameNumber(),
                                                                 trackedObject));
    sendNotifications(n);
  }
}

bool ObjectTracker::currentlyTrackingObjects() {
  //don't need to lock bc only called from perform which already has lock
  DescriptorsByTypeConstPtr descriptorsByType = getTypes();
  DescriptorsByType::const_iterator descriptorsByType_iter;
  for (descriptorsByType_iter = descriptorsByType->begin();
       descriptorsByType_iter != descriptorsByType->end(); ++descriptorsByType_iter) {
    MemoryObjects_HashedById_Set trackedObjectsByType = trackedObjects->getByTypeId(descriptorsByType_iter->first);
    if (trackedObjectsByType.size() > 0) {
      return true;
    }
  }

  return false;
}

//iterate through all newly detected objects and try to 
//"match" with existing tracked objects

int ObjectTracker::mergeObjects(MemoryObject::VecPtr newlyDetectedObjects) {
  LOG4CXX_TRACE(logger, boost::format("[mergeObject] trying to merge %d objects.") % newlyDetectedObjects->size());
  int numAddedObjects = 0;

  // iterator for newly detected objects
  MemoryObject::Vec::iterator newObjectIter;
  //iterate through all newly detected objects
  for (newObjectIter = newlyDetectedObjects->begin();
       newObjectIter != newlyDetectedObjects->end(); ++newObjectIter) {

    numAddedObjects += mergeObject(*newObjectIter);
  }

  return numAddedObjects;
}

//iterate through all newly detected objects and try to 
//"match" with existing tracked objects

int ObjectTracker::mergeObject(MemoryObject::Ptr newlyDetectedObject) {
  LOG4CXX_TRACE(logger, "[mergeObject] trying to merge single object.");
  int numAddedObjects = 0;
  bool matchFound = false;

  //first find out what typeId(s) the object qualifies as
  DescriptorsByTypeConstPtr descriptorsByType = getTypes();
  DescriptorsByType::const_iterator descriptorsByType_iter = descriptorsByType->find(newlyDetectedObject->getTypeId());
  bool shouldTrack = false;
  LOG4CXX_TRACE(logger, boost::format("[mergeObject] typeId: %lld.") % newlyDetectedObject->getTypeId());
  LOG4CXX_TRACE(logger, boost::format("[mergeObject] num descriptors: %d.") % descriptorsByType->size());
  if (descriptorsByType_iter != descriptorsByType->end()
      && newlyDetectedObject->getDetectionConfidence(descriptorsByType_iter->second) > 0.0) {
    shouldTrack = true;
  }

  //if MemoryObject doesn't make the cut as any typeId, then don't track it
  if (!shouldTrack) {
    LOG4CXX_TRACE(logger, "[mergeObject] does not qualify as any typeId, not tracking object.");
    return 0;
  }

  //for all types that newlyDetectedObject qualifies as, compare against all
  //tracked objects of those types
  MemoryObjects_HashedById_Set trackedObjectsByTypeId;
  MemoryObjects_HashedById_Set::const_iterator trackedObjectsByTypeIdIter;
  //iterate through all existing tracked objects of matching typeId
  trackedObjectsByTypeId = trackedObjects->getByTypeId(newlyDetectedObject->getTypeId());
  for (trackedObjectsByTypeIdIter = trackedObjectsByTypeId.begin();
       trackedObjectsByTypeIdIter != trackedObjectsByTypeId.end(); ++trackedObjectsByTypeIdIter) {
    //found match, no need to check other objects of type
    if ((*trackedObjectsByTypeIdIter)->compare(newlyDetectedObject) == 1.0) {
      (*trackedObjectsByTypeIdIter)->update(newlyDetectedObject);
      matchFound = true;
      break;
    }
  }

  //no match found, add new MemoryObject to Tracker
  if (!matchFound) {
    if (canTrack(newlyDetectedObject)) { //tracker can reject object
      LOG4CXX_TRACE(logger, "[mergeObject] adding object to tracker.");
      startTracking(newlyDetectedObject);
      ++numAddedObjects;
    }
  }

  return numAddedObjects;
}

void ObjectTracker::removeLowConfidenceObjects() {
  LOG4CXX_TRACE(logger, "[removeLowConfidenceObjects] method entered.");

  DescriptorsByTypeConstPtr descriptorsByType = getTypes();
  DescriptorsByType::const_iterator descriptorsByType_iter;
  long long typeId;
  for (descriptorsByType_iter = descriptorsByType->begin();
       descriptorsByType_iter != descriptorsByType->end(); ++descriptorsByType_iter) {

    //find tracked objects below given threshold
    typeId = descriptorsByType_iter->first;
    MemoryObjects_HashedById_Set trackedObjectsByTypeId = trackedObjects->getByTypeId(typeId);
    MemoryObjects_HashedById_Set::const_iterator trackedObjectsByTypeIdIter;

    //find which ones to delete
    for (trackedObjectsByTypeIdIter = trackedObjectsByTypeId.begin();
         trackedObjectsByTypeIdIter != trackedObjectsByTypeId.end(); ++trackedObjectsByTypeIdIter) {

      //if tracking confidence is below tracking threshold, remove object completely
      if ((*trackedObjectsByTypeIdIter)->getTrackingConfidence() <= trackingConfidenceThreshold) {
        if (logger->isTraceEnabled()) {
          LOG4CXX_INFO(logger, boost::format(
              "[removeLowConfidenceObjects] removing tracked object %lld bc of tracking confidence: %f.")
              % (*trackedObjectsByTypeIdIter)->getId()
              % (*trackedObjectsByTypeIdIter)->getTrackingConfidence());
        }

        stopTracking(*trackedObjectsByTypeIdIter);
      } else if ((*trackedObjectsByTypeIdIter)->getDetectionConfidence(descriptorsByType_iter->second) <=
          detectionConfidenceThreshold) {
        //else if detection confidence is below detection threshold, only remove
        //object from tracking queue as that particular type (same object can be
        //multiple types at once)
        if (logger->isTraceEnabled()) {
          LOG4CXX_INFO(logger, boost::format(
              "[removeLowConfidenceObjects] removing tracked object %lld bc of detection confidence: %f.")
              % (*trackedObjectsByTypeIdIter)->getId()
              %
          (*trackedObjectsByTypeIdIter)->getDetectionConfidence(descriptorsByType_iter->second));
        }

        stopTracking(*trackedObjectsByTypeIdIter);
      }
    }
  }
}

void ObjectTracker::sendTrackingNotificationsForAllTrackedObjects() {
  LOG4CXX_TRACE(logger, "[sendTrackingNotificationsForAllTrackedObjects] method entered.");

  DescriptorsByTypeConstPtr descriptorsByType = getTypes();
  DescriptorsByType::const_iterator descriptorsByType_iter;
  long long typeId;
  for (descriptorsByType_iter = descriptorsByType->begin();
       descriptorsByType_iter != descriptorsByType->end(); ++descriptorsByType_iter) {

    // get objects with typeId
    typeId = descriptorsByType_iter->first;
    MemoryObjects_HashedById_Set trackedObjectsByTypeId = trackedObjects->getByTypeId(typeId);
    MemoryObjects_HashedById_Set::const_iterator trackedObjectsByTypeIdIter;

    // send tracking notification for each MemoryObject
    for (trackedObjectsByTypeIdIter = trackedObjectsByTypeId.begin();
         trackedObjectsByTypeIdIter != trackedObjectsByTypeId.end(); ++trackedObjectsByTypeIdIter) {

      sendTrackingNotifications(*trackedObjectsByTypeIdIter);
    }
  }
}

bool ObjectTracker::canTrack(const MemoryObject::Ptr &newMemObj) {
  return true;
}

void ObjectTracker::startTracking(const MemoryObject::Ptr &newMemObj) {
  LOG4CXX_TRACE(logger, "[startTracking] base class method entered.");
  newMemObj->startTracking();
}

void ObjectTracker::stopTracking(const MemoryObject::Ptr &existingMemObj) {
  LOG4CXX_TRACE(logger, "[stopTracking] base class method entered.");
  existingMemObj->stopTracking();
}

void ObjectTracker::stopTrackingType(const long long &typeId) {
  LOG4CXX_TRACE(logger, "[stopTrackingType] method entered.");
  //stop tracking MemoryObjects as typeId
  MemoryObjects_HashedById_Set memoryObjects = trackedObjects->getByTypeId(typeId);
  MemoryObjects_HashedById_Set::iterator memoryObjects_itr;
  for (memoryObjects_itr = memoryObjects.begin(); memoryObjects_itr != memoryObjects.end(); ++memoryObjects_itr) {
    stopTracking(*memoryObjects_itr);
  }
}

void ObjectTracker::displayResults(CaptureData::ConstPtr capture) {
  LOG4CXX_TRACE(logger, "[displayResults] base class method entered.");

  //get image to display
  capture->frame.copyTo(displayFrame);

  // get point cloud to display
  pcl::PointCloud<pcl::PointXYZ>::Ptr displayCloud(new pcl::PointCloud<pcl::PointXYZ>());

  // go through all memory objects
  DescriptorsByTypeConstPtr descriptorsByType = getTypes();
  DescriptorsByType::const_iterator descriptorsByType_iter;

  int rootColor = 255;
  int childColor = 0;
  for (descriptorsByType_iter = descriptorsByType->begin();
       descriptorsByType_iter != descriptorsByType->end(); ++descriptorsByType_iter) {
    MemoryObjects_HashedById_Set trackedObjectsByTypeId = trackedObjects->getByTypeId(descriptorsByType_iter->first);

    //draw tracked objects
    MemoryObjects_HashedById_Set::const_iterator trackedObjectsByTypeIdIter;
    for (trackedObjectsByTypeIdIter = trackedObjectsByTypeId.begin();
         trackedObjectsByTypeIdIter != trackedObjectsByTypeId.end(); ++trackedObjectsByTypeIdIter) {

      // concatenate objects' clouds
      if (capture->hasCloudRGBData()) {
        *displayCloud += (*trackedObjectsByTypeIdIter)->getTrackingMask()->getObjectPointCloud().operator*();
      }

      // draw root scene graph node
      cv::Rect bbox = (*trackedObjectsByTypeIdIter)->getTrackingMask()->getBoundingBox();
      LOG4CXX_TRACE(logger, boost::format("[displayResults] bbox: %d %d %d %d.")
          % bbox.x % bbox.y % bbox.width % bbox.height);
      cv::rectangle(displayFrame, bbox, rootColor, 3);

      // iterate through entire scene graph
      MemoryObject::Vec trackedSceneGraph = (*trackedObjectsByTypeIdIter)->getChildren();
      MemoryObject::Vec::iterator trackedSceneGraph_itr;
      for (trackedSceneGraph_itr = trackedSceneGraph.begin();
           trackedSceneGraph_itr != trackedSceneGraph.end(); ++trackedSceneGraph_itr) {

        // draw scene graph node
        cv::Rect bbox = (*trackedSceneGraph_itr)->getTrackingMask()->getBoundingBox();
        LOG4CXX_TRACE(logger, boost::format("[displayResults] bbox: %d %d %d %d.")
            % bbox.x % bbox.y % bbox.width % bbox.height);
        cv::rectangle(displayFrame, bbox, childColor, 3);
      }
    }
  }

  //if (capture->hasCloudRGBData()) {
  //  ade::Display::displayPointCloud(capture->cloudRGB, "scene_cloud", getDisplayName());
  //  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr model_color_handler(
  //          new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(displayCloud, 0, 255, 0));
  //  ade::Display::displayPointCloud(displayCloud, model_color_handler, "model_cloud", getDisplayName());
  //}

  ade::Display::displayFrame(displayFrame, getDisplayName());
}

std::string ObjectTracker::getTrackingSummary() {
  LOG4CXX_TRACE(logger, "[logTrackingSummary] base class method entered.");

  // go through all memory objects
  std::string returnString;
  DescriptorsByTypeConstPtr descriptorsByType = getTypes();
  DescriptorsByType::const_iterator descriptorsByType_iter;
  for (descriptorsByType_iter = descriptorsByType->begin();
       descriptorsByType_iter != descriptorsByType->end(); ++descriptorsByType_iter) {
    MemoryObjects_HashedById_Set trackedObjectsByTypeId = trackedObjects->getByTypeId(descriptorsByType_iter->first);

    returnString += boost::str(boost::format("TypeId: %lld. Num tracked objects: %lu.")
                                   % descriptorsByType_iter->first
                                   % trackedObjectsByTypeId.size());
  }

  return returnString;
}