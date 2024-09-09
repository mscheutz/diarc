/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "SpatialRelationDetector.hpp"
#include "common/notification/MemoryObjectNotification.hpp"
#include "capture/util/CaptureUtilities.hpp"
#include "display/Display.hpp"
#include "stm/util/StmUtilities.hpp"
#include <pcl/registration/distances.h>

using namespace diarc::stm;

SpatialRelationDetector::SpatialRelationDetector(const long long &processorId, const int imgWidth, const int imgHeight)
        : ObjectDetector(processorId, imgWidth, imgHeight) {
  trackedObjects = TrackedObjects::getInstance();
  visionProcessName = "SpatialRelationDetector";
  logger = log4cxx::Logger::getLogger("diarc.detector.SpatialRelationDetector");
}

SpatialRelationDetector::~SpatialRelationDetector() {
}

void SpatialRelationDetector::loadConfig(const std::string &configFile) {
}

void SpatialRelationDetector::init() {
}

void SpatialRelationDetector::cleanup() {
}

void SpatialRelationDetector::handleFrameCompletionNotification(FrameCompletionNotification::ConstPtr notification) {

  // process notification
  boost::unordered_set<long long>::const_iterator typeIds_itr;
  for (typeIds_itr = notification->typeIds.begin(); typeIds_itr != notification->typeIds.end(); ++typeIds_itr) {
    long long typeId = *typeIds_itr;

    LOG4CXX_DEBUG(logger, boost::format("[handleFrameCompletionNotification] processing notification for typeId: %lld.")
                          % typeId);

    lastFrameCompletedByType[typeId] = notification->frameNumber;
  }

  // if have notifications from both sub-types -- send frameCompletion to next stage of pipeline
  if (lastFrameCompletedByType.size() == 2) {
    if (getTypes()->empty()) {
      LOG4CXX_ERROR(logger, "[handleFrameCompletionNotification] not processing any types.");
      return;
    }

    // have been notified by both referent and relatum searches, can now send along frame completion notification
    LOG4CXX_TRACE(logger, "[handleFrameCompletionNotification] sending notification.");
    long long typeId = getTypes()->begin()->first;
    FrameCompletionNotification::Ptr new_fcn(
            new FrameCompletionNotification(shared_from_this(), notification->frameNumber, typeId));
    sendNotifications(new_fcn);
  } else {
    LOG4CXX_TRACE(logger,
                  "[handleFrameCompletionNotification] haven't received notification from both referent and relatum.");
  }
}

void SpatialRelationDetector::handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification) {
  LOG4CXX_TRACE(logger, boost::format("[handleMemoryObjectNotification] method entered with typeId: %lld.")
                        % notification->object->getTypeId());

  DescriptorsByTypeConstPtr types = getTypes();
  if (types->size() > 2) {
    LOG4CXX_ERROR(logger, "A single SpatialRelationDetector can not be used across more than one search!");
    return;
  } else if (types->empty()) {
    // no types to process
    return;
  }

  MemoryObject::Ptr object = notification->object;
  DescriptorsByType::const_iterator types_itr = types->begin();
  PredicateHelper::Set::const_iterator descriptors_itr;
  for (descriptors_itr = types_itr->second.begin(); descriptors_itr != types_itr->second.end(); ++descriptors_itr) {
    const std::string &referentVar = descriptors_itr->getArg(0);
    const std::string &relatumVar = descriptors_itr->getArg(1);

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
    bool relationFound = false;
    if (referentRoot && relatumRoot) {
      LOG4CXX_DEBUG(logger, "[handleMemoryObjectNotification] have both referent and relatum.");

      MemoryObject::Vec referents = referentRoot->getMemoryObjects(referentVar);
      MemoryObject::Vec relatums = relatumRoot->getMemoryObjects(relatumVar);
      MemoryObject::Vec::iterator referent_itr;
      MemoryObject::Vec::iterator relatum_itr;
      for (referent_itr = referents.begin(); referent_itr != referents.end(); ++referent_itr) {
        for (relatum_itr = relatums.begin(); relatum_itr != relatums.end(); ++relatum_itr) {
          if (checkForRelation(*descriptors_itr, *referent_itr, *relatum_itr)) {
            relationFound = true;
          }
        }
      }

    } else {
      LOG4CXX_DEBUG(logger, "[handleMemoryObjectNotification] don't have both referent and relatum.");
    }

    if (relationFound) {
      LOG4CXX_DEBUG(logger, "Spatial Relation found.");
      // not creating any new memory objects here,
      // only adding relations, so just sending the referent root along to tracker
      sendDetectionNotifications(referentRoot);
    } else {
      LOG4CXX_DEBUG(logger, "NO Spatial Relation found.");
    }

  }

}


bool SpatialRelationDetector::checkForRelation(const PredicateHelper &relation,
                                               const diarc::stm::MemoryObject::Ptr &referent,
                                               const diarc::stm::MemoryObject::Ptr &relatum) {
  if (relation.getName().compare("touching") == 0 || relation.getName().compare("on") == 0) {
    return checkForTouchingRelation(relation, referent, relatum);
  } else if (relation.getName().compare("onTop") == 0) {
    return checkForOnTopRelation(relation, referent, relatum);
  }

  //relation not found -- fail case
  LOG4CXX_ERROR(logger, boost::format("[checkForRelation] can't handle relation of type: %s.") % relation.toString());
  return false;
}

bool SpatialRelationDetector::checkForTouchingRelation(const PredicateHelper &relation,
                                                       const diarc::stm::MemoryObject::Ptr &referent,
                                                       const diarc::stm::MemoryObject::Ptr &relatum) {

  std::vector<int> indices0 = referent->getTrackingMask()->getIndicesMask();
  std::vector<int> indices1 = relatum->getTrackingMask()->getIndicesMask();
  int x0, y0, x1, y1, index0, index1;
  float distance;
  float distThresh = 0.15; //meters
  int onCount = 0;
  int imgWidth = referent->getTrackingMask()->getImageMask().cols;
  // for each point in referent, find closest point in relatum
  LOG4CXX_TRACE(logger, boost::format("[calculateOnRelation] referent %s indices: %d. relatum %s indices: %d.")
                        % referent->getVariableName() % indices0.size() % relatum->getVariableName() % indices1.size());
  for (int i0 = 0; i0 < indices0.size(); ++i0) {
    index0 = indices0[i0];
    x0 = (index0 % imgWidth);
    y0 = (index0 / imgWidth);
    //    pcl::PointXYZ p0 = referent->getMask()->getObjectPoint(index0);
    for (int i1 = 0; i1 < indices1.size(); ++i1) {
      index1 = indices1[i1];
      x1 = (index1 % imgWidth);
      y1 = (index1 / imgWidth);
      //      pcl::PointXYZ p1 = relatum->getMask()->getObjectPoint(index1);
      //      LOG4CXX_TRACE(logger, boost::format("[calculateOnRelation] referent index: %d. (x,y) = (%d,%d). (x,y,z) = (%f,%f,%f).")
      //              % index0 % x0 % y0 % p0.x % p0.y % p0.z);
      //      LOG4CXX_TRACE(logger, boost::format("[calculateOnRelation] relatum index: %d. (x,y) = (%d,%d). (x,y,z) = (%f,%f,%f).")
      //              % index1 % x1 % y1 % p1.x % p1.y % p1.z);
      distance = static_cast<float> (pcl::distances::l2(
              referent->getTrackingMask()->getObjectPoint(index0).getVector4fMap(),
              relatum->getTrackingMask()->getObjectPoint(index1).getVector4fMap()));
      //LOG4CXX_TRACE(logger, boost::format("[calculateOnRelation] distance: %f.") % distance);
      if (distance < distThresh) {
        ++onCount;
        break;
      }
    }
  }

  LOG4CXX_DEBUG(logger, boost::format("[checkForTouchingRelation] onCount: %f.") % onCount);
  if (onCount > 0) {
    LOG4CXX_DEBUG(logger, boost::format("[checkForTouchingRelation] adding %s relation between %s %lld and %s %lld.")
                          % relation.getName()
                          % referent->getValidationResultsString()
                          % referent->getId()
                          % relatum->getValidationResultsString()
                          % relatum->getId());

    //TODO: how should this "touching" confidence be calculated?
    referent->addRelation(0.95, relation.getName(), relatum);
    return true;
  } else {
    return false;
  }
}

bool SpatialRelationDetector::checkForOnTopRelation(const PredicateHelper &relation,
                                                    const diarc::stm::MemoryObject::Ptr &referent,
                                                    const diarc::stm::MemoryObject::Ptr &relatum) {
  return false;
}
