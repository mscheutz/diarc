/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "GenericTracker.hpp"
#include "stm/util/StmUtilities.hpp"

using namespace diarc::stm;

GenericTracker::GenericTracker(const long long &processorId, const int imgWidth,
                               const int imgHeight)
        : ObjectTracker(processorId, imgWidth, imgHeight) {
  logger = log4cxx::Logger::getLogger("diarc.tracker.GenericTracker");
}

GenericTracker::~GenericTracker() {
}

// we need a way to decay confidence when there are no new objects being
//detected. this is accomplished by decaying during each new frame.

void GenericTracker::haveNewImage(CaptureNotification::ConstPtr notification) {
  DescriptorsByTypeConstPtr descriptorsByType = getTypes();
  DescriptorsByType::const_iterator descriptorsByType_iter;
  for (descriptorsByType_iter = descriptorsByType->begin();
       descriptorsByType_iter != descriptorsByType->end(); ++descriptorsByType_iter) {
    MemoryObjects_HashedById_Set trackedObjectsByType = trackedObjects->getByTypeId(descriptorsByType_iter->first);
    MemoryObjects_HashedById_Set::const_iterator trackedObjectsByType_itr;

    //for each typeId to track, decay confidence
    for (trackedObjectsByType_itr = trackedObjectsByType.begin();
         trackedObjectsByType_itr != trackedObjectsByType.end(); ++trackedObjectsByType_itr) {

      // decay root node
      LOG4CXX_DEBUG(logger, boost::format("Decaying confidence for tokenID: %lld conf: %.2f.")
          % (*trackedObjectsByType_itr)->getId() % (*trackedObjectsByType_itr)->getTrackingConfidence());
      (*trackedObjectsByType_itr)->decayTrackingConfidence();
      LOG4CXX_DEBUG(logger, boost::format("Decayed confidence for tokenID: %lld conf: %.2f.")
          % (*trackedObjectsByType_itr)->getId() % (*trackedObjectsByType_itr)->getTrackingConfidence());

      // iterate through entire scene graph, decaying each node
      MemoryObject::Vec trackedSceneGraph = (*trackedObjectsByType_itr)->getChildren();
      MemoryObject::Vec::iterator trackedSceneGraph_itr;
      for (trackedSceneGraph_itr = trackedSceneGraph.begin();
           trackedSceneGraph_itr != trackedSceneGraph.end(); ++trackedSceneGraph_itr) {
        (*trackedSceneGraph_itr)->decayTrackingConfidence();
      }

      //send notifications that trackedMO was updated
      LOG4CXX_DEBUG(logger, "[haveNewImage] sending notifications for tracked object.");
      sendTrackingNotifications(*trackedObjectsByType_itr);
    }
  }
}


void GenericTracker::removeLowConfidenceObjects() {
  LOG4CXX_TRACE(logger, "[removeLowConfidenceObjects] method entered.");

  DescriptorsByTypeConstPtr descriptorsByType = getTypes();
  DescriptorsByType::const_iterator descriptorsByType_iter;
  long long typeId;
  for (descriptorsByType_iter = descriptorsByType->begin();
       descriptorsByType_iter != descriptorsByType->end(); ++descriptorsByType_iter) {

    //find tracked objects below given threshold
    typeId = descriptorsByType_iter->first;
    MemoryObjects_HashedById_Set trackedObjectsByTypeId = trackedObjects->getByTypeId(typeId);
    MemoryObjects_HashedById_Set::const_iterator trackedObjectsByType_itr;

    //find which ones to delete
    for (trackedObjectsByType_itr = trackedObjectsByTypeId.begin();
         trackedObjectsByType_itr != trackedObjectsByTypeId.end(); ++trackedObjectsByType_itr) {

      // check if scene graph root should still be tracked
      if (!shouldBeTracked(*trackedObjectsByType_itr)) {
        // stop tracking root of scene graph (and all children)
        LOG4CXX_DEBUG(logger, boost::format("[removeLowConfidenceObjects] removing root: %s.")
                              % (*trackedObjectsByType_itr)->getValidationResultsString());
        stopTracking(*trackedObjectsByType_itr);
      } else {
        // root should be tracked, now check all of its children

        // iterate through entire scene graph, checking if each node should still be tracked
        MemoryObject::Vec trackedSceneGraph = (*trackedObjectsByType_itr)->getChildren();
        MemoryObject::Vec::iterator trackedSceneGraph_itr;
        for (trackedSceneGraph_itr = trackedSceneGraph.begin();
             trackedSceneGraph_itr != trackedSceneGraph.end(); ++trackedSceneGraph_itr) {

          //if tracking or detection confidence is below threshold, remove object completely
          if (!shouldBeTracked(*trackedSceneGraph_itr)) {
            // stop tracking and remove node from scene graph
            LOG4CXX_DEBUG(logger, boost::format("[removeLowConfidenceObjects] removing child: %s.")
                                  % (*trackedSceneGraph_itr)->getValidationResultsString());
            (*trackedSceneGraph_itr)->stopTracking(false);
            (*trackedObjectsByType_itr)->removeChild(*trackedSceneGraph_itr);
          }

        }
      }
    }
  }
}

MemoryObject::Ptr GenericTracker::mergeObject(MemoryObject::Ptr newObject) {
  LOG4CXX_TRACE(logger, boost::format("[mergeObject] with frame number: %lld") % newObject->getTypeId());
  MemoryObject::Ptr matchedObject;
  long long int typeId = newObject->getTypeId();
  const auto& trackedObjectsByType = trackedObjects->getByTypeId(typeId);

  // iterate through all existing tracked objects of moTypeId and try to "match"
  // with newly detected objects and update all existing MemoryObjects of typeId
  for (const auto& trackedObject : trackedObjectsByType) {

    // first check to see if top-level scene graphs are a match
    if (compare(trackedObject, newObject)) {
      // top-level scene graphs match -- now iterate through scene graphs to update/add nodes in tracked object
      matchedObject = trackedObject;

      // update root node
      trackedObject->addNewTrackingResult(newObject->getTrackingMask(),
                                                        trackedObject->getTrackingConfidence() * 1.3f);

      // iterate through entire scene graph, trying to find matching nodes in the tracked object
      for (const auto& newSceneGraphNode : newObject->getChildren()) {

        bool nodeMatch = false;
        for (const auto& trackedSceneGraphNode : trackedObject->getChildren()) {
          // try to match with newly detected MemoryObject
          if (compare(trackedSceneGraphNode, newSceneGraphNode)) {
            //match found, update info and conf
            trackedSceneGraphNode->addNewTrackingResult(newSceneGraphNode->getTrackingMask(),
                                                           trackedSceneGraphNode->getTrackingConfidence() * 1.3f);

            // stop looking for node match
            nodeMatch = true;
            break;
          }
        }

        // if no node match found, add new node to existing tracked scene graph
        if (!nodeMatch) {
          // start tracking as a non-root node (i.e., false)
          newSceneGraphNode->startTracking(false);

          // add new node to root node
          LOG4CXX_DEBUG(logger, boost::format("[mergeObject] adding child: %s.")
                                % newSceneGraphNode->getValidationResultsString());
          trackedObject->addChild(newSceneGraphNode);
        }
      }
    } else {
      // just print some debug info
      LOG4CXX_DEBUG(logger, boost::format("[mergeObject] top-level didn't match: %s %s.")
                           % trackedObject->getValidationResultsString()
                           % newObject->getValidationResultsString());
    }

    if (matchedObject) {
      // stop looking for a match
      return matchedObject;
    }
  }

  // if top-level scene graphs didn't match -- add newObject to tracker as root of scene graph
  if (trackedObjectsByType.empty()) {
    LOG4CXX_DEBUG(logger, "[mergeObject] no tracked objects to compare against.");
  }

  LOG4CXX_DEBUG(logger, boost::format("[mergeObject] adding new top-level object to tracker (and it's children): %s.")
                % newObject->getValidationResultsString());

  newObject->startTracking();
  return newObject;

}

bool GenericTracker::compare(const MemoryObject::Ptr &mo1, const MemoryObject::Ptr &mo2) {
  // TODO: make this more sophisticated
  bool match = mo1->getValidationResults().containsAllDescriptors(mo2->getValidationResults().getDescriptors());

  float overlap = diarc::stm::util::calculateBoundBoxOverlap(mo1->getTrackingMask()->getBoundingBox(),
                                                           mo2->getDetectionMask()->getBoundingBox());
  return match && (overlap > 0.0);
}

bool GenericTracker::shouldBeTracked(const MemoryObject::Ptr &mo) {
  if (mo->getTrackingConfidence() <= trackingConfidenceThreshold) {
    if (logger->isTraceEnabled()) {
      LOG4CXX_TRACE(logger, boost::format(
              "[removeLowConfidenceObjects] removing tracked object %lld bc of tracking confidence: %f.")
                            % mo->getId() % mo->getTrackingConfidence());
    }

    return false;
  } else if (mo->getDetectionConfidence() <= detectionConfidenceThreshold) {
    // else if detection confidence is below detection threshold, only remove
    // object from tracking queue as that particular type (same object can be
    // multiple types at once)
    if (logger->isTraceEnabled()) {
      LOG4CXX_TRACE(logger, boost::format(
              "[removeLowConfidenceObjects] removing tracked object %lld bc of detection confidence: %f.")
                            % mo->getId() % mo->getDetectionConfidence());
    }

    return false;
  }

  return true;
}