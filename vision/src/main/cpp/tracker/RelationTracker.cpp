/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "RelationTracker.hpp"

using namespace ade::stm;

RelationTracker::RelationTracker(const long long &processorId, const int imgWidth,
                                 const int imgHeight)
        : ObjectTracker(processorId, imgWidth, imgHeight) {
  logger = log4cxx::Logger::getLogger("ade.tracker.RelationTracker");
}

RelationTracker::~RelationTracker() {
}

// we need a way to decay confidence when there are no new objects being
//detected. this is accomplished by decaying each new frame.
void RelationTracker::haveNewImage(CaptureNotification::ConstPtr notification) {
  //return;
  LOG4CXX_DEBUG(logger, "[haveNewImage] method entered.");
  DescriptorsByTypeConstPtr types = getTypes();
  DescriptorsByType::const_iterator types_itr;
  PredicateHelper::Set::const_iterator descriptors_itr;
  for (types_itr = types->begin(); types_itr != types->end(); ++types_itr) {
    MemoryObjects_HashedById_Set trackedObjectsByType = trackedObjects->getByTypeId(types_itr->first);
    MemoryObjects_HashedById_Set::const_iterator existingObjectIter;

    //for each typeId to track, decay relation confidence of all nodes in scene graph that match root node's variable name
    for (existingObjectIter = trackedObjectsByType.begin(); existingObjectIter != trackedObjectsByType.end(); ++existingObjectIter) {
      for (descriptors_itr = types_itr->second.begin(); descriptors_itr != types_itr->second.end(); ++descriptors_itr) {
        MemoryObject::Vec sceneGraphNodes = (*existingObjectIter)->getMemoryObjects((*existingObjectIter)->getVariableName());
        MemoryObject::Vec::iterator sceneGraphNodes_itr;
        for (sceneGraphNodes_itr = sceneGraphNodes.begin(); sceneGraphNodes_itr != sceneGraphNodes.end(); ++sceneGraphNodes_itr) {
          LOG4CXX_DEBUG(logger, boost::format("[haveNewImage] decaying relation conf. relationName: %s. thisId: %lld.")
                                % descriptors_itr->getName() % (*sceneGraphNodes_itr)->getId());
          (*sceneGraphNodes_itr)->decayRelationConfidence(descriptors_itr->getName());
        }
      }
    }
  }
}

void RelationTracker::haveTrackedMemoryObject(ade::stm::MemoryObject::Ptr trackedObject) {
  LOG4CXX_DEBUG(logger, "[haveTrackedMemoryObject] method entered.");
  // relation tracker only gets notified of MemoryObjects that are already being tracked (by a different
  // tracker), so "haveNewMemoryObject" will never be called. this simply forwards the notification to the
  // "haveNewMemoryObject" method so MemoryObjects can be handled as if they're not tracked by this tracker (since
  // they're not. TODO: modifying the isTracked check in ObjectTracker::handleMemoryObjectNotification to explicitly
  // check if *this* tracker is tracking the object would get rid of this work around. that would also require
  // MemoryObjects to be able to have more than one typeId.
  haveNewMemoryObject(trackedObject);
}

int RelationTracker::mergeObject(MemoryObject::Ptr newObject) {
  LOG4CXX_DEBUG(logger, "[mergeObject] method entered.");

  DescriptorsByTypeConstPtr types = getTypes();
  DescriptorsByType::const_iterator types_itr;
  MemoryObjects_HashedById_Set trackedObjectsByType;
  MemoryObjects_HashedById_Set::const_iterator trackedObjectsByType_itr;
  long long typeId;
  bool alreadyTracked = false;
  for (types_itr = types->begin(); types_itr != types->end(); ++types_itr) {
    typeId = types_itr->first;
    trackedObjectsByType = trackedObjects->getByTypeId(typeId);

    // iterate through all existing tracked objects of moTypeId and try to "match"
    // with newly detected objects and update all existing MemoryObjects of moTypeId
    for (trackedObjectsByType_itr = trackedObjectsByType.begin();
         trackedObjectsByType_itr != trackedObjectsByType.end(); ++trackedObjectsByType_itr) {

      // try to match with newly detected MemoryObject
      if ((*trackedObjectsByType_itr).get() == newObject.get()) {
        alreadyTracked = true;
        break;
      }
    }

    if (alreadyTracked) {
      // break from outer loop
      break;
    }
  }

  if (alreadyTracked) {
    // no need to update relation confidences if already being tracked, relation detector already did that
    return 0;
  } else {
    // add newObject to trackedObjects (by typeId only)
    LOG4CXX_DEBUG(logger, boost::format("Adding spatial relation typeId: %lld. tokenId: %lld.")
                          % typeId % newObject->getId());
    trackedObjects->add(newObject, typeId);
    return 1;
  }
}

void RelationTracker::removeLowConfidenceObjects() {
  //return;
  LOG4CXX_DEBUG(logger, "[removeLowConfidenceObjects] method entered.");

  DescriptorsByTypeConstPtr types = getTypes();
  DescriptorsByType::const_iterator types_itr;
  long long typeId;
  bool stopTracking = false;
  int relationOccurences = 0;
  for (types_itr = types->begin();
       types_itr != types->end(); ++types_itr) {

    //find tracked objects with relations below given threshold
    typeId = types_itr->first;
    MemoryObjects_HashedById_Set trackedObjectsByType = trackedObjects->getByTypeId(typeId);
    MemoryObjects_HashedById_Set::const_iterator trackedObjects_itr;

    //find which relations to delete by looking through all tracked objects for tracker's typeId
    for (trackedObjects_itr = trackedObjectsByType.begin();
         trackedObjects_itr != trackedObjectsByType.end(); ++trackedObjects_itr) {

      // search through all tracker's relations
      PredicateHelper::Set::const_iterator descriptors_itr;
      for (descriptors_itr = types_itr->second.begin(); descriptors_itr != types_itr->second.end(); ++descriptors_itr) {

        // reset count for each relation
        relationOccurences = 0;

        // search through all nodes in tracked object scene graph that match tracked object's variable name
        MemoryObject::Vec sceneGraphNodes = (*trackedObjects_itr)->getMemoryObjects(
                (*trackedObjects_itr)->getVariableName());
        MemoryObject::Vec::iterator sceneGraphNodes_itr;
        for (sceneGraphNodes_itr = sceneGraphNodes.begin();
             sceneGraphNodes_itr != sceneGraphNodes.end(); ++sceneGraphNodes_itr) {

          //(*sceneGraphNodes_itr)->decayRelationConfidence(descriptors_itr->getName());

          //const RelationValidationResult::Vec &preRelations = (*trackedObjects_itr)->getRelations(
          //        descriptors_itr->getName());
          //LOG4CXX_DEBUG(logger, boost::format("[removeLowConfidenceObjects] pre-removal %s size: %lu.")
          //                      % descriptors_itr->getName() % preRelations.size());

          // remove low confidence relations
          //LOG4CXX_DEBUG(logger, boost::format("Removing low conf relations from scene graph. typeId: %lld. tokenId: %lld.")
          //                      % typeId % (*sceneGraphNodes_itr)->getId());
          (*sceneGraphNodes_itr)->removeRelations(descriptors_itr->getName(), trackingConfidenceThreshold);

          // keep track of how many of that relation exist in the scene graph
          relationOccurences += (*sceneGraphNodes_itr)->getRelations(descriptors_itr->getName()).size();

          //LOG4CXX_DEBUG(logger, boost::format("[removeLowConfidenceObjects] post-removal %s size: %lu.")
          //                      % descriptors_itr->getName() % relationOccurances);
        }

        // if no relations exist, remove from tracker
        if (relationOccurences == 0) {
          stopTracking = true;
          break;
        }
      }

      if (stopTracking) {
        LOG4CXX_DEBUG(logger, boost::format("Removing all spatial relation from scene graph. typeId: %lld. tokenId: %lld.")
                              % typeId % (*trackedObjects_itr)->getId());
        // remove all relations, because tracker will no longer track the relations (only tracks if all relations are
        // present) and they will persist for the life of the MemoryObject (i.e., will never be decayed/removed/updated)
        // TODO: EAK: is this necessary? aren't all relations already gone?
        for (descriptors_itr = types_itr->second.begin();
             descriptors_itr != types_itr->second.end(); ++descriptors_itr) {


          MemoryObject::Vec sceneGraphNodes = (*trackedObjects_itr)->getMemoryObjects(
                  (*trackedObjects_itr)->getVariableName());
          MemoryObject::Vec::iterator sceneGraphNodes_itr;
          for (sceneGraphNodes_itr = sceneGraphNodes.begin();
               sceneGraphNodes_itr != sceneGraphNodes.end(); ++sceneGraphNodes_itr) {
            LOG4CXX_DEBUG(logger, boost::format("Removing all spatial relation of type: %s. typeId: %lld. tokenId: %lld.")
                                  % descriptors_itr->getName() % (*sceneGraphNodes_itr)->getTypeId());
            (*sceneGraphNodes_itr)->removeRelations(descriptors_itr->getName());
          }
        }

        // remove from tracked objects
        LOG4CXX_DEBUG(logger, boost::format("Removing spatial relation from tracked objects. typeId: %lld. tokenId: %lld.")
                              % typeId % (*trackedObjects_itr)->getId());
        trackedObjects->remove((*trackedObjects_itr), typeId);
      }

    }

  }
}