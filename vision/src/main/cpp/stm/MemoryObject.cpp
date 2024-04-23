/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "MemoryObject.hpp"

#include "util/StmUtilities.hpp"
#include "TrackedObjects.hpp"
#include "visionproc/VisionProcess.hpp"
#include "display/Display.hpp"
#include "capture/util/CaptureUtilities.hpp"

#include <boost/make_shared.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <math.h>
#include <cmath>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/pcl_base.h>
#include <pcl/common/centroid.h>

using namespace ade::stm;

//static object(s)
log4cxx::LoggerPtr MemoryObject::logger(log4cxx::Logger::getLogger("ade.stm.MemoryObject"));
ObjectId MemoryObject::trackingIdGenerator;
TrackedObjects *MemoryObject::trackedObjects = TrackedObjects::getInstance();

MemoryObject::MemoryObject(const long long &typeID, const std::string &variable,
                           CaptureData::ConstPtr capture, MemoryObjectMask::ConstPtr mask)
        : id(-1),
          typeId(typeID),
          variableName(variable),
          captureData(capture),
          mask(mask),
          validationResults(),
          isBeingTracked(false),
          trackingConfidence(),
          trackingHistoryLength(40) {
  // add detection mask to tracking queue as first tracking result
  trackingHistory.push_front(mask);
}

//MemoryObject::MemoryObject(const long long& typeID, const std::string& variable,
//CaptureData::ConstPtr capture, const cv::Mat& imageMask)
//: id(-1),
//typeId(typeID),
//variableName(variable),
//captureData(capture),
//mask(new MemoryObjectMask(capture, imageMask)),
//validationResults(),
//isBeingTracked(false),
//trackingConfidence(),
//trackingHistoryLength(40) {
//    // add detection mask to tracking queue as first tracking result
//    trackingHistory.push_front(mask);
//}

MemoryObject::MemoryObject(const long long &typeID, const std::string &variable,
                           CaptureData::ConstPtr capture, const cv::Rect &boundingBoxMask)
        : id(-1),
          typeId(typeID),
          variableName(variable),
          captureData(capture),
          mask(new MemoryObjectMask(capture, boundingBoxMask)),
          validationResults(),
          isBeingTracked(false),
          trackingConfidence(),
          trackingHistoryLength(40) {
  // add detection mask to tracking queue as first tracking result
  trackingHistory.push_front(mask);
}

MemoryObject::MemoryObject(const long long &typeID, const std::string &variable,
                           CaptureData::ConstPtr capture, const std::vector<int> &imageIndicesMask)
        : id(-1),
          typeId(typeID),
          variableName(variable),
          captureData(capture),
          mask(new MemoryObjectMask(capture, imageIndicesMask)),
          validationResults(),
          isBeingTracked(false),
          trackingConfidence(),
          trackingHistoryLength(40) {
  // add detection mask to tracking queue as first tracking result
  trackingHistory.push_front(mask);
}

MemoryObject::~MemoryObject() {
  // remove all relations
  LOG4CXX_DEBUG(logger, boost::format("[~MemoryObject] method entered for thisId: %lld. with validation: %s.")
                        % id % this->getValidationResultsString());
  // TODO: why does "this->removeRelations();" crash things. something with weak_ptrs
  LOG4CXX_DEBUG(logger, "[~MemoryObject] clearing all relations and children...");
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  //this->removeRelations();
  relations.clear();
  children.clear();
  LOG4CXX_DEBUG(logger, "[~MemoryObject] done removing all relations and children.");
}

//this method is pure virtual, but this provides a quick and dirty implementation
//that can be called explicitly from the derived class method.

double MemoryObject::compare(const MemoryObject::Ptr &other) const {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);

  //TODO: check this?
  //  if (typeId != other->getTypeId()) {
  //    LOG4CXX_ERROR(logger, "[compare] objects have different typeId.");
  //    return 0.0;
  //  }

  // make sure other mo has all the same descriptors as this object
  if (!other->getValidationResults().containsAllDescriptors(this->getValidationResults().getDescriptors())) {
    LOG4CXX_DEBUG(logger, "[compare] objects have different descriptors. Not comparing.");
    return 0.0;
  }

  // find closest tracking result (in time)
  const unsigned long long &detectFrameNum = other->getDetectionMask()->getFrameNumber();
  int minIndex = 0;
  int maxIndex = trackingHistory.size() - 1;
  int index = maxIndex / 2;
  int bestIndex = -1;
  long long diff = 0;
  long long minDiff = INT_MAX;
  MemoryObjectMask::ConstPtr match;
  do {
    const unsigned long long &trackFrameNum = trackingHistory[index]->getFrameNumber();
    diff = detectFrameNum - trackFrameNum;

    if (std::abs(diff) < minDiff) {
      minDiff = std::abs(diff);
      bestIndex = index;
      match = trackingHistory[index];
    }

    LOG4CXX_TRACE(logger, boost::format(
            "[compare] index: %d. min: %d. max: %d. frame diff: %lld. trackFrameNum: %llu. detectFrameNum: %llu.") %
            index % minIndex % maxIndex % diff % trackFrameNum % detectFrameNum);

    // update index -- if pos, move left (i.e, decrease index)
    if (diff > 0) {
      maxIndex = index - 1;
      index -= (maxIndex - minIndex + 1) / 2;
      index = (index > maxIndex) ? maxIndex : index;
    } else {
      minIndex = index + 1;
      index += (maxIndex - minIndex + 1) / 2;
      index = (index < minIndex) ? minIndex : index;
    }

  } while (diff != 0 && minIndex <= maxIndex);

  LOG4CXX_DEBUG(logger, boost::format("[compare] best index: %d. frame num diff: %lld") % bestIndex % minDiff);

  //compare two bounding boxes
  static const float conf_thresh = 0.3;
  float overlap = ade::stm::util::calculateBoundBoxOverlap(match->getBoundingBox(),
                                                           other->getDetectionMask()->getBoundingBox());
  float sizeSim = ade::stm::util::calculateBoundBoxSizeSimilarity(match->getBoundingBox(),
                                                                  other->getDetectionMask()->getBoundingBox());
  float confidence = overlap * sizeSim;
  LOG4CXX_DEBUG(logger,
                boost::format("[compare] thisId: %lld. otherId: %lld. overlap: %f. sizeSim: %f. conf: %f. match: %s.")
                % id % other->getId() % overlap % sizeSim % confidence %
                ((overlap > conf_thresh) ? "true" : "false"));
  //((confidence > conf_thresh) ? "true" : "false"));

  //return (confidence > conf_thresh) ? 1.0 : 0.0;
  return (overlap > conf_thresh) ? 1.0 : 0.0;
}

void MemoryObject::update(const MemoryObject::Ptr &match) {
  //not the same type
  //TODO: check something about this?
  //  if (typeId != match->getTypeId()) {
  //    LOG4CXX_ERROR(logger, "[update] objects have different typeId.");
  //    return;
  //  }

  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);

  //TODO: probably need to merge the old and new and not just use new here
  captureData = match->getCaptureData();
  mask = match->getDetectionMask();
  validationResults = getValidationResults();

  // update entire scene graph
  LOG4CXX_DEBUG(logger, "[update] updating scene graph relations.");
  relations.clear();
  RelationValidationResult::Vec matchRelations = match->getRelations();
  RelationValidationResult::Vec::const_iterator matchRelations_itr = matchRelations.begin();
  for (; matchRelations_itr != matchRelations.end(); ++matchRelations_itr) {
    addRelation((*matchRelations_itr)->getConfidence(), (*matchRelations_itr)->getDescriptor(),
                (*matchRelations_itr)->getRelatedObject());
  }

  LOG4CXX_DEBUG(logger, "[update] updating scene graph children.");
  children.clear();
  Vec matchChildren = match->getChildren();
  Vec::const_iterator matchChildren_itr = matchChildren.begin();
  for (; matchChildren_itr != matchChildren.end(); ++matchChildren_itr) {
    addChild(*matchChildren_itr);

    // start tracking children if parent is being tracked
    // this ensures children have a valid tokenId
    if (isBeingTracked) {
      (*matchChildren_itr)->startTracking(false);
    }
  }

  LOG4CXX_DEBUG(logger, "[update] done updating scene graph.");
}

void MemoryObject::startTracking(bool root) {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  LOG4CXX_DEBUG(logger, "[startTracking] have lock, trying to start tracking.");

  if (!isBeingTracked) {
    //set unique token id
    if (id == -1) {
      id = trackingIdGenerator.getNext();
    } else {
      LOG4CXX_WARN(logger, "[startTracking] Token ID already assigned.");
    }

    //add to necessary TrackedObject containers
    if (root) {
      trackedObjects->add(shared_from_this());
    }

    //set tracked flag
    isBeingTracked = true;

    // call start tracking on all children so that they get a unique token id
    MemoryObject::Vec::iterator graph_itr;
    MemoryObject::Vec childrenVec = getChildren();
    for (graph_itr = childrenVec.begin(); graph_itr != childrenVec.end(); ++graph_itr) {
      (*graph_itr)->startTracking(false);
    }
    LOG4CXX_DEBUG(logger,
                  boost::format("[startTracking] starting tracking. typeId: %lld. tokenId: %lld.") % typeId % id);
  } else {
    LOG4CXX_ERROR(logger, "[startTracking] Already being tracked.");
  }
}

void MemoryObject::stopTracking(bool root) {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  LOG4CXX_DEBUG(logger, "[stopTracking] have lock, trying to stop tracking.");

  if (isBeingTracked) {
    //remove from necessary TrackedObject containers
    if (root) {
      trackedObjects->remove(shared_from_this());
    }

    //set tracked flag
    isBeingTracked = false;

    // call stop tracking on all children
    MemoryObject::Vec::iterator graph_itr;
    MemoryObject::Vec childrenVec = getChildren();
    for (graph_itr = childrenVec.begin(); graph_itr != childrenVec.end(); ++graph_itr) {
      (*graph_itr)->stopTracking(false);
    }

    LOG4CXX_DEBUG(logger, "[stopTracking] stopped tracking.");
  } else {
    LOG4CXX_ERROR(logger, "[stopTracking] Not being tracked.");
  }
}

bool MemoryObject::isTracked() const {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);

  return isBeingTracked;
}

void MemoryObject::decayTrackingConfidence() {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  trackingConfidence.decay();
}

void MemoryObject::setTrackingConfidenceDecayRate(const float& decayRatePerMilsec) {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  trackingConfidence.setDecayRate(decayRatePerMilsec);
}

void MemoryObject::addNewTrackingResult(MemoryObjectMask::ConstPtr mask, const float &confidence) {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);

  trackingConfidence.setLevel(confidence);

  if (trackingHistory.size() < trackingHistoryLength) {
    trackingHistory.push_front(mask);
  } else {
    trackingHistory.pop_back(); // remove oldest
    trackingHistory.push_front(mask);
  }
}

void MemoryObject::addChild(MemoryObject::Ptr child) {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);

  std::string variableName = child->getVariableName();
  std::tr1::unordered_map<std::string, MemoryObject::Vec>::iterator children_itr = children.find(variableName);
  if (children_itr == children.end()) {
    MemoryObject::Vec childVec;
    childVec.push_back(child);
    children.insert(std::tr1::unordered_map<std::string, MemoryObject::Vec>::value_type(variableName, childVec));
  } else {
    children_itr->second.push_back(child);
  }
}


void MemoryObject::removeChild(MemoryObject::Ptr child) {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);

  std::string variableName = child->getVariableName();
  std::tr1::unordered_map<std::string, MemoryObject::Vec>::iterator children_itr = children.find(variableName);
  if (children_itr != children.end()) {
    MemoryObject::Vec::iterator childrenByVar_itr;
    for (childrenByVar_itr = children_itr->second.begin(); childrenByVar_itr != children_itr->second.end();
         ++childrenByVar_itr) {
      if (child.get() == (*childrenByVar_itr).get()) {
        // found child

        // remove all relations it might have
        child->removeRelations();

        // remove child
        children_itr->second.erase(childrenByVar_itr);
        return;
      }
    }
  }

}

void MemoryObject::addRelation(const float &confidence, const PredicateHelper &descriptor,
                               MemoryObject::Ptr relatedObject, bool reciprocate) {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  LOG4CXX_DEBUG(logger, boost::format("[addRelation] descriptor: %s. confidence: %f. thisId: %lld. otherId: %lld.")
                        % descriptor.toString() % confidence % id % relatedObject->getId());

  std::string relationName = descriptor.getName();
  std::tr1::unordered_map<std::string, RelationValidationResult::Vec>::iterator relations_itr = relations.find(
          relationName);


  RelationValidationResult::Ptr relation(new RelationValidationResult(confidence, descriptor, relatedObject));
  if (relations_itr == relations.end()) {
    RelationValidationResult::Vec relationsVec;
    relationsVec.push_back(relation);
    relations[relationName] = relationsVec;
  } else {
    // find if relation already exists between MemoryObjects
    bool foundRelation = false;
    RelationValidationResult::Vec::iterator relationVals_itr;
    for (relationVals_itr = relations_itr->second.begin();
         relationVals_itr != relations_itr->second.end(); ++relationVals_itr) {
      if ((*relationVals_itr)->getRelatedObject().get() == relatedObject.get()) {
        foundRelation = true;
        break;
      }
    }

    if (foundRelation) {
      // already have that relation, just update it's confidence
      (*relationVals_itr)->setConfidence(confidence);
    } else {
      relations_itr->second.push_back(relation);
    }
  }

  // also add relation to relatedObject
  if (reciprocate) {
    relatedObject->addRelation(confidence, descriptor, shared_from_this(), false);
  }
}

void MemoryObject::decayRelationConfidence(const std::string &relationName, const MemoryObject::Ptr &relatedObject) {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  if (relatedObject) {
    LOG4CXX_DEBUG(logger, boost::format("[decayRelationConfidence] relationName: %s. thisId: %lld. otherId: %lld.")
                          % relationName % id % relatedObject->getId());
  } else {
    LOG4CXX_DEBUG(logger, boost::format("[decayRelationConfidence] relationName: %s. thisId: %lld. other: null.")
                          % relationName % id);
  }

  std::tr1::unordered_map<std::string, RelationValidationResult::Vec>::iterator relations_itr = relations.find(
          relationName);
  if (relations_itr != relations.end()) {
    RelationValidationResult::Vec::iterator relationVals_itr;
    for (relationVals_itr = relations_itr->second.begin();
         relationVals_itr != relations_itr->second.end(); ++relationVals_itr) {

      // if relatedObject is specified, only decay relation to that object (and don't recurse),
      // otherwise decay relation confidence to all relations of specified type
      if (relatedObject) {
        if ((*relationVals_itr)->getRelatedObject().get() == relatedObject.get()) {
          (*relationVals_itr)->decayConfidence();
          break;
        }
      } else {
        (*relationVals_itr)->decayConfidence();
        MemoryObject::Ptr theRelatedObject = (*relationVals_itr)->getRelatedObject();
        if (theRelatedObject) {
          theRelatedObject->decayRelationConfidence(relationName, shared_from_this());
        } else {
          LOG4CXX_DEBUG(logger, "[decayRelationConfidence] weak_ptr to related object is null.");
        }
      }
    }
  }
}

void MemoryObject::removeRelations(const ade::stm::MemoryObject::Ptr &relatedObject) {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  if (relatedObject) {
    LOG4CXX_DEBUG(logger, boost::format("[removeRelations] thisId: %lld. otherId: %lld.")
                          % id % relatedObject->getId());
  } else {
    LOG4CXX_DEBUG(logger, boost::format("[removeRelations] thisId: %lld. other: null.") % id);
  }

  std::tr1::unordered_map<std::string, RelationValidationResult::Vec>::iterator relations_itr;
  RelationValidationResult::Vec::iterator relationVals_itr;
  for (relations_itr = relations.begin(); relations_itr != relations.end(); ++relations_itr) {

    for (relationVals_itr = relations_itr->second.begin();
         relationVals_itr != relations_itr->second.end(); ++relationVals_itr) {

      // if relatedObject is specified, only remove relation to that object (i.e., single edge, and don't recurse),
      // otherwise remove all relations of specified type
      if (relatedObject) {
        if ((*relationVals_itr)->getRelatedObject().get() == relatedObject.get()) {
          relations_itr->second.erase(relationVals_itr);
          break;
        }
      } else {
        MemoryObject::Ptr theRelatedObject = (*relationVals_itr)->getRelatedObject();
        if (theRelatedObject) {
          theRelatedObject->removeRelations(shared_from_this());
        } else {
          LOG4CXX_DEBUG(logger, "[removeRelations] weak_ptr to related object is null.");
        }
      }
    }
  }

  // finally, clear all relations on this MemoryObject if the relatedObject has not been specified
  if (!relatedObject) {
    relations.clear();
  }
}

void MemoryObject::removeRelations(const std::string &relationName, const float &confidenceThresh,
                                   const MemoryObject::Ptr &relatedObject) {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);

  if (relatedObject) {
    LOG4CXX_DEBUG(logger,
                  boost::format("[removeRelations] relationName: %s. confidenceThresh %f. thisId: %lld. otherId: %lld.")
                  % relationName % confidenceThresh % id % relatedObject->getId());
  } else {
    LOG4CXX_DEBUG(logger,
                  boost::format("[removeRelations] relationName: %s. confidenceThresh %f. thisId: %lld. other: null.")
                  % relationName % confidenceThresh % id);
  }

  std::tr1::unordered_map<std::string, RelationValidationResult::Vec>::iterator relations_itr = relations.find(
          relationName);
  if (relations_itr != relations.end()) {
    RelationValidationResult::Vec::iterator relationVals_itr;
    bool erased;
    for (relationVals_itr = relations_itr->second.begin(); relationVals_itr != relations_itr->second.end();) {
      erased = false;

      // if relatedObject is specified, only remove relation to that object (i.e., single edge, and don't recurse),
      // otherwise remove all relations of specified type
      if (relatedObject) {
        if ((*relationVals_itr)->getRelatedObject().get() == relatedObject.get()
            && (*relationVals_itr)->getConfidence() < confidenceThresh) {
          relations_itr->second.erase(relationVals_itr);
          break;
        }
      } else {
        if ((*relationVals_itr)->getConfidence() < confidenceThresh) {
          // first remove relation from relatedObject
          MemoryObject::Ptr theRelatedObject = (*relationVals_itr)->getRelatedObject();
          if (theRelatedObject) {
            theRelatedObject->removeRelations(relationName, confidenceThresh, shared_from_this());
          } else {
            LOG4CXX_DEBUG(logger, "[removeRelations] weak_ptr to related object is null.");
          }

          // deleting during iteration -- this moves itr to next
          relationVals_itr = relations_itr->second.erase(relationVals_itr);
          erased = true;
        }
      }

      if (!erased) {
        ++relationVals_itr;
      }
    }

  }
}

void MemoryObject::removeRelations(const std::string &relationName, const MemoryObject::Ptr &relatedObject) {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);

  if (relatedObject) {
    LOG4CXX_DEBUG(logger, boost::format("[removeRelations] relationName: %s. thisId: %lld. otherId: %lld.")
                          % relationName % id % relatedObject->getId());
  } else {
    LOG4CXX_DEBUG(logger, boost::format("[removeRelations] relationName: %s. thisId: %lld. other: null.")
                          % relationName % id);
  }

  std::tr1::unordered_map<std::string, RelationValidationResult::Vec>::iterator relations_itr = relations.find(
          relationName);
  if (relations_itr != relations.end()) {
    RelationValidationResult::Vec::iterator relationVals_itr;
    for (relationVals_itr = relations_itr->second.begin();
         relationVals_itr != relations_itr->second.end(); ++relationVals_itr) {

      // if relatedObject is specified, only remove relation to that object (i.e., single edge, and don't recurse),
      // otherwise remove all relations of specified type
      if (relatedObject) {
        if ((*relationVals_itr)->getRelatedObject().get() == relatedObject.get()) {
          relations_itr->second.erase(relationVals_itr);
          break;
        }
      } else {
        MemoryObject::Ptr theRelatedObject = (*relationVals_itr)->getRelatedObject();
        if (theRelatedObject) {
          theRelatedObject->removeRelations(relationName, shared_from_this());
        } else {
          LOG4CXX_DEBUG(logger, "[removeRelations] (relationName) weak_ptr to related object is null.");
        }
      }
    }

    if (!relatedObject) {
      relations.erase(relations_itr);
    }

  }
}

void MemoryObject::addValidationResult(const float &confidence, const PredicateHelper &descriptor) {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);

  if (logger->isTraceEnabled()) {
    LOG4CXX_TRACE(logger, boost::format("[addValidationResult] typeId: %lld. descriptor: %s. conf: %f.")
                          % typeId % descriptor.toString() % confidence);
  }

  ValidationResult::Ptr validationResult = ValidationResult::Ptr(new ValidationResult(confidence, descriptor, mask));
  validationResults.addValidationResult(validationResult);
}

void MemoryObject::addValidationResult(ValidationResult::ConstPtr validationResult) {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);

  if (logger->isTraceEnabled()) {
    LOG4CXX_TRACE(logger, boost::format("[addValidationResult] typeId: %lld. descriptor: %s. conf: %f.")
                          % typeId % validationResult->getDescriptor().toString() % validationResult->getConfidence());
  }

  validationResults.addValidationResult(validationResult);
}

void MemoryObject::addValidationResult(const float &confidence, const PredicateHelper &descriptor,
                                       const cv::Mat_<float> &imageMask) {

  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);

  ValidationResult::Ptr newValidationResult(new ValidationResult(confidence, descriptor, captureData, imageMask));
  addValidationResult(newValidationResult);
}

void MemoryObject::addValidationResult(const float &confidence, const PredicateHelper &descriptor,
                                       const std::vector<int> &indicesMask) {

  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);

  ValidationResult::Ptr newValidationResult(new ValidationResult(confidence, descriptor, captureData, indicesMask));
  addValidationResult(newValidationResult);
}

long long MemoryObject::getId() const {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  return id;
}

long long MemoryObject::getTypeId() const {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  return typeId;
}

std::string MemoryObject::getVariableName() const {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  return variableName;
}

MemoryObjectMask::ConstPtr MemoryObject::getDetectionMask() const {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  return mask;
}

MemoryObjectMask::ConstPtr MemoryObject::getTrackingMask() const {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  return trackingHistory.front();
}

float MemoryObject::getTrackingConfidence() const {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  return trackingConfidence.getLevel();
}

float MemoryObject::getDetectionConfidence() const {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  // TODO: this should include children and relation confidence values too
  return validationResults.getConfidence();
}

//float MemoryObject::getDetectionConfidence(const PredicateHelper::Set &descriptors) const {
//  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
//
//  float confidence = 1.0;
//  bool foundAll = true;
//  PredicateHelper::Set::const_iterator descriptors_itr;
//  for (descriptors_itr = descriptors.begin(); descriptors_itr != descriptors.end(); ++descriptors_itr) {
//
//    if (descriptors_itr->getNumArgs() == 1) {
//      std::string varName = descriptors_itr->getArg(0);
//
//    } else if (descriptors_itr->getNumArgs() == 2) {
//      std::string varName0 = descriptors_itr->getArg(0);
//      std::string varName1 = descriptors_itr->getArg(1);
//
//    } else {
//      LOG4CXX_ERROR(logger, boost::format("[getDetectionConfidence] can't handle descriptor: ")
//                            % descriptors_itr->toString());
//      foundAll = false;
//      confidence = 0.0;
//      break;
//    }
//
//    // first check validations
//    if (validationResults.containsDescriptor(*descriptors_itr)) {
//      confidence *= validationResults.getConfidence(*descriptors_itr);
//    } else if () {
//
//    } else if () {
//
//    } else {
//      confidence = 0.0;
//      break;
//    }
//  }
//
//  if (!foundAll && logger->isTraceEnabled()) {
//    std::string requested;
//    PredicateHelper::Set::const_iterator itr;
//    for (itr = descriptors.begin(); itr != descriptors.end(); ++itr) {
//      requested.append(itr->toString());
//      requested.append(" ");
//    }
//    LOG4CXX_TRACE(logger, boost::format(
//            "[getDetectionConfidence] Doesn't contain all descriptors, returning 0. Object: %s. Requested: %s.")
//                          % validationResults.getResultsString() % requested);
//  }
//
//  return confidence;
//}

float MemoryObject::getDetectionConfidence(const PredicateHelper::Set &descriptors) const {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  LOG4CXX_DEBUG(logger, "[getDetectionConfidence] method entered.");

  // traverse scene graph putting all validation descriptors and relations into set
  PredicateHelper::Set sceneGraphDescriptors;
  std::deque<MemoryObject::ConstPtr> frontier;
  MemoryObject::ConstPtr currNode;
  frontier.push_back(shared_from_this());
  while (!frontier.empty()) {
    // get next node
    currNode = frontier.front();
    frontier.pop_front();

    // add current node's children to frontier
    MemoryObject::Vec currNodeChildren = currNode->getChildren();
    frontier.insert(frontier.end(), currNodeChildren.begin(), currNodeChildren.end());

    // add current node's descriptors to sceneGraphDescriptors
    const PredicateHelper::Set currNodeDescriptors = currNode->getValidationResults().getDescriptors();
    sceneGraphDescriptors.insert(currNodeDescriptors.begin(), currNodeDescriptors.end());

    // add current node's relations to sceneGraphDescriptors
    RelationValidationResult::Vec currNodeRelations = currNode->getRelations();
    RelationValidationResult::Vec::const_iterator relations_itr;
    for (relations_itr = currNodeRelations.begin(); relations_itr != currNodeRelations.end(); ++relations_itr) {
      sceneGraphDescriptors.insert((*relations_itr)->getDescriptor());
    }
  }

  // now check that all descriptors are contained in the sceneGraphDescriptors
  PredicateHelper::Set::const_iterator descriptors_itr;
  for (descriptors_itr = descriptors.begin(); descriptors_itr != descriptors.end(); ++descriptors_itr) {
    if (sceneGraphDescriptors.count(*descriptors_itr) == 0) {
      // TODO: change this to debug!
      if (logger->isDebugEnabled()) {
        PredicateHelper::Set::const_iterator debug_descriptors_itr;
        std::string descriptorsString;
        for (debug_descriptors_itr = sceneGraphDescriptors.begin(); debug_descriptors_itr != sceneGraphDescriptors.end(); ++debug_descriptors_itr) {
          descriptorsString.append(debug_descriptors_itr->toString());
          descriptorsString.append(" ");
        }
        LOG4CXX_DEBUG(logger, boost::format("[getDetectionConfidence] typeId: %lld. scene graph descriptors: %s. \n does not contain descriptor: %s.")
                             % this->typeId
                             % descriptorsString
                             % descriptors_itr->toString());
      }
      return 0.0f;
    }
  }

  // TODO: actually calculate the confidence value of the descriptors
  return 1.0f;
}


MemoryObject::Vec MemoryObject::getMemoryObjects(const std::string &variableName) {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);

  MemoryObject::Vec returnMemoryObjects = getChildren(variableName);
  if (this->getVariableName().compare(variableName) == 0) {
    returnMemoryObjects.push_back(shared_from_this());
  }
  return returnMemoryObjects;
}

const MemoryObject::Vec MemoryObject::getChildren(const std::string &variableName) const {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);

  MemoryObject::Vec returnChildren;
  std::tr1::unordered_map<std::string, MemoryObject::Vec>::const_iterator children_itr;
  for (children_itr = children.begin(); children_itr != children.end(); ++children_itr) {
    if (children_itr->first.compare(variableName) == 0) {
      returnChildren.reserve(children_itr->second.size());
      returnChildren.insert(returnChildren.end(), children_itr->second.begin(), children_itr->second.end());
    }

    // iterate through vector of children, getting their children of appropriate var name
    MemoryObject::Vec::const_iterator mo_itr;
    for (mo_itr = children_itr->second.begin(); mo_itr != children_itr->second.end(); ++mo_itr) {
      MemoryObject::Vec tmpChildren = (*mo_itr)->getChildren(variableName);
      returnChildren.reserve(returnChildren.size() + tmpChildren.size());
      returnChildren.insert(returnChildren.end(), tmpChildren.begin(), tmpChildren.end());
    }
  }
  return returnChildren;
}

const MemoryObject::Vec MemoryObject::getChildren() const {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);

  MemoryObject::Vec returnChildren;
  std::tr1::unordered_map<std::string, MemoryObject::Vec>::const_iterator children_itr;
  for (children_itr = children.begin(); children_itr != children.end(); ++children_itr) {
    returnChildren.insert(returnChildren.end(), children_itr->second.begin(), children_itr->second.end());
  }
  return returnChildren;
}

const RelationValidationResult::Vec MemoryObject::getRelations(const std::string &relationName) const {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);

  std::tr1::unordered_map<std::string, RelationValidationResult::Vec>::const_iterator relations_itr = relations.find(
          relationName);
  if (relations_itr == relations.end()) {
    RelationValidationResult::Vec relationsVec;
    return relationsVec;
  } else {
    return relations_itr->second;
  }
}

const RelationValidationResult::Vec MemoryObject::getRelations() const {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);

  RelationValidationResult::Vec relationsVec;
  std::tr1::unordered_map<std::string, RelationValidationResult::Vec>::const_iterator relations_itr;
  for (relations_itr = relations.begin(); relations_itr != relations.end(); ++relations_itr) {
    relationsVec.insert(relationsVec.end(), relations_itr->second.begin(), relations_itr->second.end());
  }
  return relationsVec;
}

ValidationResults MemoryObject::getValidationResults() const {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  return validationResults;
}

ValidationResult::Vec MemoryObject::getValidationResults(const PredicateHelper &descriptor) const {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  return validationResults.getResults(descriptor);
}

std::string MemoryObject::getValidationResultsString() const {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  return validationResults.getResultsString();
}

CaptureData::ConstPtr MemoryObject::getCaptureData() const {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  return captureData;
}

//use when calling multiple GETS or SETS in succession to avoid clients getting "mismatched" data

void MemoryObject::lock() const {
  data_mutex.lock();
};

void MemoryObject::unlock() const {
  data_mutex.unlock();
};

//use when calling multiple GETS in succession to avoid getting "mismatched" data
//void MemoryObject::lock_shared() const {
//    printf("MO: lock_shared\n");
//    data_mutex.lock_shared();
//    printf("MO: lock_shared done\n");
//};
//
//void MemoryObject::unlock_shared() const {
//    printf("MO: unlock_shared\n");
//    data_mutex.unlock_shared();
//    printf("MO: unlock_shared done\n");
//};

///////////////// POINT CLOUD OBJECT ///////////////////////////////////////////

PointCloudObject::~PointCloudObject() {
}

void PointCloudObject::update(const MemoryObject::Ptr &match) {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);

  //base class update, which updates confidence, among other things
  this->MemoryObject::update(match);

  //cast from base to derived type
  PointCloudObject::Ptr matchPCO = boost::dynamic_pointer_cast<PointCloudObject>(match);
  if (!matchPCO) { //bad cast
    LOG4CXX_ERROR(logger, "[compare] Bad cast to PointCloudObject::Ptr.");
    return;
  }

  //update point cloud specific content
  wireframe_polygons = matchPCO->getWireframePolygons();
  object_coeffs = matchPCO->getWireframeCoefficients();

}

double PointCloudObject::compare(const MemoryObject::Ptr &other) const {
  //cast from base to derived type
  PointCloudObject::Ptr otherPointCloudObj = boost::dynamic_pointer_cast<PointCloudObject>(other);
  if (!otherPointCloudObj) { //bad cast
    LOG4CXX_ERROR(logger, "[compare] Bad cast to PointCloudObject::Ptr.");
    return 0.0;
  }

  //don't really need to lock for this compare, but just here in case more
  //comparisons are added later
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);

  //TODO: something about this?
  //  if (typeId != otherPointCloudObj->getTypeId()) {
  //    LOG4CXX_ERROR(logger, "[compare] has different typeId.");
  //    return 0.0;
  //  }

  //compare two bounding boxes
  static const float conf_thresh = 0.7;
  float overlap = ade::stm::util::calculateBoundBoxOverlap(mask->getBoundingBox(),
                                                           other->getDetectionMask()->getBoundingBox());
  float sizeSim = ade::stm::util::calculateBoundBoxSizeSimilarity(mask->getBoundingBox(),
                                                                  other->getDetectionMask()->getBoundingBox());
  float confidence = overlap * sizeSim;
  LOG4CXX_TRACE(logger, boost::format("[compare] thisId: %lld. otherId: %lld. overlap: %f. sizeSim: %f. conf: %f.")
                        % id % other->getId() % overlap % sizeSim % confidence);

  return (confidence > conf_thresh) ? 1.0 : 0.0;
}

void PointCloudObject::setWireframe(std::vector<pcl::Vertices> &polygons) {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  wireframe_polygons = boost::make_shared<std::vector<pcl::Vertices> >(polygons);
}

void PointCloudObject::setWireframe(std::vector<pcl::Vertices> &polygons,
                                    pcl::ModelCoefficients::Ptr coeffs) {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  wireframe_polygons = boost::make_shared<std::vector<pcl::Vertices> >(polygons);
  object_coeffs = coeffs;
}

boost::shared_ptr<std::vector<pcl::Vertices> const> PointCloudObject::getWireframePolygons() const {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  return wireframe_polygons;
}

pcl::ModelCoefficients::ConstPtr PointCloudObject::getWireframeCoefficients() const {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  return object_coeffs;
}

///////////////////////////// SurfaceObject ////////////////////////////////
#ifdef USE_V4R_V0

double SurfaceObject::compare(const MemoryObject::Ptr & other) const {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  return MemoryObject::compare(other);
}

void SurfaceObject::setSurfaceModels(std::vector<surface::SurfaceModel::Ptr>& surfModels) {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  surfaceModels = surfModels;

  surfaceImages.resize(surfaceModels.size());
  for (size_t i = 0; i < surfaceModels.size(); i++) {
    //surfaceImages[i] = getProjectedSurfaceImage(surfaceModels[i]);
    surfaceImages[i] = getOrthogonallyProjectedSurfaceImage(surfaceModels[i]);
  }
}

cv::Mat SurfaceObject::getProjectedSurfaceImage(surface::SurfaceModel::Ptr surf) {
  cv::Rect rect = calculateSurfaceBoundingRectangle(surf);
  cv::Mat img(rect.height, rect.width, CV_8UC3, cv::Scalar(0));
  for (size_t i = 0; i < surf->indices.size(); ++i) {
    int index = surf->indices[i];
    int x = index % (int) captureData->cloudRGB->width - rect.x;
    int y = index / (int) captureData->cloudRGB->width - rect.y;
    img.at<cv::Vec3b>(y, x)[2] = captureData->cloudRGB->points[index].r;
    img.at<cv::Vec3b>(y, x)[1] = captureData->cloudRGB->points[index].g;
    img.at<cv::Vec3b>(y, x)[0] = captureData->cloudRGB->points[index].b;
  }
  return img;
}

cv::Mat SurfaceObject::getOrthogonallyProjectedSurfaceImage(surface::SurfaceModel::Ptr surf) {
  //get point in surf
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr surfaceCloud(new pcl::PointCloud<pcl::PointXYZRGB > ());
  pcl::IndicesPtr indices(new std::vector<int>);
  (*indices) = surf->indices;
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(captureData->cloudRGB);
  extract.setIndices(indices);
  extract.setKeepOrganized(true);
  extract.setNegative(false);
  extract.filter(*surfaceCloud);

  //get 3d transform to "align" surface with image plane
  //get surface "directions"
  pcl::PCA<pcl::PointXYZRGB> pca;
  pca.setInputCloud(captureData->cloudRGB);
  pca.setIndices(indices);
  Eigen::Matrix3f eigenVecs = pca.getEigenVectors();
  Eigen::Vector3f y_dir = eigenVecs.col(1);
  Eigen::Vector3f z_dir = eigenVecs.col(2);

  //find 3x3 transformation matrix that rotates surface to be orthogonal to image plane
  Eigen::Affine3f transform = pcl::getTransformationFromTwoUnitVectors(y_dir, z_dir);

  //transform 3d points
  //first, demean original points (pcl::demeanPointCloud doesn't preserve color)
  Eigen::Matrix3f covar;
  Eigen::Vector4f centroid;
  pcl::computeMeanAndCovarianceMatrix(*surfaceCloud, covar, centroid);
  for (size_t i = 0; i < surf->indices.size(); ++i) {
    int index = surf->indices[i];
    surfaceCloud->points[index].x = static_cast<float> (surfaceCloud->points[index].x - centroid(0));
    surfaceCloud->points[index].y = static_cast<float> (surfaceCloud->points[index].y - centroid(1));
    surfaceCloud->points[index].z = static_cast<float> (surfaceCloud->points[index].z - centroid(2));
  }

  //then, perform rotation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGB > ());
  pcl::transformPointCloud(*surfaceCloud, *transformedCloud, transform);

  //finally, translate back
  for (size_t i = 0; i < surf->indices.size(); ++i) {
    int index = surf->indices[i];
    surfaceCloud->points[index].x = static_cast<float> (surfaceCloud->points[index].x + centroid(0));
    surfaceCloud->points[index].y = static_cast<float> (surfaceCloud->points[index].y + centroid(1));
    surfaceCloud->points[index].z = static_cast<float> (surfaceCloud->points[index].z + centroid(2));
    transformedCloud->points[index].x = static_cast<float> (transformedCloud->points[index].x + centroid(0));
    transformedCloud->points[index].y = static_cast<float> (transformedCloud->points[index].y + centroid(1));
    transformedCloud->points[index].z = static_cast<float> (transformedCloud->points[index].z + centroid(2));
  }

  // project points to 2d
  cv::Point3f tempPoint;
  std::vector<cv::Point3f> surfacePoints;
  std::vector<cv::Point3f> transformedPoints;
  surfacePoints.reserve(surf->indices.size());
  transformedPoints.reserve(surf->indices.size());
  int index = -1;
  for (int i = 0; i < surf->indices.size(); ++i) {
    index = surf->indices[i];
    tempPoint.x = surfaceCloud->points.at(index).x;
    tempPoint.y = surfaceCloud->points.at(index).y;
    tempPoint.z = surfaceCloud->points.at(index).z;
    if (!std::isnan(tempPoint.x) && !std::isnan(tempPoint.y) && !std::isnan(tempPoint.z)) {
      surfacePoints.push_back(tempPoint);
    }
    tempPoint.x = transformedCloud->points.at(index).x;
    tempPoint.y = transformedCloud->points.at(index).y;
    tempPoint.z = transformedCloud->points.at(index).z;
    if (!std::isnan(tempPoint.x) && !std::isnan(tempPoint.y) && !std::isnan(tempPoint.z)) {
      transformedPoints.push_back(tempPoint);
    }
  }
  std::vector<cv::Point2f> surfImgPoints;
  std::vector<cv::Point2f> transformedImgPoints;
  ade::capture::util::projectPoints(surfacePoints, surfImgPoints, 0);
  ade::capture::util::projectPoints(transformedPoints, transformedImgPoints, 0);

  //find homography between projected points
  cv::Mat homography = cv::findHomography(surfImgPoints, transformedImgPoints, CV_RANSAC, 5);

  //build projected image from projected points
  cv::Mat projectedImg = cv::Mat::zeros(captureData->cloud->height, captureData->cloud->width, CV_8UC3);
  cv::Vec3b pixelColor;
  int cloudIndex;
  for (int i = 0; i < surfImgPoints.size(); ++i) {
    cloudIndex = surf->indices[i];
    pixelColor = cv::Vec3b(captureData->cloudRGB->points[cloudIndex].b,
            captureData->cloudRGB->points[cloudIndex].g,
            captureData->cloudRGB->points[cloudIndex].r);
    projectedImg.at<cv::Vec3b>(static_cast<int> (surfImgPoints[i].y), static_cast<int> (surfImgPoints[i].x)) = pixelColor;
  }

  //apply perspective transform
  cv::Mat transformedImg = cv::Mat::zeros(captureData->cloud->height, captureData->cloud->width, projectedImg.type());
  cv::warpPerspective(projectedImg, transformedImg, homography, transformedImg.size());

  //  if (logger->isDebugEnabled()) {
  //    ade::Display::createWindowIfDoesNotExist("original projection");
  //    ade::Display::displayFrame(projectedImg, "original projection");
  //    ade::Display::createWindowIfDoesNotExist("transformed projection");
  //    ade::Display::displayFrame(transformedImg, "transformed projection");
  //    sleep(20);
  //  }

  return transformedImg;
}

cv::Rect SurfaceObject::calculateSurfaceBoundingRectangle(surface::SurfaceModel::Ptr surf) {
  int xmin = (int) captureData->cloud->width;
  int ymin = (int) captureData->cloud->height;
  int xmax = -1;
  int ymax = -1;
  for (size_t i = 0; i < surf->indices.size(); ++i) {
    int index = surf->indices[i];
    int x = index % (int) captureData->cloud->width;
    int y = index / (int) captureData->cloud->width;
    xmin = std::min(xmin, x);
    ymin = std::min(ymin, y);
    xmax = std::max(xmax, x);
    ymax = std::max(ymax, y);
  }
  cv::Rect r;
  r.x = xmin;
  r.y = ymin;
  r.width = xmax - xmin + 1;
  r.height = ymax - ymin + 1;
  return r;
}

int SurfaceObject::getNumSurfaces() const {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  return surfaceModels.size();
}

std::vector<surface::SurfaceModel::Ptr> SurfaceObject::getSurfaceModels() {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  return surfaceModels;
}

const std::vector<cv::Mat>& SurfaceObject::getSurfaceImages() const {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex);
  return surfaceImages;
}

#endif //USE_V4R_V0
