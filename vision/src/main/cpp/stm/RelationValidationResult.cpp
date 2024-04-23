/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "RelationValidationResult.hpp"
#include "MemoryObject.hpp"
#include <boost/thread/locks.hpp>

using namespace ade::stm;

RelationValidationResult::RelationValidationResult(const float &confidence, const PredicateHelper &descriptor,
                                                   MemoryObject::Ptr &relatedObject)
        : descriptor_(descriptor),
          confidence_(confidence),
          relatedObject_(relatedObject),
          data_mutex_() {

}

RelationValidationResult::~RelationValidationResult() {

}

void RelationValidationResult::setConfidence(const float &confidence) {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex_);
  confidence_.setLevel(confidence);
}

void RelationValidationResult::decayConfidence() {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex_);
  confidence_.decay();
}

float RelationValidationResult::getConfidence() const {
  boost::lock_guard<boost::recursive_mutex> lock(data_mutex_);
  return confidence_.getLevel();
}

const PredicateHelper &RelationValidationResult::getDescriptor() const {
  return descriptor_;
}

MemoryObject::Ptr RelationValidationResult::getRelatedObject() {
  return relatedObject_.lock();
}

MemoryObject::ConstPtr RelationValidationResult::getRelatedObject() const {
  return relatedObject_.lock();
}