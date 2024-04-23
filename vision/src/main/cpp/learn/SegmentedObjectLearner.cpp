/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "SegmentedObjectLearner.hpp"

using namespace ade::stm;

SegmentedObjectLearner::SegmentedObjectLearner(const long long& processorId, const int imgWidth, const int imgHeight)
: Learner(processorId, imgWidth, imgHeight) {
  visionProcessName = "SegmentedObjectLearner";
  logger = log4cxx::Logger::getLogger("ade.learn.SegmentedObjectLearner");
}

SegmentedObjectLearner::~SegmentedObjectLearner() {
}

void SegmentedObjectLearner::handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification) {
  MemoryObject::Ptr object = notification->object;

  // get current descriptors to process
  DescriptorsByTypeConstPtr descriptorsByType = getTypes();
  DescriptorsByType::const_iterator descriptorsByType_iter = descriptorsByType->find(object->getTypeId());

  if (descriptorsByType_iter != descriptorsByType->end()) {
    LearningNotification::Ptr n(new LearningNotification(shared_from_this(),
                                                         object->getTypeId(),
                                                         object->getTrackingMask()->getFrameNumber(),
                                                         object,
                                                         descriptorsByType_iter->second));
    sendNotifications(n);
  } else {
    LOG4CXX_ERROR(logger, boost::format("No descriptor found for typeId: %d.") % object->getTypeId());
  }
}
