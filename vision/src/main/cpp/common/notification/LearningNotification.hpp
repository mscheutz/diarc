/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   LearningNotification.hpp
 * Author: Evan Krause <evan.krause@tufts.edu>
 *
 * Created on November 28, 2016, 5:27 PM
 */

#ifndef LEARNINGNOTIFICATION_HPP
#define	LEARNINGNOTIFICATION_HPP

#include "common/notification/VisionNotification.hpp"
#include "stm/MemoryObject.hpp"

/*
 * Notifies about a new object learning event.
 */
class LearningNotification : public VisionNotification {
public:
  typedef boost::shared_ptr<LearningNotification> Ptr;
  typedef boost::shared_ptr<const LearningNotification> ConstPtr;

  LearningNotification(const VisionProcessConstPtr& notifier_,
                       const long long& typeId_,
                       const unsigned long long& frameNumber_,
                       diarc::stm::MemoryObject::Ptr& object_,
                       PredicateHelper::Set descriptors_)
  : VisionNotification(notifier_, LEARNING, typeId_, frameNumber_),
  object(object_),
  descriptors(descriptors_) {
  }

  //! detected object - intentionally non-const - can be modified concurrently
  diarc::stm::MemoryObject::Ptr object;
  
  //! description of new thing being learned
  const PredicateHelper::Set descriptors;
};


#endif	/* LEARNINGNOTIFICATION_HPP */

