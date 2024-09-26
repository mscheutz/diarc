/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   MemoryObjectsNotification.hpp
 * Author: evan
 *
 * Created on August 27, 2015, 5:10 PM
 */

#ifndef MEMORYOBJECTSNOTIFICATION_HPP
#define	MEMORYOBJECTSNOTIFICATION_HPP

#include "stm/MemoryObject.hpp"
#include "common/notification/VisionNotification.hpp"

/*
 * Notification about a vector of detected objects.
 */
class MemoryObjectsNotification : public VisionNotification {
public:
  typedef boost::shared_ptr<MemoryObjectsNotification> Ptr;
  typedef boost::shared_ptr<const MemoryObjectsNotification> ConstPtr;

  MemoryObjectsNotification(const VisionProcessConstPtr& notifier_,
                            const long long& typeId_,
                            const unsigned long long& frameNumber_,
                            diarc::stm::MemoryObject::VecPtr objects_)
  : VisionNotification(notifier_, MEMORY_OBJECT, typeId_, frameNumber_),
    objects(objects_) {
  }

  diarc::stm::MemoryObject::VecPtr objects;
};

#endif	/* MEMORYOBJECTSNOTIFICATION_HPP */

