/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   MemoryObjectNotification.hpp
 * Author: evan
 *
 * Created on August 27, 2015, 5:00 PM
 */

#ifndef MEMORYOBJECTNOTIFICATION_HPP
#define	MEMORYOBJECTNOTIFICATION_HPP

#include "common/notification/Notification.hpp"
#include "stm/MemoryObject.hpp"

/*
 * Notifies about a new detected object.
 */
class MemoryObjectNotification : public Notification {
public:
  typedef boost::shared_ptr<MemoryObjectNotification> Ptr;
  typedef boost::shared_ptr<const MemoryObjectNotification> ConstPtr;

  MemoryObjectNotification(const long long& processorId_,
                           const long long& typeId_,
                           const unsigned long long& frameNumber_,
                           diarc::stm::MemoryObject::Ptr& object_)
  : Notification(MEMORY_OBJECT, processorId_, typeId_, frameNumber_),
  object(object_) {
  }

  //! detected object - intentionally non-const - can be modified concurrently
  diarc::stm::MemoryObject::Ptr object;
};

#endif	/* MEMORYOBJECTNOTIFICATION_HPP */

