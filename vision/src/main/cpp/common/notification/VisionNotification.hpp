/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   VisionNotification.hpp
 * Author: evan
 *
 * Created on August 27, 2015, 4:03 PM
 */

#ifndef VISIONNOTIFICATION_HPP
#define	VISIONNOTIFICATION_HPP

#include "common/notification/Notification.hpp"
//#include "visionproc/VisionProcess.hpp"

class VisionProcess;
typedef boost::shared_ptr<VisionProcess> VisionProcessPtr;
typedef boost::shared_ptr<const VisionProcess> VisionProcessConstPtr;

class VisionNotification : public Notification {
public:
  typedef boost::shared_ptr<VisionNotification> Ptr;
  typedef boost::shared_ptr<const VisionNotification> ConstPtr;

  VisionNotification(const VisionProcessConstPtr& notifier_,
                     const Type& type_,
                     const long long& typeId_,
                     const unsigned long long& frameNumber_)
  : Notification(type_, typeId_, frameNumber_), notifier(notifier_) {
  }
  
  VisionProcessConstPtr notifier;

};

#endif	/* VISIONNOTIFICATION_HPP */

