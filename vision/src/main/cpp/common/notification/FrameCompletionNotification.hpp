/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   FrameCompletionNotification.hpp
 * Author: Evan Krause
 *
 * Created on August 27, 2015, 4:04 PM
 */

#ifndef FRAMECOMPLETIONNOTIFICATION_HPP
#define	FRAMECOMPLETIONNOTIFICATION_HPP

#include "common/notification/VisionNotification.hpp"

class FrameCompletionNotification : public VisionNotification {
public:
  typedef boost::shared_ptr<FrameCompletionNotification> Ptr;
  typedef boost::shared_ptr<const FrameCompletionNotification> ConstPtr;

  FrameCompletionNotification(const VisionProcessConstPtr& notifier_,
                              const long long& typeId_,
                              const unsigned long long& frameNumber_)
  : VisionNotification(notifier_, FRAME_COMPLETION, typeId_, frameNumber_) {
  }
};


#endif	/* FRAMECOMPLETIONNOTIFICATION_HPP */

