/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   MotionNotification.hpp
 * Author: Evan Krause
 *
 * Created on Feb 25 2020, 5:18 PM
 */

#ifndef MOTIONNOTIFICATION_HPP
#define	MOTIONNOTIFICATION_HPP

#include "common/notification/ImageProcessorNotification.hpp"

/*
 * Notifies about detected motion.
 */
class MotionNotification : public ImageProcessorNotification {
public:
  typedef boost::shared_ptr<MotionNotification> Ptr;
  typedef boost::shared_ptr<const MotionNotification> ConstPtr;

  MotionNotification(const VisionProcessConstPtr &notifier_,
                     const unsigned long long &frameNumber_,
                     const CaptureData::ConstPtr &captureData_,
                     const cv::Mat &motionFrame_)
      : ImageProcessorNotification(notifier_, MOTION, frameNumber_, captureData_),
        motionFrame(motionFrame_.clone()) {
  }

  const cv::Mat motionFrame;
};

#endif	/* MOTIONNOTIFICATION_HPP */

