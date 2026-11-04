/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   ImageProcessorNotification.hpp
 * Author: Evan Krause
 *
 * Created on August 27, 2015, 4:27 PM
 */

#ifndef IMAGEPROCESSORNOTIFICATION_HPP
#define	IMAGEPROCESSORNOTIFICATION_HPP

#include "common/CaptureData.hpp"
#include "common/notification/Notification.hpp"

class ImageProcessorNotification : public Notification {
public:
  typedef boost::shared_ptr<ImageProcessorNotification> Ptr;
  typedef boost::shared_ptr<const ImageProcessorNotification> ConstPtr;

  ImageProcessorNotification(const Type& type_,
                             const long long& processorId_,
                             const unsigned long long& frameNumber_,
                             const CaptureData::ConstPtr& captureData_)
          : Notification(type_, processorId_, -1L, frameNumber_),
            captureData(captureData_) {
  }

  //! all capture data from a single time-step
  CaptureData::ConstPtr captureData;
};

#endif	/* IMAGEPROCESSORNOTIFICATION_HPP */

