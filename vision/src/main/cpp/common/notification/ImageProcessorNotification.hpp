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
#include "common/notification/VisionNotification.hpp"

class ImageProcessorNotification : public VisionNotification {
public:
  typedef boost::shared_ptr<ImageProcessorNotification> Ptr;
  typedef boost::shared_ptr<const ImageProcessorNotification> ConstPtr;

  ImageProcessorNotification(const VisionProcessConstPtr& notifier_,
                             const Type& type_,
                             const unsigned long long& frameNumber_,
                             const CaptureData::ConstPtr& captureData_)
          : VisionNotification(notifier_, type_, -1L, frameNumber_),
            captureData(captureData_) {
  }

  //! all capture data from a single time-step
  CaptureData::ConstPtr captureData;
};

#endif	/* IMAGEPROCESSORNOTIFICATION_HPP */

