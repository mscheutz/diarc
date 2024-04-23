/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   SiftNotification.hpp
 * Author: Evan Krause
 *
 * Created on August 27, 2015, 4:49 PM
 */

#ifndef SIFTNOTIFICATION_HPP
#define	SIFTNOTIFICATION_HPP

#include "common/notification/ImageProcessorNotification.hpp"

/*
 * Notifies about detected sift features in an image frame.
 */
class SiftNotification : public ImageProcessorNotification {
public:
  typedef boost::shared_ptr<SiftNotification> Ptr;
  typedef boost::shared_ptr<const SiftNotification> ConstPtr;

  SiftNotification(const VisionProcessConstPtr& notifier_,
                   const unsigned long long& frameNumber_,
                   const CaptureData::ConstPtr& captureData_,
                   const SiftFeatures::ConstPtr& sift_)
          : ImageProcessorNotification(notifier_, SIFT, frameNumber_, captureData_),
            siftFeatures(sift_) {
  }

  SiftFeatures::ConstPtr siftFeatures;
};

#endif	/* SIFTNOTIFICATION_HPP */

