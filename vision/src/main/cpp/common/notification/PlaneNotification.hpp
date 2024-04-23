/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   PlaneNotification.hpp
 * Author: Evan Krause
 *
 * Created on August 27, 2015, 4:43 PM
 */

#ifndef PLANENOTIFICATION_HPP
#define	PLANENOTIFICATION_HPP

#include "common/notification/ImageProcessorNotification.hpp"
#include "point_clouds/ExtractedPlane.hpp"
/*
 * Notifies about a new detected plain.
 */
class PlaneNotification : public ImageProcessorNotification {
public:
  typedef boost::shared_ptr<PlaneNotification> Ptr;
  typedef boost::shared_ptr<const PlaneNotification> ConstPtr;

  PlaneNotification(const VisionProcessConstPtr& notifier_,
                    const unsigned long long& frameNumber_,
                    const CaptureData::ConstPtr& captureData_,
                    const ExtractedPlane::ConstPtr& plane_)
          : ImageProcessorNotification(notifier_, PLANE, frameNumber_, captureData_),
            plane(plane_) {
  }

  ExtractedPlane::ConstPtr plane;
};


#endif	/* PLANENOTIFICATION_HPP */

