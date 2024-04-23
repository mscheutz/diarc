/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   CaptureNotification.hpp
 * Author: Evan Krause
 *
 * Created on August 24, 2015, 5:54 PM
 */

#ifndef CAPTURENOTIFICATION_HPP
#define	CAPTURENOTIFICATION_HPP

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include "common/notification/Notification.hpp"
#include "common/CaptureData.hpp"

class CaptureNotification : public Notification {
public:
  typedef boost::shared_ptr<CaptureNotification> Ptr;
  typedef boost::shared_ptr<const CaptureNotification> ConstPtr;

  CaptureNotification(unsigned long long& frameNumber_, const CaptureData::ConstPtr& captureData_)
          : Notification(CAPTURE, -1L, frameNumber_),
            captureData(captureData_) {
  }

  CaptureNotification(unsigned long long& frameNumber,
          const cv::Mat& transform_,
          const cv::Mat& frame_,
          const cv::Mat& frame2_,
          const cv::Mat& depthFrame_,
          const cv::Mat& depthFrame2_,
          const cv::Mat& depthFrameIntensity_,
          pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_,
          pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudRGB_)
  : Notification(CAPTURE, -1L, frameNumber),
  captureData(new CaptureData(frameNumber,
  transform_,
  frame_,
  frame2_,
  depthFrame_,
  depthFrame2_,
  depthFrameIntensity_,
  cloud_,
  cloudRGB_)) {
  }

  //! all capture data from a single time-step
  CaptureData::ConstPtr captureData;
};

#endif	/* CAPTURENOTIFICATION_HPP */

