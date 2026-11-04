/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   CaptureData.hpp
 * Author: Evan Krause
 *
 * Created on August 27, 2015, 4:15 PM
 */

#ifndef CAPTUREDATA_HPP
#define  CAPTUREDATA_HPP

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <boost/shared_ptr.hpp>

class CaptureData {
public:
  typedef boost::shared_ptr<CaptureData> Ptr;
  typedef boost::shared_ptr<const CaptureData> ConstPtr;

  CaptureData(unsigned long long &frameNumber_,
              const cv::Mat &transform_,
              const cv::Mat &frame_,
              const cv::Mat &frame2_,
              const cv::Mat &depthFrame_,
              const cv::Mat &depthFrame2_,
              const cv::Mat &depthFrameIntensity_,
              pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_,
              pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudRGB_)
          : frameNumber(frameNumber_),
            transform(transform_.clone()),
            frame(frame_.clone()),
            frame2(frame2_.clone()),
            depthFrame(depthFrame_.clone()),
            depthFrame2(depthFrame2_.clone()),
            depthFrameIntensity(depthFrameIntensity_.clone()),
            cloud(cloud_->makeShared()),
            cloudRGB(cloudRGB_->makeShared()) {
  }

  //! frame number of captured data
  const unsigned long long frameNumber;

  //! coordinate frame transform from base to vision
  const cv::Mat transform;

  //! camera 0 data - always copy data if modifying it
  const cv::Mat frame;

  //! camera 1 data - always copy data if modifying it
  const cv::Mat frame2;

  //! stereo pair data - always copy data if modifying it
  const cv::Mat depthFrame, depthFrame2, depthFrameIntensity;

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudRGB;

  /**
   * Convenience method to check if cloud has been populated with cloud data.
   */
  bool hasCloudData() const {
    return (cloud && !cloud->empty());
  }

  /**
   * Convenience method to check if cloudRGB has been populated with cloud data.
   */
  bool hasCloudRGBData() const {
    return (cloudRGB && !cloudRGB->empty());
  }
};

#endif  /* CAPTUREDATA_HPP */

