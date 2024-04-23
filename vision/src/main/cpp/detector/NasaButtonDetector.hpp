/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   NasaButtonDetector.hpp
 * Author: Evan Krause <evan.krause@tufts.edu>
 *
 * Created on December 7, 2016, 4:25 PM
 */

#ifndef NASABUTTONDETECTOR_HPP
#define NASABUTTONDETECTOR_HPP

#include "ObjectDetector.hpp"

#ifdef USE_ROS_VISION
#include <ros/ros.h>
#include <tf/transform_listener.h>
//#include <pcl_ros/point_cloud.h>
#endif //USE_ROS_VISION

class NasaButtonDetector : public ObjectDetector {
public:
  typedef boost::shared_ptr<NasaButtonDetector> Ptr;
  typedef boost::shared_ptr<const NasaButtonDetector> ConstPtr;

  NasaButtonDetector(const long long& processorId, const int imgWidth, const int imgHeight);
  ~NasaButtonDetector();

protected:
  virtual void handleCaptureNotification(CaptureNotification::ConstPtr notification);

private:

//  void buildAveragePointCloud(const cv::Mat& frame, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudRGB);
//  cv::BackgroundSubtractorMOG2 bg_model;
//  cv::Mat xyzFrame;
//  cv::Mat xyzFGMask;
//  cv::Mat xyzBGMask;
//  bool avgPointCloudInit;
//  pcl::PointCloud<pcl::PointXYZ>::Ptr avgPointCloud;

  cv::Mat fgMask;
  bool buttonDetectionReported;

#ifdef USE_ROS_VISION
  ros::NodeHandle* n;
  ros::Publisher pub;
  tf::TransformListener listener;
#endif //USE_ROS_VISION
};

#endif /* NASABUTTONDETECTOR_HPP */

