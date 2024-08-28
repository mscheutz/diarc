/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef CAPTUREMULTISENSE_HPP
#define CAPTUREMULTISENSE_HPP

#include "Capture.hpp"

#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <stereo_msgs/DisparityImage.h>
#include <pcl/point_cloud.h>

namespace diarc {
  namespace capture {

	class CaptureMultisense : public Capture {
	public:
	  CaptureMultisense(const std::string &configFile);
	  ~CaptureMultisense();

	  virtual void interruptWait();

	  bool captureCurrentCameraFrame();

	  //======== DO NOT CALL - only for ROS callbacks =================
	  void captureCallback(const sensor_msgs::ImageConstPtr& left_rgb_msg,
			  const sensor_msgs::ImageConstPtr & right_rgb_msg,
			  const stereo_msgs::DisparityImageConstPtr & disparity_msg,
			  const sensor_msgs::PointCloud2ConstPtr & point_cloud_msg);
	  //===============================================================

	private:

	  // data
	  CAM_MODE pr2_cam_mode;
	  mutable boost::mutex data_mutex;
	  boost::condition_variable condition;
	  bool data_received;
	  bool interrupt_flag;

	  sensor_msgs::Image::ConstPtr left_rgb;
	  sensor_msgs::Image::ConstPtr right_rgb;
	  stereo_msgs::DisparityImage::ConstPtr disparity;
	  sensor_msgs::PointCloud2::ConstPtr point_cloud;
	  ros::NodeHandle* n;

	  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
	  stereo_msgs::DisparityImage, sensor_msgs::PointCloud2> MySyncPolicy;
	  message_filters::Synchronizer< MySyncPolicy >* sync;

	  message_filters::Subscriber<sensor_msgs::Image>* left_rgb_sub;
	  message_filters::Subscriber<sensor_msgs::Image>* right_rgb_sub;
	  message_filters::Subscriber<stereo_msgs::DisparityImage>* disparity_sub;
	  message_filters::Subscriber<sensor_msgs::PointCloud2>* point_cloud_sub;

	  ros::AsyncSpinner* spinner;
	  pcl::PointCloud<pcl::PointXYZ>::ConstPtr const getPointCloud(const unsigned long int frameNum) const;
	};


  } //namespace capture
} //namespace diarc


#endif  //CAPTUREMULTISENSE_HPP
