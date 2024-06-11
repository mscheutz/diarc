/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef CAPTUREROS_HPP
#define CAPTUREROS_HPP

#include "Capture.hpp"

#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>

namespace diarc {
  namespace capture {

    class CaptureROS : public Capture {
    public:
      CaptureROS(const std::string &configFile);

      ~CaptureROS();

      virtual void interruptWait();

      bool captureCurrentCameraFrame();

      //======== DO NOT CALL - only for ROS callbacks =================
      void captureCallback_MONO(const sensor_msgs::Image::ConstPtr &msg);

      void
      captureCallback_STEREO(const sensor_msgs::ImageConstPtr &image_msg, const sensor_msgs::ImageConstPtr &image2_msg);

      void captureCallback_RGB_AND_DEPTH(const sensor_msgs::ImageConstPtr &rgb_msg,
                                         const sensor_msgs::ImageConstPtr &depth_msg);

      void captureCallback_POINT_CLOUD(const sensor_msgs::PointCloud2::ConstPtr &msg);
      //===============================================================

    private:
      enum MODE {
        MONO,
        STEREO,
        RGB_AND_DEPTH,
        POINT_CLOUD
      };

      MODE mode_;

      //capture helpers
      bool capture_MONO();

      bool capture_STEREO();

      bool capture_RGB_AND_DEPTH();

      //data
      mutable boost::mutex data_mutex_;
      boost::condition_variable condition_;
      bool data_received_;
      bool interrupt_flag_;
      bool convertToMeters_;
      bool convertToRGB_;
      bool performDepthRegistration_;
      std::string rosNamespace_;

      sensor_msgs::Image::ConstPtr image_msg_;
      sensor_msgs::Image::ConstPtr image2_msg_;
      sensor_msgs::Image::ConstPtr depth_msg_;
      sensor_msgs::PointCloud2::ConstPtr point_cloud_msg_;

      ros::NodeHandle *n_;
      image_transport::ImageTransport *it_;
      image_transport::Subscriber single_image_sub_;  // only for single image case
      image_transport::SubscriberFilter *image_sub_;
      image_transport::SubscriberFilter *image2_sub_;
      image_transport::SubscriberFilter *depth_sub_;

      typedef message_filters::sync_policies::ApproximateTime <sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
      message_filters::Synchronizer <MySyncPolicy> *sync_;

      ros::AsyncSpinner *spinner_;
      pcl::PointCloud<pcl::PointXYZ>::ConstPtr const getPointCloud(const unsigned long int frameNum) const;
    };


  } //namespace capture
} //namespace diarc


#endif  //CAPTUREROS_HPP
