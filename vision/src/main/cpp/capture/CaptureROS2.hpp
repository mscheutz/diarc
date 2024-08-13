/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef CAPTUREROS2_HPP
#define CAPTUREROS2_HPP

#include "Capture.hpp"

#include <boost/thread/mutex.hpp>
#include <pcl/point_cloud.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/image.h>
#include <sensor_msgs/msg/point_cloud.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>

namespace ade {
  namespace capture {

    class CaptureROS2 : public Capture {
    public:
      CaptureROS2(const std::string &configFile);

      ~CaptureROS2();

      virtual void interruptWait();

      bool captureCurrentCameraFrame();

      //======== DO NOT CALL - only for ROS callbacks =================
      void captureCallback_MONO(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

      void
      captureCallback_STEREO(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg, const sensor_msgs::msg::Image::ConstSharedPtr &image2_msg);

      void captureCallback_RGB_AND_DEPTH(const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
                                         const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg);

      void captureCallback_POINT_CLOUD(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
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

      void spin();
      std::thread spin_thread_;
      //data
      mutable boost::mutex data_mutex_;
      boost::condition_variable condition_;
      bool data_received_;
      bool interrupt_flag_;
      bool convertToMeters_;
      bool convertToRGB_;
      bool performDepthRegistration_;
      std::string rosNamespace_;

      sensor_msgs::msg::Image::ConstSharedPtr image_msg_;
      sensor_msgs::msg::Image::ConstSharedPtr image2_msg_;
      sensor_msgs::msg::Image::ConstSharedPtr depth_msg_;
      sensor_msgs::msg::PointCloud2::ConstSharedPtr point_cloud_msg_;

      rclcpp::Node::SharedPtr n_;
      image_transport::ImageTransport *it_;
      image_transport::Subscriber single_image_sub_;  // only for single image case
      image_transport::SubscriberFilter image_sub_;
      image_transport::SubscriberFilter image2_sub_;
      image_transport::SubscriberFilter depth_sub_;

      std::shared_ptr<message_filters::TimeSynchronizer <sensor_msgs::msg::Image, sensor_msgs::msg::Image>> sync_;

      pcl::PointCloud<pcl::PointXYZ>::ConstPtr const getPointCloud(const unsigned long int frameNum) const;
    };


  } //namespace capture
} //namespace ade  


#endif  //CAPTUREROS2_HPP
