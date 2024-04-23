/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "CaptureMultisense.hpp"
#include "point_clouds/PointCloudUtilities.hpp"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/image_encodings.h>
#include <utility>      //std::pair

namespace ade {
  namespace capture {

    CaptureMultisense::CaptureMultisense(const std::string &configFile)
            : Capture(configFile),
              data_mutex(),
              condition(),
              data_received(false),
              interrupt_flag(false) {

      using boost::property_tree::ptree;
      ptree pt;
      read_xml(configFile, pt);
      std::string left_image_topic = pt.get<std::string>("capture.leftImageTopic");
      std::string right_image_topic = pt.get<std::string>("capture.rightImageTopic", "");
      std::string disparity_topic = pt.get<std::string>("capture.disparityTopic", "");
      std::string point_cloud_topic = pt.get<std::string>("capture.pointCloudTopic", "");

      // initialize ROS
      LOG4CXX_INFO(logger, "Initializing ROS.");
      std::vector<std::pair<std::string, std::string> > remapping;
      ros::init(remapping, "ade_vision_cap");

      // create node handle
      LOG4CXX_INFO(logger, "Initializing ROS node.");
      n = new ros::NodeHandle();

      // sensor subs
      left_rgb_sub = new message_filters::Subscriber<sensor_msgs::Image>(*n, left_image_topic, 1);
      right_rgb_sub = new message_filters::Subscriber<sensor_msgs::Image>(*n, right_image_topic, 1);
      disparity_sub = new message_filters::Subscriber<stereo_msgs::DisparityImage>(*n, disparity_topic, 1);
      point_cloud_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(*n, point_cloud_topic, 1);

      // synchronize image subs
      sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *left_rgb_sub, *right_rgb_sub,
                                                             *disparity_sub, *point_cloud_sub);
      sync->registerCallback(boost::bind(&CaptureMultisense::captureCallback, this, _1, _2, _3, _4));

      LOG4CXX_INFO(logger, "starting async spinner.");
      spinner = new ros::AsyncSpinner(1);
      spinner->start();
      LOG4CXX_INFO(logger, "async spinner started.");

      ///////////////HACK FOR DEMO - THESE ARE KINECT PARAMS///////////////////////////////////
      //set depth image parameters
      CameraParameters::Ptr camParams(new CameraParameters(VisionConstants::imgWidth, VisionConstants::imgHeight));

      double depth_fl = 287.907867 * 2.0;
      float fx = depth_fl * (imgWidth / 640.0);
      camParams->M.at<float>(0, 0) = fx;
      float fy = depth_fl * (imgHeight / 480.0);
      camParams->M.at<float>(1, 1) = fy;
      float cx = ((float) imgWidth - 1.f) / 2.f;
      camParams->M.at<float>(0, 2) = cx;
      float cy = ((float) imgHeight - 1.f) / 2.f;
      camParams->M.at<float>(1, 2) = cy;
      camParams->intrinsicSet = true;
      cameras->setCameraParameters(0, camParams);

      //set rgb frame parameters
      camParams = CameraParameters::Ptr(new CameraParameters(VisionConstants::imgWidth, VisionConstants::imgHeight));
      double image_fl = 262.5 * 2.0;
      float fx2 = image_fl * (imgWidth / 640.0);
      camParams->M.at<float>(0, 0) = fx2;
      float fy2 = image_fl * (imgHeight / 480.0);
      camParams->M.at<float>(1, 1) = fy2;
      float cx2 = ((float) imgWidth - 1.f) / 2.f;
      camParams->M.at<float>(0, 2) = cx2;
      float cy2 = ((float) imgHeight - 1.f) / 2.f;
      camParams->M.at<float>(1, 2) = cy;
      camParams->intrinsicSet = true;
      cameras->setCameraParameters(1, camParams);
      ////////////////////////////////////////////////////////////

      //init depth frame
      depthFrame.create(imgHeight, imgWidth, CV_32FC1);
      depthFrameIntensity.create(imgHeight, imgWidth, CV_32FC1);

      LOG4CXX_INFO(logger, "Finished constructing.");
    }

    CaptureMultisense::~CaptureMultisense() {
      LOG4CXX_INFO(logger, "desctructor.");
      ros::shutdown();
      delete left_rgb_sub;
      delete right_rgb_sub;
      delete sync;
      spinner->stop();
      delete spinner;
      delete n;
    }

    void CaptureMultisense::interruptWait() {
      LOG4CXX_INFO(logger, "interruptWait.");
      boost::mutex::scoped_lock lock(data_mutex);
      interrupt_flag = true;
      condition.notify_all();
    }

    bool CaptureMultisense::captureCurrentCameraFrame() {
      LOG4CXX_DEBUG(logger, "captureCurrentCameraFrame.");
      boost::mutex::scoped_lock lock(data_mutex);
      boost::system_time const timeout = boost::get_system_time() + boost::posix_time::milliseconds(3500);
      while (!data_received && !interrupt_flag) {
        LOG4CXX_DEBUG(logger, "capture waiting ..........");
        condition.timed_wait(lock, timeout); //wait(lock);
      }
      LOG4CXX_DEBUG(logger, ".........done waiting.");
      if (data_received) {
        // time stamp
        frameCount = left_rgb->header.stamp.toNSec();

        // left image
        cv_bridge::CvImageConstPtr cv_left_rgb_ptr;
        cv_left_rgb_ptr = cv_bridge::toCvShare(left_rgb, sensor_msgs::image_encodings::TYPE_8UC3);
        cv::resize(cv_left_rgb_ptr->image, frame, frame.size(), 0, 0, CV_INTER_NN);
        cv::cvtColor(frame, frame, CV_RGB2BGR);

        // right image
        cv_bridge::CvImageConstPtr cv_right_rgb_ptr;
        cv_right_rgb_ptr = cv_bridge::toCvShare(right_rgb, sensor_msgs::image_encodings::TYPE_8UC3);
        cv::resize(cv_right_rgb_ptr->image, frame2, frame2.size(), 0, 0, CV_INTER_NN);
        cv::cvtColor(frame2, frame2, CV_RGB2BGR);

        // disparity image
        cv_bridge::CvImageConstPtr cv_disparity_ptr;
        cv_disparity_ptr = cv_bridge::toCvShare(disparity->image, disparity, disparity->image.encoding);
        cv::resize(cv_disparity_ptr->image, depthFrameIntensity, depthFrameIntensity.size(), 0, 0, CV_INTER_NN);

        // TEST
//		for (int r = 0; r < depthFrameIntensity.rows; ++r) {
//		  for (int c = 0; c < depthFrameIntensity.cols; ++c) {
//			depthFrameIntensity.at<>()
//		  }
//		}
        // TEST

        // point cloud
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*point_cloud, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloudRGB);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

        data_received = false;
        return true;
      } else {
        return false;
      }
    }

    void CaptureMultisense::captureCallback(const sensor_msgs::ImageConstPtr &left_rgb_msg,
                                            const sensor_msgs::ImageConstPtr &right_rgb_msg,
                                            const stereo_msgs::DisparityImageConstPtr &disparity_msg,
                                            const sensor_msgs::PointCloud2ConstPtr &point_cloud_msg) {
      boost::mutex::scoped_lock lock(data_mutex);
      LOG4CXX_DEBUG(logger, "captureCallback");
      left_rgb = left_rgb_msg;
      right_rgb = right_rgb_msg;
      disparity = disparity_msg;
      point_cloud = point_cloud_msg;
      data_received = true;
      condition.notify_one();
    }


    //int main(int argc, char **argv) {
    //    int width = 640;
    //    int height = 480;
    //    IplImage* rgbImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
    //    cvNamedWindow("rgb", CV_WINDOW_AUTOSIZE);
    //
    //    ros::init(argc, argv, "Vision Capture Listener");
    //
    //    ros::NodeHandle n;
    //
    //    ros::Subscriber sub = n->subscribe("/camera/image_raw", 1000, chatterCallback);
    //
    //    ros::spin();
    //
    //    cvReleaseImage(&rgbImg);
    //    return 0;
    //}


  } //namespace capture
} //namespace ade  
