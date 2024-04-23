/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "CaptureROS.hpp"
#include "point_clouds/PointCloudUtilities.hpp"
#include "display/Display.hpp"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/rgbd.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/image_encodings.h>
#include <utility>      //std::pair

namespace ade {
  namespace capture {

    CaptureROS::CaptureROS(const std::string &configFile)
            : Capture(configFile),
              data_mutex_(),
              condition_(),
              data_received_(false),
              interrupt_flag_(false),
              convertToMeters_(false),
              convertToRGB_(false),
              performDepthRegistration_(false),
              rosNamespace_("ade_vision_cap") {

      // parse xml for configuration params
      using boost::property_tree::ptree;
      ptree pt;
      read_xml(configFile, pt);
      std::string image_topic_ = pt.get<std::string>("capture.imageTopic");
      std::string image2_topic_ = pt.get<std::string>("capture.imageTopic2", "");
      std::string depth_topic_ = pt.get<std::string>("capture.depthTopic", "");
      convertToMeters_ = pt.get<bool>("capture.convertToMeters", false);
      convertToRGB_ = pt.get<bool>("capture.convertToRGB", false);
      performDepthRegistration_ = pt.get<bool>("capture.depthRegistration", false);
      rosNamespace_ = pt.get<std::string>("capture.namespace", "ade_vision_cap");

      //initialize ROS
      LOG4CXX_INFO(logger, "Initializing ROS.");
      std::vector<std::pair<std::string, std::string> > remapping;
      ros::init(remapping, rosNamespace_);

      //create node handle
      LOG4CXX_INFO(logger, "Initializing ROS node.");
      n_ = new ros::NodeHandle();

      //create image transport
      LOG4CXX_INFO(logger, "Initializing image transport.");
      it_ = new image_transport::ImageTransport(*n_);

      if (image2_topic_.empty() && depth_topic_.empty()) {
        // single RGB case
        mode_ = MONO;
        single_image_sub_ = it_->subscribe(image_topic_, 10, &CaptureROS::captureCallback_MONO, this);
      } else if (depth_topic_.empty()) {
        // stereo RGB case
        mode_ = STEREO;
        image_sub_ = new image_transport::SubscriberFilter(*it_, image_topic_, 1);
        image2_sub_ = new image_transport::SubscriberFilter(*it_, image2_topic_, 1);
        sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *image_sub_, *image2_sub_);
        sync_->registerCallback(boost::bind(&CaptureROS::captureCallback_RGB_AND_DEPTH, this, _1, _2));
      } else {
        // singe RGB and single depth case
        mode_ = RGB_AND_DEPTH;
        image_sub_ = new image_transport::SubscriberFilter(*it_, image_topic_, 1);
        depth_sub_ = new image_transport::SubscriberFilter(*it_, depth_topic_, 1);
        sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *image_sub_, *depth_sub_);
        sync_->registerCallback(boost::bind(&CaptureROS::captureCallback_RGB_AND_DEPTH, this, _1, _2));
      }

      LOG4CXX_INFO(logger, "starting async spinner.");
      spinner_ = new ros::AsyncSpinner(1);
      spinner_->start();
      LOG4CXX_INFO(logger, "async spinner started.");

      ///////////////HACK FOR DEMO - THESE ARE KINECT PARAMS///////////////////////////////////
      ////set depth image parameters
      //CameraParameters::Ptr camParams(new CameraParameters(VisionConstants::imgWidth, VisionConstants::imgHeight));
      //
      //double depth_fl = 287.907867 * 2.0;
      //float fx = depth_fl * (imgWidth / 640.0);
      //cvmSet(camParams->M, 0, 0, fx);
      //float fy = depth_fl * (imgHeight / 480.0);
      //cvmSet(camParams->M, 1, 1, fy);
      //float cx = ((float) imgWidth - 1.f) / 2.f;
      //cvmSet(camParams->M, 0, 2, cx);
      //float cy = ((float) imgHeight - 1.f) / 2.f;
      //cvmSet(camParams->M, 1, 2, cy);
      //camParams->intrinsicSet = true;
      //cameras->setCameraParameters(0, camParams);
      //
      ////set rgb frame parameters
      //camParams = CameraParameters::Ptr(new CameraParameters(VisionConstants::imgWidth, VisionConstants::imgHeight));
      //double image_fl = 262.5 * 2.0;
      //float fx2 = image_fl * (imgWidth / 640.0);
      //cvmSet(camParams->M, 0, 0, fx2);
      //float fy2 = image_fl * (imgHeight / 480.0);
      //cvmSet(camParams->M, 1, 1, fy2);
      //float cx2 = ((float) imgWidth - 1.f) / 2.f;
      //cvmSet(camParams->M, 0, 2, cx2);
      //float cy2 = ((float) imgHeight - 1.f) / 2.f;
      //cvmSet(camParams->M, 1, 2, cy2);
      //camParams->intrinsicSet = true;
      //cameras->setCameraParameters(1, camParams);
      ////////////////////////////////////////////////////////////

      ///////////////HACK FOR DEMO - THESE ARE REALSENSE PARAMS///////////////////////////////////
      ////set depth image parameters
      //CameraParameters::Ptr camParams(new CameraParameters(VisionConstants::imgWidth, VisionConstants::imgHeight));
      //
      //float fx = 360.01333 * (imgWidth / 480.0);
      //cvmSet(camParams->M, 0, 0, fx);
      //float fy = 360.013366699 * (imgHeight / 270.0);
      //cvmSet(camParams->M, 1, 1, fy);
      //float cx = 243.87228 * (imgWidth / 480.0);
      //cvmSet(camParams->M, 0, 2, cx);
      //float cy = 137.9218444 * (imgHeight / 270.0);
      //cvmSet(camParams->M, 1, 2, cy);
      //camParams->intrinsicSet = true;
      //cameras->setCameraParameters(0, camParams);
      //
      ////set rgb frame parameters
      //camParams = CameraParameters::Ptr(new CameraParameters(VisionConstants::imgWidth, VisionConstants::imgHeight));
      //float fx2 = 1297.672904 * (imgWidth / 1280.0);
      //cvmSet(camParams->M, 0, 0, fx2);
      //float fy2 = 1298.631344 * (imgHeight / 720.0);
      //cvmSet(camParams->M, 1, 1, fy2);
      //float cx2 = 620.914026 * (imgWidth / 1280.0);
      //cvmSet(camParams->M, 0, 2, cx2);
      //float cy2 = 238.280325 * (imgHeight / 720.0);
      //cvmSet(camParams->M, 1, 2, cy2);
      //camParams->intrinsicSet = true;
      //cameras->setCameraParameters(1, camParams);
      //
      //StereoCameraParameters::Ptr stereoParams(new StereoCameraParameters(VisionConstants::imgWidth, VisionConstants::imgHeight));
      //cameras->setStereoParameters(0, stereoParams);
      ////////////////////////////////////////////////////////////

      //init depth frame
      depthFrame.create(imgHeight, imgWidth, CV_32FC1);
      LOG4CXX_INFO(logger, "Finished constructing.");
    }

    CaptureROS::~CaptureROS() {
      LOG4CXX_INFO(logger, "destructor.");

      spinner_->stop();

      switch (mode_) {
        case STEREO:
          delete image_sub_;
          delete image2_sub_;
          delete sync_;
          break;
        case RGB_AND_DEPTH:
          delete image_sub_;
          delete depth_sub_;
          delete sync_;
          break;
      }

      delete it_;
      delete spinner_;
      delete n_;
    }

    void CaptureROS::interruptWait() {
      LOG4CXX_INFO(logger, "interruptWait.");
      boost::mutex::scoped_lock lock(data_mutex_);
      interrupt_flag_ = true;
      condition_.notify_all();
    }

    bool CaptureROS::captureCurrentCameraFrame() {
      switch (mode_) {
        case MONO:
          return capture_MONO();
        case STEREO:
          return capture_STEREO();
        case RGB_AND_DEPTH:
          return capture_RGB_AND_DEPTH();
        default:
          return false;
      }
    }

    bool CaptureROS::capture_MONO() {
      LOG4CXX_DEBUG(logger, "capture_MONO");

      // wait for new data to be captured
      boost::mutex::scoped_lock lock(data_mutex_);
      boost::system_time const timeout = boost::get_system_time() + boost::posix_time::milliseconds(3500);
      while (!data_received_ && !interrupt_flag_) {
        condition_.timed_wait(lock, timeout);
      }

      // copy data to local data structures
      if (data_received_) {
        LOG4CXX_DEBUG(logger, "[capture_MONO] data received");
        try {
          //cv_bridge::CvImageConstPtr cv_rgb_ptr = cv_bridge::toCvShare(image_msg_, sensor_msgs::image_encodings::TYPE_8UC3);
          cv_bridge::CvImagePtr cv_rgb_ptr = cv_bridge::toCvCopy(image_msg_, sensor_msgs::image_encodings::TYPE_8UC3);
          cv::resize(cv_rgb_ptr->image, frame, frame.size(), 0, 0, CV_INTER_NN);
          //          cv::cvtColor(colorImg, frame, CV_BGR2RGB);
        } catch (cv_bridge::Exception &e) {
          LOG4CXX_ERROR(logger, boost::format("cv_bridge exception: %s") % e.what());
          data_received_ = false;
          return false;
        }

        data_received_ = false;
        return true;
      } else {
        return false;
      }
    }

    bool CaptureROS::capture_STEREO() {
      LOG4CXX_DEBUG(logger, "capture_STEREO");

      // wait for new data to be captured
      boost::mutex::scoped_lock lock(data_mutex_);
      boost::system_time const timeout = boost::get_system_time() + boost::posix_time::milliseconds(3500);
      while (!data_received_ && !interrupt_flag_) {
        condition_.timed_wait(lock, timeout);
      }

      // copy data to local data structures
      if (data_received_) {
        memcpy(frame.data, &image_msg_->data[0], image_msg_->data.size());
        memcpy(frame2.data, &image2_msg_->data[0], image2_msg_->data.size());

        data_received_ = false;
        return true;
      } else {
        return false;
      }

    }

    bool CaptureROS::capture_RGB_AND_DEPTH() {
      LOG4CXX_DEBUG(logger, "capture_RGB_AND_DEPTH");

      // wait for new data to be captured
      boost::mutex::scoped_lock lock(data_mutex_);
      boost::system_time const timeout = boost::get_system_time() + boost::posix_time::milliseconds(3500);
      while (!data_received_ && !interrupt_flag_) {
        condition_.timed_wait(lock, timeout);
      }

      if (data_received_) {
        //
        cv_bridge::CvImageConstPtr cv_rgb_ptr;
        cv_bridge::CvImageConstPtr cv_depth_ptr;
        try {
          // get rgb frame and resize
          cv_rgb_ptr = cv_bridge::toCvShare(image_msg_, sensor_msgs::image_encodings::TYPE_8UC3);
          cv::resize(cv_rgb_ptr->image, frame, frame.size(), 0, 0, CV_INTER_NN);

          // get depth frame and resize
          cv_depth_ptr = cv_bridge::toCvShare(depth_msg_, sensor_msgs::image_encodings::TYPE_32FC1);
          //cv::Mat tmpDepthFrame(imgHeight, imgWidth, CV_32FC1);
          cv::resize(cv_depth_ptr->image, depthFrame, depthFrame.size(), 0, 0, CV_INTER_NN);

          if (convertToMeters_) {
            depthFrame *= (1 / 1000.0); //convert from mm to m
          }

          if (convertToRGB_) {
            cv::cvtColor(frame, frame, CV_BGR2RGB);
          }

          if (performDepthRegistration_) {
            // register depth frame with color frame (i.e., so that depth frame is aligned pixel-to-pixel with color frame)
            cv::Mat R = cameras->getStereoParameters(0)->R;
            cv::Mat T = cameras->getStereoParameters(0)->T;
            cv::Mat transform = (cv::Mat_<double>(4, 4) << R.at<float>(0,0), R.at<float>(0,1), R.at<float>(0,2), T.at<float>(0),
                    R.at<float>(1,0), R.at<float>(1,1), R.at<float>(1,2), T.at<float>(1),
                    R.at<float>(2,0), R.at<float>(2,1), R.at<float>(2,2), T.at<float>(2),
                    0.0, 0.0, 0.0, 1.0);

            cv::rgbd::registerDepth(cameras->getCameraParameters(0)->M,
                                    cameras->getCameraParameters(1)->M,
                                    cameras->getCameraParameters(1)->D,
                                    transform,
                                    depthFrame,
                                    frame.size(),
                                    depthFrame);
          }

          // create point cloud from color and depth frames
          ade::pc::util::depthAndColorToPointCloud(depthFrame, frame, cloudRGB, cloud);

        } catch (cv_bridge::Exception &e) {
          LOG4CXX_ERROR(logger, boost::format("cv_bridge exception: %s") % e.what());
          data_received_ = false;
          return false;
        }

        data_received_ = false;
        return true;
      } else {
        return false;
      }
    }

    void CaptureROS::captureCallback_MONO(const sensor_msgs::Image::ConstPtr &msg) {
      LOG4CXX_DEBUG(logger,
                    boost::format(
                        "captureCallback_MONO - I got an image with size: [%dx%d], encoding[%s], timestamp: [%d-%d], frame id: [%s]") %
                        msg->width %
                        msg->height % msg->encoding.c_str() % msg->header.stamp.sec % msg->header.stamp.nsec %
                        msg->header.frame_id.c_str());

      boost::mutex::scoped_lock lock(data_mutex_);
      image_msg_ = msg;
      data_received_ = true;
      condition_.notify_one();
    }

    void CaptureROS::captureCallback_STEREO(const sensor_msgs::Image::ConstPtr &msg,
                                            const sensor_msgs::Image::ConstPtr &msg2) {
      LOG4CXX_DEBUG(logger,
                    boost::format(
                        "captureCallback_STEREO - I got an image with size: [%dx%d], encoding[%s], timestamp: [%d-%d], frame id: [%s]") %
                        msg->width %
                        msg->height % msg->encoding.c_str() % msg->header.stamp.sec % msg->header.stamp.nsec %
                        msg->header.frame_id.c_str());

      boost::mutex::scoped_lock lock(data_mutex_);
      image_msg_ = msg;
      image2_msg_ = msg2;
      data_received_ = true;
      condition_.notify_one();
    }

    void CaptureROS::captureCallback_RGB_AND_DEPTH(const sensor_msgs::ImageConstPtr &rgb_msg,
                                                   const sensor_msgs::ImageConstPtr &depth_msg) {
      LOG4CXX_DEBUG(logger, "captureCallback_RGB_AND_DEPTH");

      boost::mutex::scoped_lock lock(data_mutex_);
      image_msg_ = rgb_msg;
      depth_msg_ = depth_msg;
      data_received_ = true;
      condition_.notify_one();
    }

    void CaptureROS::captureCallback_POINT_CLOUD(const sensor_msgs::PointCloud2::ConstPtr &msg) {
      LOG4CXX_DEBUG(logger,
                    boost::format(
                        "captureCallback_POINT_CLOUD - I got a point cloud with timestamp: [%d-%d], frame id: [%s] with size: (%d x %d)") %
                        msg->header.stamp.sec % msg->header.stamp.nsec % msg->header.frame_id.c_str() % msg->width %
                        msg->height);

      boost::mutex::scoped_lock lock(data_mutex_);
      point_cloud_msg_ = msg;
      data_received_ = true;
      condition_.notify_one();
    }

    //int main(int argc, char **argv) {
    //    int width = 640;
    //    int height = 480;
    //    IplImage* rgbImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
    //    cvNamedWindow("rgb", CV_WINDOW_AUTOSIZE);
    //
    //    ros::init(argc, argv, "Vision Capture Listener");
    //
    //    ros::NodeHandle n_;
    //
    //    ros::Subscriber sub = n_->subscribe("/camera/image_raw", 1000, chatterCallback);
    //
    //    ros::spin();
    //
    //    cvReleaseImage(&rgbImg);
    //    return 0;
    //}


  } //namespace capture
} //namespace ade  
