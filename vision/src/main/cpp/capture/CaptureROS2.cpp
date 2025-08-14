/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "CaptureROS2.hpp"
#include "point_clouds/PointCloudUtilities.hpp"
#include "display/Display.hpp"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <opencv2/rgbd.hpp>
#include <pcl/common/transforms.h>
#include <utility>      //std::pair

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

namespace diarc {
  namespace capture {

    CaptureROS2::CaptureROS2(const std::string &configFile)
            : Capture(configFile),
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
      std::string camera_info_topic = pt.get<std::string>("capture.cameraInfoTopic", "");
      std::string depth_info_topic = pt.get<std::string>("capture.depthInfoTopic", "");
      convertToMeters_ = pt.get<bool>("capture.convertToMeters", false);
      convertToRGB_ = pt.get<bool>("capture.convertToRGB", false);
      performDepthRegistration_ = pt.get<bool>("capture.depthRegistration", false);
      rosNamespace_ = pt.get<std::string>("capture.namespace", "diarc_vision_capture");

      //initialize ROS
      LOG4CXX_DEBUG(logger, "Initializing ROS2.");
      int argCount = 0;
      char **argv = NULL;
      rclcpp::init(argCount, argv);

      //create node handle
      LOG4CXX_DEBUG(logger, "Initializing ROS2 node.");
      n_ = rclcpp::Node::make_shared("diarc_vision_capture");

      //create image transport
      LOG4CXX_DEBUG(logger, "Initializing image transport.");
      it_ = new image_transport::ImageTransport(n_);

      if (image2_topic_.empty() && depth_topic_.empty()) {
        // single RGB case
        mode_ = MONO;

        if (camera_info_topic.empty()) {
          LOG4CXX_WARN(logger, "cameraInfoTopic not set. Using default camera parameters.");
        }

        single_image_sub_ = it_->subscribe(image_topic_, 10, &CaptureROS2::captureCallback_MONO, this);
      } else if (depth_topic_.empty()) {
        // stereo RGB case
        mode_ = STEREO;

        // TODO: get camera paramaters for the stero case

        image_sub_.subscribe(n_.get(), image_topic_, image_transport::TransportHints(n_.get()).getTransport());
        image2_sub_.subscribe(n_.get(), image2_topic_, image_transport::TransportHints(n_.get()).getTransport());
        sync_ = std::make_shared<message_filters::Synchronizer<approximate_policy>>(approximate_policy(10), image_sub_, image2_sub_);
        sync_->registerCallback(std::bind(&CaptureROS2::captureCallback_STEREO, this, std::placeholders::_1, std::placeholders::_2));
      } else {
        // singe RGB and single depth case
        mode_ = RGB_AND_DEPTH;

        if (camera_info_topic.empty() || depth_info_topic.empty()) {
          LOG4CXX_WARN(logger, "cameraInfoTopic and depthInfoTopic not set. Using default camera parameters.");
        }

        image_sub_.subscribe(n_.get(), image_topic_, image_transport::TransportHints(n_.get()).getTransport());
        depth_sub_.subscribe(n_.get(), depth_topic_, image_transport::TransportHints(n_.get()).getTransport());
        sync_ = std::make_shared<message_filters::Synchronizer<approximate_policy>>(approximate_policy(10), image_sub_, depth_sub_);
        sync_->registerCallback(std::bind(&CaptureROS2::captureCallback_RGB_AND_DEPTH, this, std::placeholders::_1, std::placeholders::_2));
      }

      if (!camera_info_topic.empty()) {
        camera_info_sub_ = n_->create_subscription<sensor_msgs::msg::CameraInfo>(camera_info_topic, 10, std::bind(&CaptureROS2::cameraInfoCallback, this, std::placeholders::_1));
      }

      if (!depth_info_topic.empty()) {
        depth_info_sub_ = n_->create_subscription<sensor_msgs::msg::CameraInfo>(depth_info_topic, 10, std::bind(&CaptureROS2::depthInfoCallback, this, std::placeholders::_1));
      }

      LOG4CXX_DEBUG(logger, "starting spinner...");
      spin_thread_ = std::thread(&CaptureROS2::spin, this);
      LOG4CXX_DEBUG(logger, "spinner started.");

      //init depth frame
      depthFrame.create(imgHeight, imgWidth, CV_32FC1);
      LOG4CXX_DEBUG(logger, "Finished constructing.");
    }

    void CaptureROS2::spin() {
      rclcpp::spin(n_);
    }

    CaptureROS2::~CaptureROS2() {
      LOG4CXX_DEBUG(logger, "destructor.");
      rclcpp::shutdown();
      spin_thread_.join();

      delete it_;
    }

    void CaptureROS2::interruptWait() {
      LOG4CXX_DEBUG(logger, "interruptWait.");
      boost::mutex::scoped_lock lock(data_mutex_);
      interrupt_flag_ = true;
      condition_.notify_all();
    }

    bool CaptureROS2::captureCurrentCameraFrame() {
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

    bool CaptureROS2::capture_MONO() {
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
          cv_bridge::CvImagePtr cv_rgb_ptr = cv_bridge::toCvCopy(image_msg_, sensor_msgs::image_encodings::TYPE_8UC3);
          cv::resize(cv_rgb_ptr->image, frame, frame.size(), 0, 0, CV_INTER_NN);

          if (convertToRGB_) {
            cv::cvtColor(frame, frame, CV_BGR2RGB);
          }
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

    bool CaptureROS2::capture_STEREO() {
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

        if (convertToRGB_) {
          cv::cvtColor(frame, frame, CV_BGR2RGB);
          cv::cvtColor(frame2, frame2, CV_BGR2RGB);
        }
        data_received_ = false;
        return true;
      } else {
        return false;
      }

    }

    bool CaptureROS2::capture_RGB_AND_DEPTH() {
      // LOG4CXX_DEBUG(logger, "capture_RGB_AND_DEPTH");

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
          diarc::pc::util::depthAndColorToPointCloud(depthFrame, frame, cloudRGB, cloud);

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

    void CaptureROS2::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg) {
      LOG4CXX_DEBUG(logger, "cameraInfoCallback called");

      // unsubscribe
      camera_info_sub_.reset();

      //set rgb frame parameters
      CameraParameters::Ptr camParams(new CameraParameters(VisionConstants::imgWidth, VisionConstants::imgHeight));
      camParams->M.at<float>(0, 0) = msg->k.at(0) * (static_cast<float>(imgWidth) / msg->width);   // f_x
      camParams->M.at<float>(1, 1) = msg->k.at(4) * (static_cast<float>(imgHeight) / msg->height); // f_y
      camParams->M.at<float>(0, 2) = msg->k.at(2) * (static_cast<float>(imgWidth) / msg->width);   // c_x
      camParams->M.at<float>(1, 2) = msg->k.at(5) * (static_cast<float>(imgHeight) / msg->height); // c_y
      camParams->intrinsicSet = true;

      switch (mode_) {
        case MONO:
        case STEREO:
          cameras->setCameraParameters(0, camParams);
          break;
        case RGB_AND_DEPTH:
          cameras->setCameraParameters(1, camParams);
          break;
        default:
          LOG4CXX_WARN(logger, "cameraInfoCallback called for unhandled CAM_MODE.");
      }

      LOG4CXX_DEBUG(logger, boost::format("cameraInfoCallback - cam_info: [%dx%d], [fx,fy,cx,cy][%d,%d,%d,%d]") %
                        msg->width % msg->height % msg->k.at(0) % msg->k.at(4) % msg->k.at(2) % msg->k.at(5));
    }

    void CaptureROS2::depthInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg) {
      LOG4CXX_DEBUG(logger, "depthInfoCallback called");

      // unsubscribe
      depth_info_sub_.reset();

      // set depth image parameters
      CameraParameters::Ptr camParams(new CameraParameters(VisionConstants::imgWidth, VisionConstants::imgHeight));
      camParams->M.at<float>(0, 0) = msg->k.at(0) * (static_cast<float>(imgWidth) / msg->width);   // f_x
      camParams->M.at<float>(1, 1) = msg->k.at(4) * (static_cast<float>(imgHeight) / msg->height); // f_y
      camParams->M.at<float>(0, 2) = msg->k.at(2) * (static_cast<float>(imgWidth) / msg->width);   // c_x
      camParams->M.at<float>(1, 2) = msg->k.at(5) * (static_cast<float>(imgHeight) / msg->height); // c_y
      camParams->intrinsicSet = true;
      cameras->setCameraParameters(0, camParams);

      LOG4CXX_DEBUG(logger, boost::format("depthInfoCallback - cam_info: [%dx%d], [fx,fy,cx,cy][%d,%d,%d,%d]") %
                        msg->width % msg->height % msg->k.at(0) % msg->k.at(4) % msg->k.at(2) % msg->k.at(5));
    }

    void CaptureROS2::captureCallback_MONO(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
      LOG4CXX_DEBUG(logger,
                    boost::format(
                        "captureCallback_MONO - I got an image with size: [%dx%d], encoding[%s], timestamp: [%d-%d], frame id: [%s]") %
                        msg->width %
                        msg->height % msg->encoding.c_str() % msg->header.stamp.sec % msg->header.stamp.nanosec %
                        msg->header.frame_id.c_str());

      boost::mutex::scoped_lock lock(data_mutex_);
      image_msg_ = msg;
      data_received_ = true;
      condition_.notify_one();
    }

    void CaptureROS2::captureCallback_STEREO(const sensor_msgs::msg::Image::ConstSharedPtr &msg,
                                            const sensor_msgs::msg::Image::ConstSharedPtr &msg2) {
      LOG4CXX_DEBUG(logger,
                    boost::format(
                        "captureCallback_STEREO - I got an image with size: [%dx%d], encoding[%s], timestamp: [%d-%d], frame id: [%s]") %
                        msg->width %
                        msg->height % msg->encoding.c_str() % msg->header.stamp.sec % msg->header.stamp.nanosec %
                        msg->header.frame_id.c_str());

      boost::mutex::scoped_lock lock(data_mutex_);
      image_msg_ = msg;
      image2_msg_ = msg2;
      data_received_ = true;
      condition_.notify_one();
    }

    void CaptureROS2::captureCallback_RGB_AND_DEPTH(const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
                                                   const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg) {
      //LOG4CXX_DEBUG(logger, "captureCallback_RGB_AND_DEPTH");

      boost::mutex::scoped_lock lock(data_mutex_);
      image_msg_ = rgb_msg;
      depth_msg_ = depth_msg;
      data_received_ = true;
      condition_.notify_one();
    }

    void CaptureROS2::captureCallback_POINT_CLOUD(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
      LOG4CXX_DEBUG(logger,
                    boost::format(
                        "captureCallback_POINT_CLOUD - I got a point cloud with timestamp: [%d-%d], frame id: [%s] with size: (%d x %d)") %
                        msg->header.stamp.sec % msg->header.stamp.nanosec % msg->header.frame_id.c_str() % msg->width %
                        msg->height);

      boost::mutex::scoped_lock lock(data_mutex_);
      point_cloud_msg_ = msg;
      data_received_ = true;
      condition_.notify_one();
    }

  } //namespace capture
} //namespace diarc
