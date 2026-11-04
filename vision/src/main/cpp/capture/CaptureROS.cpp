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

namespace diarc {
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
              rosNamespace_("diarc_vision_cap") {

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
      rosNamespace_ = pt.get<std::string>("capture.namespace", "diarc_vision_cap");
      std::string depth_to_color_T = pt.get<std::string>("capture.depthToColorT", "");

      if (depth_to_color_T != "") {
        std::istringstream iss(depth_to_color_T);
        float x, y, z;
        if (iss >> x >> y >> z) {
          depth_to_color_T_ = (cv::Mat_<float>(3, 1) << x, y, z);
        }
      }

      //initialize ROS
      LOG4CXX_DEBUG(logger, "Initializing ROS.");
      std::vector<std::pair<std::string, std::string> > remapping;
      ros::init(remapping, rosNamespace_);

      //create node handle
      LOG4CXX_DEBUG(logger, "Initializing ROS node.");
      n_ = new ros::NodeHandle();

      //create image transport
      LOG4CXX_DEBUG(logger, "Initializing image transport.");
      it_ = new image_transport::ImageTransport(*n_);

      if (image2_topic_.empty() && depth_topic_.empty()) {
        // single RGB case
        mode_ = MONO;

        if (camera_info_topic.empty()) {
          LOG4CXX_WARN(logger, "cameraInfoTopic not set. Using default camera parameters.");
        }

        single_image_sub_ = it_->subscribe(image_topic_, 10, &CaptureROS::captureCallback_MONO, this);
      } else if (depth_topic_.empty()) {
        // stereo RGB case
        mode_ = STEREO;

        // TODO: get camera paramaters for the stero case

        image_sub_ = new image_transport::SubscriberFilter(*it_, image_topic_, 1);
        image2_sub_ = new image_transport::SubscriberFilter(*it_, image2_topic_, 1);
        sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *image_sub_, *image2_sub_);
        sync_->registerCallback(boost::bind(&CaptureROS::captureCallback_RGB_AND_DEPTH, this, _1, _2));
      } else {
        // singe RGB and single depth case
        mode_ = RGB_AND_DEPTH;

        if (camera_info_topic.empty() || depth_info_topic.empty()) {
          LOG4CXX_WARN(logger, "cameraInfoTopic and depthInfoTopic not set. Using default camera parameters.");
        }

        image_sub_ = new image_transport::SubscriberFilter(*it_, image_topic_, 1);
        depth_sub_ = new image_transport::SubscriberFilter(*it_, depth_topic_, 1);
        sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *image_sub_, *depth_sub_);
        sync_->registerCallback(boost::bind(&CaptureROS::captureCallback_RGB_AND_DEPTH, this, _1, _2));
      }

      if (!camera_info_topic.empty()) {
        camera_info_sub_ = n_->subscribe<sensor_msgs::CameraInfo>(camera_info_topic, 10, boost::bind(&CaptureROS::cameraInfoCallback, this, _1));
      }

      if (!depth_info_topic.empty()) {
        depth_info_sub_ = n_->subscribe<sensor_msgs::CameraInfo>(depth_info_topic, 10, boost::bind(&CaptureROS::depthInfoCallback, this, _1));
      }


      LOG4CXX_DEBUG(logger, "starting async spinner.");
      spinner_ = new ros::AsyncSpinner(1);
      spinner_->start();
      LOG4CXX_DEBUG(logger, "async spinner started.");

      //init depth frame
      depthFrame.create(imgHeight, imgWidth, CV_32FC1);
      LOG4CXX_DEBUG(logger, "Finished constructing.");
    }

    CaptureROS::~CaptureROS() {
      LOG4CXX_DEBUG(logger, "destructor.");

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
      LOG4CXX_DEBUG(logger, "interruptWait.");
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

            LOG4CXX_DEBUG(logger, "performDepthRegistration - transform: " << transform);
            cv::rgbd::registerDepth(cameras->getCameraParameters(0)->M,
                                    cameras->getCameraParameters(1)->M,
                                    cameras->getCameraParameters(1)->D,
                                    transform,
                                    depthFrame,
                                    frame.size(),
                                    depthFrame);
          }

          // create point cloud from color and depth frames
          if (performDepthRegistration_) {
            // use color camera matrix bc depth has already been aligned with color frame
            diarc::pc::util::depthAndColorToPointCloud(depthFrame, frame, cloudRGB, cloud, 1);
          } else {
            // use depth camera matrix
            diarc::pc::util::depthAndColorToPointCloud(depthFrame, frame, cloudRGB, cloud, 0);
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

    void CaptureROS::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg) {
      LOG4CXX_DEBUG(logger, "cameraInfoCallback called");

      // unsubscribe
      camera_info_sub_.shutdown();

      //set rgb frame parameters
      CameraParameters::Ptr camParams(new CameraParameters(VisionConstants::imgWidth, VisionConstants::imgHeight));
      camParams->M.at<float>(0, 0) = msg->K.at(0) * (static_cast<float>(imgWidth) / msg->width);   // f_x
      camParams->M.at<float>(1, 1) = msg->K.at(4) * (static_cast<float>(imgHeight) / msg->height); // f_y
      camParams->M.at<float>(0, 2) = msg->K.at(2) * (static_cast<float>(imgWidth) / msg->width);   // c_x
      camParams->M.at<float>(1, 2) = msg->K.at(5) * (static_cast<float>(imgHeight) / msg->height); // c_y
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
                            msg->width % msg->height % msg->K.at(0) % msg->K.at(4) % msg->K.at(2) % msg->K.at(5));
    }

    void CaptureROS::depthInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg) {
      LOG4CXX_DEBUG(logger, "depthInfoCallback called");

      // unsubscribe
      depth_info_sub_.shutdown();

      // set depth image parameters
      CameraParameters::Ptr camParams(new CameraParameters(VisionConstants::imgWidth, VisionConstants::imgHeight));
      camParams->M.at<float>(0, 0) = msg->K.at(0) * (static_cast<float>(imgWidth) / msg->width);   // f_x
      camParams->M.at<float>(1, 1) = msg->K.at(4) * (static_cast<float>(imgHeight) / msg->height); // f_y
      camParams->M.at<float>(0, 2) = msg->K.at(2) * (static_cast<float>(imgWidth) / msg->width);   // c_x
      camParams->M.at<float>(1, 2) = msg->K.at(5) * (static_cast<float>(imgHeight) / msg->height); // c_y
      camParams->intrinsicSet = true;
      cameras->setCameraParameters(0, camParams);

      // set extrinsic camera parameters between depth and color frame
      StereoCameraParameters::Ptr stereoParams(new StereoCameraParameters(VisionConstants::imgWidth, VisionConstants::imgHeight));
      if (depth_to_color_T_.empty()) {
        stereoParams->T.at<float>(0) = msg->P.at(3); // T_x
        stereoParams->T.at<float>(1) = msg->P.at(7); // T_y
        stereoParams->T.at<float>(2) = msg->P.at(11); // T_z
      } else {
        stereoParams->T.at<float>(0) = depth_to_color_T_.at<float>(0); // T_x
        stereoParams->T.at<float>(1) = depth_to_color_T_.at<float>(1);  // T_y
        stereoParams->T.at<float>(2) = depth_to_color_T_.at<float>(2);  // T_z
      }
      stereoParams->set = true;
      cameras->setStereoParameters(0, stereoParams);

      LOG4CXX_DEBUG(logger, boost::format("depthInfoCallback - cam_info: [%dx%d], [fx,fy,cx,cy][%d,%d,%d,%d]") %
                            msg->width % msg->height % msg->K.at(0) % msg->K.at(4) % msg->K.at(2) % msg->K.at(5));

      LOG4CXX_DEBUG(logger, boost::format("depthInfoCallback - cam_info extrinsic: [Tx,Ty,Tz][%d,%d,%d]") %
                            msg->P.at(3) % msg->P.at(7) % msg->P.at(11));
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
} //namespace diarc
