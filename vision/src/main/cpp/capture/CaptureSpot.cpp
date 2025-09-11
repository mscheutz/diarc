/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "CaptureSpot.hpp"
#include "util/CaptureUtilities.hpp"
#include "point_clouds/PointCloudUtilities.hpp"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>


namespace diarc {
  namespace capture {
    CaptureSpot::CaptureSpot(const std::string& configFile)
      : Capture(configFile) {
      // parse configuration params
      loadConfig(configFile);

      // Create a Client SDK object.
      LOG4CXX_INFO(logger, "Creating SDK ------");
      client_sdk = ::bosdyn::client::CreateStandardSDK("get_image");
      LOG4CXX_INFO(logger, "------Created SDK");

      // Create a robot instance. A robot instance represents a single user on a single robot.
      LOG4CXX_INFO(logger, "Creating robot instance ------");
      ::bosdyn::client::Result<std::unique_ptr<::bosdyn::client::Robot>> robot_result = client_sdk->
        CreateRobot(hostname);
      if (!robot_result.status) {
        LOG4CXX_ERROR(logger, boost::format("Could not create robot: %s.") % robot_result.status.DebugString());
      }
      robot = std::move(robot_result.response);
      LOG4CXX_INFO(logger, "------Created Robot");

      LOG4CXX_INFO(logger, "Authenticating with robot ------");
      ::bosdyn::common::Status status = robot->Authenticate(username, password);
      if (!status) {
        LOG4CXX_ERROR(logger, boost::format("Could not authenticate with robot: %s.") % status.DebugString());
      }
      LOG4CXX_INFO(logger, "------Authenticated with Robot");

      // Create an ImageClient.
      LOG4CXX_INFO(logger, "Creating image client ------");
      image_client_result = robot->EnsureServiceClient<::bosdyn::client::ImageClient>(
        ::bosdyn::client::ImageClient::GetDefaultServiceName());
      if (!image_client_result.status) {
        LOG4CXX_ERROR(
          logger, boost::format("Could not create image client: %s.") % image_client_result.status.DebugString());
      }

      LOG4CXX_INFO(logger, "------ Created image client");
    };

    CaptureSpot::~CaptureSpot() {
    };

    void CaptureSpot::loadConfig(const std::string& configFile) {
      // get directory of config file
      unsigned found = configFile.find_last_of("/\\");
      std::string dir = configFile.substr(0, found + 1);

      // extract configuration from xml
      using boost::property_tree::ptree;
      ptree pt;
      read_xml(configFile, pt);

      std::string hostname = pt.get<std::string>("capture.hostname");
      std::string username = pt.get<std::string>("capture.username");
      std::string password = pt.get<std::string>("capture.password");
    }

    bool CaptureSpot::captureCurrentCameraFrame() {
      std::vector imageSources = std::vector<std::string>{"hand_depth_in_hand_color_frame", "hand_color_image"};
      ::bosdyn::client::GetImageResultType get_image_result = image_client_result.response->GetImage(imageSources);

      // Now get the images from the async calls.
      if (!get_image_result.status) {
        LOG4CXX_ERROR(
          logger, boost::format("Could not get hand images: %s") % get_image_result.status.DebugString().c_str());
        return false;
      }

      // populate diarc camera params (only sets them the first time)
      setDepthCameraParameters(get_image_result.response.image_responses()[0]);
      setRGBCameraParameters(get_image_result.response.image_responses()[1]);

      // populate diarc images
      bosdyn::api::Image depth_img = get_image_result.response.image_responses()[0].shot().image();
      cv::Mat cv_depth_img(depth_img.rows(), depth_img.cols(), CV_32FC1, (void*)depth_img.data().data());
      cv::resize(cv_depth_img, depthFrame, depthFrame.size(), 0, 0, cv::INTER_NEAREST);

      bosdyn::api::Image rgb_img = get_image_result.response.image_responses()[1].shot().image();
      cv::Mat cv_rgb_img(rgb_img.rows(), rgb_img.cols(), CV_8UC3, (void*)rgb_img.data().data());
      cv::resize(cv_rgb_img, frame, frame.size(), 0, 0, cv::INTER_NEAREST);

      // create point cloud from color and depth frames
      pc::util::depthAndColorToPointCloud(depthFrame, frame, cloudRGB, cloud);
      return true;
    }

    void CaptureSpot::setRGBCameraParameters(const bosdyn::api::ImageResponse& image_response) const {
      if (cameras->getCameraParameters(1)->intrinsicSet) {
        return;
      }

      //set rgb frame parameters
      int width = image_response.shot().image().cols();
      int height = image_response.shot().image().rows();
      double f_x = image_response.source().pinhole().intrinsics().focal_length().x();
      double f_y = image_response.source().pinhole().intrinsics().focal_length().y();
      double c_x = image_response.source().pinhole().intrinsics().principal_point().x();
      double c_y = image_response.source().pinhole().intrinsics().principal_point().y();
      CameraParameters::Ptr camParams(new CameraParameters(VisionConstants::imgWidth, VisionConstants::imgHeight));
      camParams->M.at<float>(0, 0) = f_x * (static_cast<float>(imgWidth) / width); // f_x
      camParams->M.at<float>(1, 1) = f_y * (static_cast<float>(imgHeight) / height); // f_y
      camParams->M.at<float>(0, 2) = c_x * (static_cast<float>(imgWidth) / width); // c_x
      camParams->M.at<float>(1, 2) = c_y * (static_cast<float>(imgHeight) / height); // c_y
      camParams->intrinsicSet = true;
      cameras->setCameraParameters(1, camParams);
    }

    void CaptureSpot::setDepthCameraParameters(const bosdyn::api::ImageResponse& image_response) const {
      if (cameras->getCameraParameters(0)->intrinsicSet) {
        return;
      }

      // set depth image parameters
      int width = image_response.shot().image().cols();
      int height = image_response.shot().image().rows();
      double f_x = image_response.source().pinhole().intrinsics().focal_length().x();
      double f_y = image_response.source().pinhole().intrinsics().focal_length().y();
      double c_x = image_response.source().pinhole().intrinsics().principal_point().x();
      double c_y = image_response.source().pinhole().intrinsics().principal_point().y();
      CameraParameters::Ptr camParams(new CameraParameters(VisionConstants::imgWidth, VisionConstants::imgHeight));
      camParams->M.at<float>(0, 0) = f_x * (static_cast<float>(imgWidth) / width); // f_x
      camParams->M.at<float>(1, 1) = f_y * (static_cast<float>(imgHeight) / height); // f_y
      camParams->M.at<float>(0, 2) = c_x * (static_cast<float>(imgWidth) / width); // c_x
      camParams->M.at<float>(1, 2) = c_y * (static_cast<float>(imgHeight) / height); // c_y
      camParams->intrinsicSet = true;
      cameras->setCameraParameters(0, camParams);
    }
  } //namespace capture
} //namespace diarc
