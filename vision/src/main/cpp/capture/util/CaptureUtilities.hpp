/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef CAPTUREUTILITIES_HPP
#define CAPTUREUTILITIES_HPP

#include <opencv2/opencv.hpp>
#include "common/VisionConstants.hpp"
#include "capture/calibration/Cameras.hpp"
#include "capture/CaptureEnums.hpp"

namespace diarc {
  namespace capture {
    namespace util {

      // convert string to CAM_MODE enum
      CAM_MODE stringToCAM_MODE(const std::string& str);

      //check if captured frame is "dark" (avg pixel intensity below threshold). thread safe
      // avgDarknessThresh = 26 is same as roughly 6000000 total for 320*240*3 image;
      bool isFrameDark(const cv::Mat frame, const int avgDarknessThresh = 26);

      //undistort captured frame. thread safe
      void undistortFrame(const cv::Mat src, cv::Mat dst, CameraParameters::ConstPtr camParams, int cvInterpolationFlag = cv::INTER_LINEAR);

      //undistort and re-map depth frame to "match" rgb frame. assumes rgb frame has already been undistorted.
      //only use for depth enabled devices (e.g., kinect)
      void rectifyFrame(const cv::Mat src, cv::Mat dst, StereoCameraParameters::ConstPtr stereoParams);

      //generate disparity and depth maps - pass in distorted images as this method undistorts. not thread safe.
      //resulting depth maps are in meter units
      bool generateDepthMaps(const cv::Mat imageSrcLeft, const cv::Mat imageSrcRight,
              cv::Mat depthMapLeft, cv::Mat depthMapRight,
              StereoCameraParameters::ConstPtr stereoParamsLeft, StereoCameraParameters::ConstPtr stereoParamsRight,
              CameraParameters::ConstPtr camParamsLeft, CameraParameters::ConstPtr camParamsRight);


      //helper for calcPanTilt
      //TODO: this probably shouldn't be here
      double toRadians(const double& deg);
      
      // calculates the unit direction vector from (x,y) pixel coordinates)
      cv::Vec3d calcDirection(const int camIndex, const int x, const int y);

      //calculates the pan,tilt in radians from x,y pixel coordinates.  (pan,tilt) = (0,0) is the center of the frame (ie. (x,y) = (width/2,height/2))
      void calcPanTilt(const int camIndex, const int x, const int y, double& pan, double& tilt);
      void calc3dToPanTiltDist(const int camIndex, const double& x, const double& y, const double& z, double& pan, double& tilt, double& dist);

      //project 3D point onto image plane
      void projectPoint(const float& x, const float& y, const float& z, int& xx, int& yy, const int camIndex);
      void projectPoints(const cv::Mat& objectPoints, std::vector<cv::Point2f>& imagePoints, const int camIndex);
      void projectPoints(const std::vector<cv::Point3f>& objectPoints, std::vector<cv::Point2f>& imagePoints, const int camIndex);

      void depthTo3d(const cv::Mat img_depth, cv::Mat img_3d, const int camIndex);

      void convertFramesBasedOnOpenCVConstant(cv::Mat image, const int cvConstant);
      void convertToRGB(cv::Mat image, COLOR_SPACE currColor);
      void convertColorsIfNecessary(cv::Mat image, COLOR_SPACE currColor, COLOR_SPACE desiredColor);

    } //namespace util
  } //namespace capture
} //namespace diarc
#endif
