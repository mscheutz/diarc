/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef STEREOCALIBRATION_HPP
#define STEREOCALIBRATION_HPP

#include <opencv2/opencv.hpp>
#include <vector>

#include "Cameras.hpp"

class StereoCalibration {
public:

  enum CALIB_MODE {
    RGB,               //two RGB cameras
    DEPTH_RGB_L,       //Depth frame (from SR, kinect, etc) and RGB cam left (ie, frame1),
    DEPTH_RGB_R,       //Depth frame (from SR, kinect, etc) and RGB cam right (ie, frame2),
    NUM_MODES
  };

  StereoCalibration(const int numCornersX, const int numCornersY, const enum CALIB_MODE calibmode);

  ~StereoCalibration();

  int getParametersSize() const;

  void getParameters(double *params);

  bool takeSnapshot();

  void getDatum(int index, double *params2) const;

  void getPoint(const double *params, const double *datum, double *point3d);

private:
  bool findCheckerboardPoints(const int imgIndex);

  void setParameters(const double *params);

  //chckerboard params
  const int cornersX, cornersY;

  //data updated on takeSnapshot
  bool framesInitialized;
  cv::Mat images[2];
  cv::Mat framesToDraw[2];
  std::vector<cv::Point2d> pointsPixel[2];

  //calibration params
  CALIB_MODE calibrationMode;
  Cameras *stereoPair;
  //CameraParameters::Ptr camParams[2];
  //StereoCameraParameters::Ptr stereoParams;

};

#endif  //STEREOCALIBRATION_HPP