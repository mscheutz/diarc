/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef SWISSRANGERCALIBRATION_HPP
#define SWISSRANGERCALIBRATION_HPP

#include <opencv2/opencv.hpp>
#include <vector>

#include "../Capture.hpp"

#include <pcl/visualization/pcl_visualizer.h>

class DepthSensorCalibration {
public:
  enum CALIB_MODE {
    SR,           //swiss ranger
    KINECT,       //kinect
    NUM_MODES
  };

  DepthSensorCalibration(const int numCornersX, const int numCornersY, const enum CALIB_MODE calibmode);

  ~DepthSensorCalibration();

  bool takeSnapshot();

  void getDatum(int index, double &x, double &y, double &z);

private:
  bool findCheckerboardPoints();

  void display2d();

  void display3d();

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

  //chckerboard params
  const int cornersX, cornersY;

  //data updated on takeSnapshot
  std::vector<cv::Point2d> pointsPixel;
  std::vector<cv::Point3d> points3d;
  cv::Mat imgIntensity;
  cv::Mat img3d;
  cv::Mat frameToDraw;

  //other flags
  CALIB_MODE calibrationMode;
};

#endif  //SWISSRANGERCALIBRATION_HPP
