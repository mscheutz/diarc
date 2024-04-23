/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef CALCULATEOPTICALFLOW_HPP
#define CALCULATEOPTICALFLOW_HPP

#include <opencv2/opencv.hpp>

#include "common/VisionConstants.hpp"

class CalculateOpticalFlow {
public:
  CalculateOpticalFlow (const int imgWidth, const int imgHeight);
  ~CalculateOpticalFlow();

  void calcOpticalFlowFarneback(const cv::Mat& currentFrame, const cv::Mat& prevFrame, cv::Mat& flow);
  void calcOpticalFlowLK (const cv::Mat currentFrame, const cv::Mat prevFrame, cv::Mat results);

private:
    cv::Size size;
    cv::Mat gray_currentFrame;
    cv::Mat gray_prevFrame;
    cv::Mat obj;
    cv::Mat velx;
    cv::Mat vely;
};

#endif
