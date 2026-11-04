/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef OPTICALFLOWPROCESSOR_HPP
#define OPTICALFLOWPROCESSOR_HPP

#include "common/CaptureData.hpp"
#include "ImageProcessor.hpp"

class OpticalFlowProcessor : public ImageProcessor {
public:
  typedef boost::shared_ptr<OpticalFlowProcessor> Ptr;
  typedef boost::shared_ptr<const OpticalFlowProcessor> ConstPtr;

  OpticalFlowProcessor(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight, const bool isStereo);
  ~OpticalFlowProcessor();

protected:
  virtual void handleCaptureNotification(CaptureNotification::ConstPtr notification);
  
private:

  void calcOpticalFlow(const cv::Mat& prevFrame, const cv::Mat& currentFrame, cv::Mat& flow);

  CaptureData::ConstPtr currCaptureData;
  CaptureData::ConstPtr prevCaptureData;

  cv::Mat grayCurrentFrame;
  cv::Mat grayPrevFrame;
  cv::Mat opticalFlow;
};

#endif  //OPTICALFLOWPROCESSOR_HPP