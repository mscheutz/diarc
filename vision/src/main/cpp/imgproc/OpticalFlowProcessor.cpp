/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "OpticalFlowProcessor.hpp"
#include "common/notification/CaptureNotification.hpp"
#include "common/notification/MotionNotification.hpp"
#include "display/Display.hpp"

OpticalFlowProcessor::OpticalFlowProcessor(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight, const bool isStereo)
: ImageProcessor(processorId, imgWidth, imgHeight, isStereo),
opticalFlow(),
  grayPrevFrame(),
  grayCurrentFrame() {
  logger = log4cxx::Logger::getLogger("ade.imgproc.OpticalFlowProcessor");

//  grayPrevFrame.create(imgHeight, imgWidth, CV_8UC1);
//  grayCurrentFrame.create(imgHeight, imgWidth, CV_8UC1);
//  opticalFlow.create(imgHeight, imgWidth, CV_32FC2);
}

OpticalFlowProcessor::~OpticalFlowProcessor() {
}

void OpticalFlowProcessor::handleCaptureNotification(CaptureNotification::ConstPtr notification) {

  // update capture info
  if (!currCaptureData) {
    // first notification
    currCaptureData = notification->captureData;

    // no previous notification, needed to do optical flow calc
    return;
  } else {
    // update
    prevCaptureData = currCaptureData;
    currCaptureData = notification->captureData;
  }

  LOG4CXX_TRACE(logger, "[handleNotification] trying to calculate optical flow...");
  calcOpticalFlow(prevCaptureData->frame, currCaptureData->frame, opticalFlow);

  MotionNotification::Ptr motionNotification(new MotionNotification(shared_from_this(), notification->frameNumber, notification->captureData, opticalFlow));
  sendNotifications(motionNotification);

  if (getDisplayFlag()) {
    ade::Display::displayFrame(opticalFlow, getDisplayName());
  }
  LOG4CXX_TRACE(logger, "[handleNotification] done.");

  if (is_stereo) {
    calcOpticalFlow(prevCaptureData->frame2, currCaptureData->frame2, opticalFlow);
    ade::Display::displayFrame(opticalFlow, "opticalflow 2");

    LOG4CXX_WARN(logger, "[handleCaptureNotification] motion notifications not sent for second stereo image.");
  }
}

void OpticalFlowProcessor::calcOpticalFlow(const cv::Mat& prevFrame, const cv::Mat& currentFrame, cv::Mat& flow) {
  cv::cvtColor(prevFrame, grayPrevFrame, cv::COLOR_BGR2GRAY);
  cv::cvtColor(currentFrame, grayCurrentFrame, cv::COLOR_BGR2GRAY);
  cv::calcOpticalFlowFarneback(grayPrevFrame, grayCurrentFrame, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
  //CV_32FC2


  cv::Mat mag, ang;
// split flow:
  std::vector<cv::Mat> channels(3);
  cv::split(flow, channels);
  cv::Mat flowX = channels[0];
  cv::Mat flowY = channels[1];
  cv::cartToPolar(flowX, flowY, mag, ang);

  //mag
  mag.convertTo(mag, CV_8U);
  cv::threshold(mag, flow, 1, 255, cv::THRESH_BINARY);
}
