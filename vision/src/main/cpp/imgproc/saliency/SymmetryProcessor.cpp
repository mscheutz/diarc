/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "SymmetryProcessor.hpp"

#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>

#include <capture/calibration/Cameras.hpp>

#include "display/Display.hpp"

SymmetryProcessor::SymmetryProcessor(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight, const bool isStereo)
: SaliencyProcessor(processorId, imgWidth, imgHeight, isStereo) {
  visionProcessName = "SymmetryProcessor";
  logger = log4cxx::Logger::getLogger("ade.imgproc.saliency.SymmetryProcessor");
}

SymmetryProcessor::~SymmetryProcessor() {
}

void SymmetryProcessor::handleCaptureNotification(CaptureNotification::ConstPtr notification) {
  const cv::Mat image = notification->captureData->frame;

  //updateMask();

  int result;

  // start creating parameters
  AttentionModule::SymmetryMapParameters parameters;
  image.copyTo(parameters.image);
  parameters.width = image.cols;
  parameters.height = image.rows;

  result = AttentionModule::CalculateSymmetryMap(parameters);
  if (result) {
    LOG4CXX_ERROR(logger, "Unable to calculate saliency.");
    return;
  }
  cv::Mat resultImage;
  parameters.map.copyTo(resultImage);

  // WHYDO: MZ: why do we multiply with the mask?
  //cv::Mat mask_float;
  //mask.convertTo(mask_float,CV_32F,1.0f);
  //img = img.mul(mask_float);


  if (getDisplayFlag()) {
    ade::Display::displayFrame(resultImage, getDisplayName());
  }

  Notification::Ptr n(new SaliencyNotification(shared_from_this(), notification->captureData->frameNumber, notification->captureData, resultImage));
  sendNotifications(n);
}