/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "IKNSaliencyMapProcessor.hpp"

#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>

#include <capture/calibration/Cameras.hpp>

#include "display/Display.hpp"

IKNSaliencyMapProcessor::IKNSaliencyMapProcessor(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight, const bool isStereo)
: SaliencyProcessor(processorId, imgWidth, imgHeight, isStereo) {
  //diarc::Display::createWindowIfDoesNotExist("surfaceheight 2");
  //viewer = new pcl::visualization::CloudViewer("Relative Height Visualizer");
  visionProcessName = "IKNSaliencyMapProcessor";
  logger = log4cxx::Logger::getLogger("diarc.imgproc.saliency.IKNSaliencyMapProcessor");
}

IKNSaliencyMapProcessor::~IKNSaliencyMapProcessor() {
  //delete viewer;
}

void IKNSaliencyMapProcessor::handleCaptureNotification(CaptureNotification::ConstPtr notification) {
  const cv::Mat image = notification->captureData->frame;

  // start creating parameters
  AttentionModule::IKNMapParameters parameters;
  image.copyTo(parameters.image);
  parameters.width = image.cols;
  parameters.height = image.rows;

  if (AttentionModule::CalculateIKNMap(parameters)) {
    LOG4CXX_WARN(logger, "[haveNewImage] unable to calculate IKN saliency.");
    return;
  }
  
  cv::Mat resultImage;
  parameters.map.copyTo(resultImage);

  //cv::Mat mask_float;
  //mask.convertTo(mask_float,CV_32F,1.0f);
  //img = img.mul(mask_float);

  Notification::Ptr n(new SaliencyNotification(shared_from_this(), notification->captureData->frameNumber, notification->captureData, resultImage));
  sendNotifications(n);
  
  if (getDisplayFlag()) {
    diarc::Display::displayFrame(resultImage, getDisplayName());
  }
}
