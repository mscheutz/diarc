/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "SiftProcessor.hpp"

#include <boost/thread/locks.hpp>
#include "display/Display.hpp"
#include "common/notification/CaptureNotification.hpp"
#include "common/notification/SiftNotification.hpp"

SiftProcessor::SiftProcessor(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight, const bool isStereo)
: ImageProcessor(processorId, imgWidth, imgHeight, isStereo),
siftHelper(NULL) {
  logger = log4cxx::Logger::getLogger("ade.imgproc.SiftProcessor");
}

SiftProcessor::~SiftProcessor() {
}

void SiftProcessor::handleCaptureNotification(CaptureNotification::ConstPtr notification) {
  if (siftHelper == NULL) {
    siftHelper = new SiftFeatureExtractor(img_width, img_height);
  }
  
  //get captured frames info
  const unsigned long long& frameNum = notification->captureData->frameNumber;
  const cv::Mat currFrame1 = notification->captureData->frame;
  SiftFeatures::Ptr siftFeatures;

  try {
    //perform sift feature extraction
    if (getDisplayFlag()) {
      currFrame1.copyTo(displayFrame);
      siftFeatures = siftHelper->calcSiftDescriptors(frameNum, currFrame1, displayFrame);

      //display results
      ade::Display::displayFrame(displayFrame, getDisplayName());
    } else {
      siftFeatures = siftHelper->calcSiftDescriptors(frameNum, currFrame1);
    }
  } catch (cv::Exception e) {
    LOG4CXX_ERROR(logger, boost::format("OpenCV exception: %s.") % e.what());
  } catch (...) {
    LOG4CXX_ERROR(logger, "catch all exception.");
  }

  SiftNotification::Ptr siftNotification(new SiftNotification(shared_from_this(), siftFeatures->frameNum, notification->captureData, siftFeatures));
  sendNotifications(siftNotification);
}

void SiftProcessor::cleanup() {
  if (siftHelper != NULL) {
    delete siftHelper;
    siftHelper = NULL;
  }
}
