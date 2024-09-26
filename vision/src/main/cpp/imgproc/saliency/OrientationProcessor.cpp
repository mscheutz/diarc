/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "OrientationProcessor.hpp"

#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>

#include "capture/calibration/Cameras.hpp"
#include "display/Display.hpp"

OrientationProcessor::OrientationProcessor(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight, const bool isStereo)
: SaliencyProcessor(processorId, imgWidth, imgHeight, isStereo) {
  orientationSaliencyMap.setWidth(imgWidth);
  orientationSaliencyMap.setHeight(imgHeight);
  orientationSaliencyMap.setFilterSize(5 * (imgWidth / 320));
  orientationSaliencyMap.setBandwidth(2); //1.0);20//(2.0 * (imgWidth/320.0));
  visionProcessName = "OrientationProcessor";
  logger = log4cxx::Logger::getLogger("diarc.imgproc.saliency.OrientationProcessor");
}

OrientationProcessor::~OrientationProcessor() {
}
void OrientationProcessor::handleCaptureNotification(CaptureNotification::ConstPtr notification) {
  const cv::Mat image = notification->captureData->frame;

  // get current descriptors to process
  TypesByDescriptorConstPtr descriptors = getDescriptors();
  int numDescriptors = descriptors->size();

  if (numDescriptors == 0) {
    LOG4CXX_ERROR(logger, "No descriptors set, returning.");
    return;
  }

  cv::Mat tmpImage = image;     //bc setImage doesn't take **const** ref like it should 
  orientationSaliencyMap.setImage(tmpImage);

  // maps to combine
  int mapsIndex = 0;
  std::vector<cv::Mat> maps;
  maps.resize(numDescriptors);
  cv::Mat temp;

  TypesByDescriptor::const_iterator descriptors_itr = descriptors->begin();
  for (descriptors_itr = descriptors->begin(); descriptors_itr != descriptors->end(); ++descriptors_itr) {
    float angle = mapConfig(descriptors_itr->first.getName());
    orientationSaliencyMap.setAngle(angle);
    if (orientationSaliencyMap.calculateOrientationMap(temp)) {
      LOG4CXX_ERROR(logger, "Unable to calculate saliency.");
      return;
    }

    temp.copyTo(maps.at(mapsIndex++));
  }

  cv::Mat resultImage;
  AttentionModule::CombineMaps(maps, resultImage, AttentionModule::AM_SUM, EPUtils::NT_EMPTY);

  Notification::Ptr n(new SaliencyNotification(shared_from_this(), notification->captureData->frameNumber, notification->captureData, resultImage));
  sendNotifications(n);

  if (getDisplayFlag()) {
    diarc::Display::displayFrame(resultImage, getDisplayName());
  }
}

int OrientationProcessor::mapConfig(const std::string &orientation) {
  if (!orientation.compare("horizontal")) {
    return (90);
  } else if (!orientation.compare("vertical")) {
    return (0);
  }

  return (-1);

}
