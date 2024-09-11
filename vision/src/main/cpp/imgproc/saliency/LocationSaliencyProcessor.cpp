/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "LocationSaliencyProcessor.hpp"

#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>

#include "capture/calibration/Cameras.hpp"
#include "display/Display.hpp"

LocationSaliencyProcessor::LocationSaliencyProcessor(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight, const bool isStereo)
: SaliencyProcessor(processorId, imgWidth, imgHeight, isStereo) {
  locationSaliencyMap.setWidth(imgWidth);
  locationSaliencyMap.setHeight(imgHeight);
  visionProcessName = "LocationSaliencyProcessor";
  logger = log4cxx::Logger::getLogger("diarc.imgproc.saliency.LocationSaliencyProcessor");
}

// EAK: FIXME: this method doesn't look like it uses the captured frames to process,
// so not sure what's going on here ??

void LocationSaliencyProcessor::handleCaptureNotification(CaptureNotification::ConstPtr notification) {

  // get current descriptors to process
  TypesByDescriptorConstPtr descriptors = getDescriptors();
  int numDescriptors = descriptors->size();

  if (numDescriptors == 0) {
    LOG4CXX_ERROR(logger, "No descriptors set, returning.");
    return;
  }

  // maps to combine
  int mapsIndex = 0;
  std::vector<cv::Mat> maps;
  maps.resize(numDescriptors);
  cv::Mat temp;

  cv::Mat resultImage;
  int result;
  TypesByDescriptor::const_iterator descriptors_itr = descriptors->begin();
  for (descriptors_itr = descriptors->begin(); descriptors_itr != descriptors->end(); ++descriptors_itr) {
    //std::cerr << "[" << visionProcessName << "::perform]: " << " Processing descriptor " << i << ":" << descriptors.at(i) << std::endl;
    int location = mapConfig(descriptors_itr->first.getName());
    if (location < 0) {
      continue;
    }
    //std::cerr << "[" << visionProcessName << "::perform]: " << " Descriptor " << i << ":" << descriptors.at(i) << " exists "<< std::endl;
    locationSaliencyMap.setLocation(location);
    result = locationSaliencyMap.calculateLocationMap(temp);
    //parameters.location = location;
    //result = AttentionModule::CalculateLocationMap(parameters);
    if (result) {
      LOG4CXX_ERROR(logger, "Unable to calculate saliency.");
      return;
    }

    //parameters.map.copyTo(temp);
    //maps.push_back(temp);
    temp.copyTo(maps.at(mapsIndex++));
  }
  AttentionModule::CombineMaps(maps, resultImage, AttentionModule::AM_SUM, EPUtils::NT_EMPTY);


  Notification::Ptr n(new SaliencyNotification(shared_from_this(), notification->captureData->frameNumber, notification->captureData, resultImage));
  sendNotifications(n);

  if (getDisplayFlag()) {
    diarc::Display::displayFrame(resultImage, getDisplayName());
  }
}

int LocationSaliencyProcessor::mapConfig(const std::string &location) {
  if (!location.compare("center")) {
    return (AttentionModule::AM_CENTER);
  } else if (!location.compare("left_centric")) {
    return (AttentionModule::AM_LEFT_CENTER);
  } else if (!location.compare("left")) {
    return (AttentionModule::AM_LEFT);
  } else if (!location.compare("right_centric")) {
    return (AttentionModule::AM_RIGHT_CENTER);
  } else if (!location.compare("right")) {
    return (AttentionModule::AM_RIGHT);
  } else if (!location.compare("top_centric")) {
    return (AttentionModule::AM_TOP_CENTER);
  } else if (!location.compare("top")) {
    return (AttentionModule::AM_TOP);
  } else if (!location.compare("bottom_centric")) {
    return (AttentionModule::AM_BOTTOM_CENTER);
  } else if (!location.compare("bottom")) {
    return (AttentionModule::AM_BOTTOM);
  }

  return (-1);
}
