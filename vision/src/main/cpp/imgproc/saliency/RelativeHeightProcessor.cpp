/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "RelativeHeightProcessor.hpp"

#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>

#include <capture/calibration/Cameras.hpp>

#include "display/Display.hpp"

#include <sys/time.h>

RelativeHeightProcessor::RelativeHeightProcessor(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight, const bool isStereo)
: SaliencyProcessor(processorId, imgWidth, imgHeight, isStereo) {
  surfaceHeightSaliencyMap.setWidth(imgWidth);
  surfaceHeightSaliencyMap.setHeight(imgHeight);
  visionProcessName = "RelativeHeightProcessor";
  logger = log4cxx::Logger::getLogger("diarc.imgproc.saliency.RelativeHeightProcessor");
  capturedFrames->registerForNotification(VisionProcess::Ptr(this));
}

RelativeHeightProcessor::~RelativeHeightProcessor() {
}

void RelativeHeightProcessor::handlePlaneNotification(PlaneNotification::ConstPtr notification) {
  ExtractedPlane::ConstPtr plane = notification->plane;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud = notification->captureData->cloud;
  cv::Mat img;

  // copy current descriptors
  descriptor_mutex.lock();
  std::vector<std::string> descriptors = processingDescriptors;
  descriptor_mutex.unlock();

  surfaceHeightSaliencyMap.setCloud(cloud);
  surfaceHeightSaliencyMap.setIndices(plane->getFilteredObjectsIndices());
  surfaceHeightSaliencyMap.setModelCoefficients(plane->getPlaneCoefficients());

  int processingDescriptorsSize = descriptors.size();
  if (!processingDescriptorsSize) {
    surfaceHeightSaliencyMap.setHeightType(AttentionModule::AM_TALL);
    if (surfaceHeightSaliencyMap.calculateSurfaceHeightMap(img)) {
      LOG4CXX_ERROR(logger, "Unable to calculate saliency.");
      return;
    }
  } else {
    // maps to combine
    std::vector<cv::Mat> maps;
    maps.resize(processingDescriptorsSize);
    cv::Mat temp;

    for (int i = 0; i < processingDescriptorsSize; ++i) {
      int heightType = mapConfig(descriptors.at(i));
      if (heightType < 0) {
        continue;
      }
      surfaceHeightSaliencyMap.setHeightType(heightType);
      if (surfaceHeightSaliencyMap.calculateSurfaceHeightMap(temp)) {
        LOG4CXX_ERROR(logger, "Unable to calculate saliency.");
        return;
      }

      //maps.push_back(temp);
      temp.copyTo(maps.at(i));
    }

    AttentionModule::CombineMaps(maps, img, AttentionModule::AM_SUM, EPUtils::NT_EMPTY);
  }

  result_mutex.lock();
  img.copyTo(resultImage);
  result_mutex.unlock();
  
  Notification::Ptr n(new SaliencyNotification(shared_from_this(), notification->captureData->frameNumber, notification->captureData, resultImage));
  sendNotifications(n);
  
  if (getDisplayFlag()) {
    diarc::Display::displayFrame(img, getDisplayName());
  }
}

int RelativeHeightProcessor::mapConfig(std::string &height) {
  if (!height.compare("short")) {
    return (AttentionModule::AM_SHORT);
  } else if (!height.compare("tall")) {
    return (AttentionModule::AM_TALL);
  }

  return (-1);
}
