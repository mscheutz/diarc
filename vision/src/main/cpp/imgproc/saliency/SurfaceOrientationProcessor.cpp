/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "SurfaceOrientationProcessor.hpp"

#include <opencv2/opencv.hpp>

#include <capture/calibration/Cameras.hpp>

#include "display/Display.hpp"

SurfaceOrientationProcessor::SurfaceOrientationProcessor(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight, const bool isStereo)
: SaliencyProcessor(processorId, imgWidth, imgHeight, isStereo) {
  visionProcessName = "SurfaceOrienationProcessor";
  logger = log4cxx::Logger::getLogger("ade.imgproc.saliency.SurfaceOrienationProcessor");
}

SurfaceOrientationProcessor::~SurfaceOrientationProcessor() {
}

void RelativeHeightProcessor::handlePlaneNotification(PlaneNotification::ConstPtr notification) {
  ExtractedPlane::ConstPtr plane = notification->plane;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud = notification->captureData->cloud;
  cv::Mat img;

  if (!plane->computeFileteredObjectsNormals(cloud)) {
    LOG4CXX_ERROR(logger, "Unable to calculate normals.");
    return;
  }

  pcl::PointIndices::Ptr indices_filtered_objects = plane->getFilteredObjectsIndices();
  pcl::PointCloud<pcl::Normal>::Ptr filtered_objects_normals = plane->getFileteredObjectsNormals();
  pcl::ModelCoefficients::Ptr coefficients = plane->getPlaneCoefficients();

  //updateMask();

  // copy current descriptors
  descriptor_mutex.lock();
  std::vector<std::string> descriptors = processingDescriptors;
  descriptor_mutex.unlock();

  AttentionModule::RelativeSurfaceOrientationMapParameters parameters;
  parameters.width = image.cols;
  parameters.height = image.rows;
  parameters.cloud = cloud;
  parameters.indices = indices_filtered_objects;
  parameters.normals = filtered_objects_normals;
  parameters.normal.normal[0] = coefficients->values[0];
  parameters.normal.normal[1] = coefficients->values[1];
  parameters.normal.normal[2] = coefficients->values[2];
  //mask.copyTo(parameters.mask);

  int processingDescriptorsSize = descriptors.size();
  if (!processingDescriptorsSize) {
    parameters.orientationType = AttentionModule::AM_HORIZONTAL;
    if (AttentionModule::CalculateRelativeSurfaceOrientationMap(parameters)) {
      LOG4CXX_ERROR(logger, "Unable to calculate saliency.");
      return;
    }
  } else {
    // maps to combine
    std::vector<cv::Mat> maps;

    for (int i = 0; i < processingDescriptorsSize; ++i) {
      int orientationType = mapConfig(descriptors.at(i));
      if (orientationType < 0) {
        continue;
      }
      parameters.orientationType = orientationType;
      if (AttentionModule::CalculateRelativeSurfaceOrientationMap(parameters)) {
        LOG4CXX_ERROR(logger, "Unable to calculate saliency.");
        return;
      }

      cv::Mat temp;
      parameters.map.copyTo(temp);
      maps.push_back(temp);
    }

    AttentionModule::CombineMaps(maps, img, AttentionModule::AM_SUM, EPUtils::NT_NONE_REAL);
  }


  boost::lock_guard<boost::mutex> lock(result_mutex);
  img.copyTo(resultImage);
  //mask = cv::Mat_<uchar>::ones(img_height,img_width);

  Notification::Ptr n(new SaliencyNotification(shared_from_this(), notification->captureData->frameNumber, notification->captureData, resultImage));
  sendNotifications(n);

  if (getDisplayFlag()) {
    ade::Display::displayFrame(img, getDisplayName());
  }
}

int SurfaceOrientationProcessor::mapConfig(std::string &orientation) {
  if (!orientation.compare("vertical")) {
    return (AttentionModule::AM_VERTICAL);
  } else if (!orientation.compare("horizontal")) {
    return (AttentionModule::AM_HORIZONTAL);
  }

  return (-1);
}
