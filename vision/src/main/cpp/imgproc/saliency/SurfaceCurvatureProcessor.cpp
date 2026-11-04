/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "SurfaceCurvatureProcessor.hpp"

#include <capture/calibration/Cameras.hpp>

#include "display/Display.hpp"

SurfaceCurvatureProcessor::SurfaceCurvatureProcessor(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight, const bool isStereo)
: SaliencyProcessor(processorId, imgWidth, imgHeight, isStereo) {
  visionProcessName = "SurfaceCurvatureProcessor";
  logger = log4cxx::Logger::getLogger("diarc.imgproc.saliency.RelativeHeightProcessor");
}

SurfaceCurvatureProcessor::~SurfaceCurvatureProcessor() {
}

bool SurfaceCurvatureProcessor::handleNotification(Notification::Ptr n) {
  bool result = false;
  if (n->type == Notification::PLANE) {
    PlaneDetectionProcessor::PlaneNotification::Ptr sn = boost::dynamic_pointer_cast<PlaneDetectionProcessor::PlaneNotification>(n);
    if (sn)
      result = haveNewImage(sn->plane);
  } else {
    LOG4CXX_WARN(logger, boost::format("[handleNotification] can't handle notification of type: %d.") % n->type);
  }
  return result;
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

  AttentionModule::SurfaceCurvatureMapParameters parameters;
  parameters.width = image.cols;
  parameters.height = image.rows;
  parameters.cloud = cloud;
  parameters.indices = indices_filtered_objects;
  parameters.normals = filtered_objects_normals;
  //mask.copyTo(parameters.mask);

  int result;
  int processingDescriptorsSize = descriptors.size();
  if (!processingDescriptorsSize) {
    parameters.curvatureType = AttentionModule::AM_CONVEX;
    if (AttentionModule::CalculateSurfaceCurvatureMap(parameters)) {
      LOG4CXX_ERROR(logger, "Unable to calculate saliency.");
      return;
    }
  } else {
    // maps to combine
    std::vector<cv::Mat> maps;

    for (int i = 0; i < processingDescriptorsSize; ++i) {
      int curvatureType = mapConfig(descriptors.at(i));
      if (curvatureType < 0) {
        continue;
      }
      parameters.curvatureType = curvatureType;
      if (AttentionModule::CalculateSurfaceCurvatureMap(parameters)) {
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
    diarc::Display::displayFrame(img, getDisplayName());
  }

}

int SurfaceCurvatureProcessor::mapConfig(std::string &curvature) {
  if (!curvature.compare("flat")) {
    return (AttentionModule::AM_FLAT);
  } else if (!curvature.compare("convex")) {
    return (AttentionModule::AM_CONVEX);
  }

  return (-1);
}
