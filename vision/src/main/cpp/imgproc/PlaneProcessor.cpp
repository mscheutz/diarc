/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "PlaneProcessor.hpp"

#include "capture/Capture.hpp"
#include "common/notification/PlaneNotification.hpp"
#include "display/Display.hpp"
#include "point_clouds/PCLFunctions.hpp"

#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/common/time.h>
#include <sys/time.h>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>

PlaneProcessor::PlaneProcessor(const long long &processorId,
                               const unsigned int imgWidth,
                               const unsigned int imgHeight,
                               const bool isStereo)
    : ImageProcessor(processorId, imgWidth, imgHeight, isStereo),
      table_height(0.7),
      dist_threshold(0.01),
      alwaysCalcPlane(false) {
  visionProcessName = "PlaneProcessor";
  logger = log4cxx::Logger::getLogger("ade.imgproc.PlaneProcessor");
}

PlaneProcessor::~PlaneProcessor() {
  LOG4CXX_DEBUG(logger, "[destructor] method entered.");
}

void PlaneProcessor::handleCaptureNotification(CaptureNotification::ConstPtr notification) {
  LOG4CXX_DEBUG(logger, "[haveNewImage] in handleCaptureNotification");

  const unsigned long long frameNum = notification->frameNumber;
  const cv::Mat transform = notification->captureData->transform;
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud = notification->captureData->cloudRGB;

  ExtractedPlane::Ptr currentPlane(new ExtractedPlane());
  currentPlane->setCloudRGB(notification->captureData->cloudRGB);
  currentPlane->setTransform(transform);
  currentPlane->setFrameNumber(frameNum);

  LOG4CXX_DEBUG(logger, "[haveNewImage] Starting segmenting plane");
  // If we have not yet successfully found a plane, attempt to segment from scratch
  if (shouldCalculatePlane(transform) || alwaysCalcPlane) {
    LOG4CXX_DEBUG(logger, "Calculating new plane.");

//    pcl::ScopeTime t("Finding new plane.");
    if (!SegmentPlane<pcl::PointXYZRGB>(cloud,
                                        currentPlane->getPlaneIndices(),
                                        currentPlane->getObjectsIndices(),
                                        currentPlane->getPlaneCoefficients(), transform, dist_threshold, table_height)) {
      LOG4CXX_DEBUG(logger, "[haveNewImage] SegmentPlane did not find plane.");
      return;
    }
    // Copy planeCoefficients for future use
    lastPlaneCoefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients());
    lastPlaneCoefficients->values.resize(4);
    lastPlaneCoefficients->values[0] = currentPlane->getPlaneCoefficients()->values[0];
    lastPlaneCoefficients->values[1] = currentPlane->getPlaneCoefficients()->values[1];
    lastPlaneCoefficients->values[2] = currentPlane->getPlaneCoefficients()->values[2];
    lastPlaneCoefficients->values[3] = currentPlane->getPlaneCoefficients()->values[3];
  } else {
    // we have already detected a plane, attempt to find plane and objects indices using previous plane coefficients
    LOG4CXX_DEBUG(logger, "[haveNewImage] Have existing planeCoefficients, calling SegmentPlaneGivenCoeffs");

//    pcl::ScopeTime t("Processing frame with previous plane coeffs.");
    if (!SegmentPlaneGivenCoeffs<pcl::PointXYZRGB>(cloud,
                                                   currentPlane->getPlaneIndices(),
                                                   currentPlane->getObjectsIndices(),
                                                   lastPlaneCoefficients, dist_threshold)) {
      LOG4CXX_DEBUG(logger, "[haveNewImage] SegmentPlaneGivenCoeffs did not find plane.");
      return;
    }
    // populate output planeCoefficients as stored ones
    currentPlane->getPlaneCoefficients() = lastPlaneCoefficients;
  }

  if (logger->isDebugEnabled()) {
    LOG4CXX_DEBUG(logger,
                  boost::format("[haveNewImage] currentPlane plane indices size: %d.")
                      % currentPlane->getPlaneIndices()->indices.size());
    LOG4CXX_DEBUG(logger,
                  boost::format("[haveNewImage] currentPlane object indices size: %d.")
                      % currentPlane->getObjectsIndices()->indices.size());
  }

  if (!filterPointsOnPlane<pcl::PointXYZRGB>(cloud,
                                             currentPlane->getPlaneIndices(),
                                             currentPlane->getObjectsIndices(),
                                             currentPlane->getPlaneCoefficients(),
                                             currentPlane->getFilteredObjectsIndices())) {
    LOG4CXX_DEBUG(logger, "[haveNewImage] filterPointsOnPlane failed to filter points on plane.");
    return;
  }
  LOG4CXX_DEBUG(logger,
                boost::format("[haveNewImage] currentPlane filtered object indices size: %d.")
                    % currentPlane->getFilteredObjectsIndices()->indices.size());

  PlaneNotification::Ptr
      n(new PlaneNotification(shared_from_this(), frameNum, notification->captureData, currentPlane));
  sendNotifications(n);

  if (getDisplayFlag()) {
    cv::Mat temp = cv::Mat_<uchar>::zeros(img_height, img_width);
    pcl::PointIndices::Ptr indices_plane = currentPlane->getPlaneIndices();
    for (int i = 0; i < indices_plane->indices.size(); ++i) {
      int index = indices_plane->indices.at(i);
      int xx = index % img_width;
      int yy = index / img_width;
      temp.at<uchar>(yy, xx) = 255;
    }
    ade::Display::displayFrame(temp, getDisplayName());
  }

  return;
}

bool PlaneProcessor::shouldCalculatePlane(const cv::Mat &newTransform) {
  bool result = false;

  if (lastTransform.empty() || lastPlaneCoefficients == NULL) {
    LOG4CXX_DEBUG(logger, "No plane or transform.");
    result = true;
  } else {

    // check if transform has changed
    double epsilon = 0.001;
    for (int i = 0; i < 12; ++i) {
      if (abs(lastTransform.at<double>(i) - newTransform.at<double>(i)) > epsilon) {
        LOG4CXX_DEBUG(logger, boost::format("Transform has changed. Index: %d. Old: %f. New: %f.")
            % i % lastTransform.at<double>(i) % newTransform.at<double>(i));
        result = true;
        break;
      }
    }
  }

  lastTransform = cv::Mat(newTransform);
  return result;
}

void PlaneProcessor::loadConfig(const std::string &config) {
  // populate tree structure pt
  using boost::property_tree::ptree;
  ptree pt;
  read_xml(config, pt);

  std::string predicateName;

  // traverse pt

  BOOST_FOREACH(ptree::value_type const& dataNode, pt.get_child("processor")) {
          if (dataNode.first.compare("config") == 0) {
            table_height = dataNode.second.get<float>("table_height", table_height);
            dist_threshold = dataNode.second.get<float>("dist_threshold", dist_threshold);
            alwaysCalcPlane = dataNode.second.get<bool>("alwaysCalculatePlane", false);
            LOG4CXX_DEBUG(logger, boost::format("Setting table height: %f.") % table_height);
            LOG4CXX_DEBUG(logger, boost::format("Setting plane inlier dist thresh: %f.") % dist_threshold);
            LOG4CXX_DEBUG(logger, boost::format("Setting plane alwaysCalcPlane to: %s.") % (alwaysCalcPlane ? "true" : "false"));
          }
        }
}