/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

//
// Created by evan on 2/25/20.
//

#include "MotionDetector.hpp"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <fstream>

#include "capture/util/CaptureUtilities.hpp"
#include "display/Display.hpp"

using namespace ade::stm;

MotionDetector::MotionDetector(const long long &processorId, const int imgWidth, const int imgHeight)
    : ObjectDetector(processorId, imgWidth, imgHeight) {
  visionProcessName = "MotionDetector";
  logger = log4cxx::Logger::getLogger("ade.detector.MotionDetector");
}

MotionDetector::~MotionDetector() {

}

void MotionDetector::registerForCaptureNotification() {
  // intentionally empty
}

void MotionDetector::unregisterForCaptureNotification() {
  // intentionally empty
}

void MotionDetector::handleMotionNotification(MotionNotification::ConstPtr notification) {
  const cv::Mat motionFrame = notification->motionFrame;
  const cv::Mat currentFrame = notification->captureData->frame;

  // segment motion frame into clusters
  cv::Mat labelImage(motionFrame.size(), CV_32S);
  cv::Mat stat, centroid;
  int nLabels = cv::connectedComponentsWithStats(motionFrame, labelImage, stat, centroid, 8);

  MemoryObject::VecPtr newBlobs(new MemoryObject::Vec());

  // get current descriptors to process
  DescriptorsByTypeConstPtr types = getTypes();
  DescriptorsByType::const_iterator types_itr;
  PredicateHelper::Set::const_iterator descriptors_itr;

  for (int i = 0; i < nLabels; i++) {
    cv::Rect bb(cv::Point(stat.at<int>(i, cv::CC_STAT_LEFT), stat.at<int>(i, cv::CC_STAT_TOP)),
               cv::Size(stat.at<int>(i, cv::CC_STAT_WIDTH), stat.at<int>(i, cv::CC_STAT_HEIGHT)));

    // EAK: for some reason, there are blobs of the entire image. this weeds those out
    if (bb.x == 0 && bb.y == 0
        && (bb.width == currentFrame.cols)
        && (bb.height == currentFrame.rows)) {
      LOG4CXX_DEBUG(logger, "Blob size matches frame size...ignoring.");
      continue;
    }

    // ignore small bounding boxes
    if ((bb.width < 10) || (bb.height < 10)) {
      LOG4CXX_DEBUG(logger, "Blob size less than 10x10...ignoring.");
      continue;
    }
    LOG4CXX_DEBUG(logger, boost::format("bounding box: %d, %d, %d, %d.") % bb.x % bb.y % bb.width % bb.height);

    // create MemoryObject for each relevant typeId
    for (types_itr = types->begin(); types_itr != types->end(); ++types_itr) {
      std::string varName = types_itr->second.begin()->getArg(0);
      MemoryObject::Ptr newBlob(new MemoryObject(types_itr->first, varName, notification->captureData, bb));
      newBlob->setTrackingConfidenceDecayRate(0.001);

      for (descriptors_itr = types_itr->second.begin(); descriptors_itr != types_itr->second.end(); ++descriptors_itr) {
        newBlob->addValidationResult(0.9, *descriptors_itr);
      }

      //add blob to stack
      newBlobs->push_back(newBlob);
    }
  }

  //draw on blob boxes if displaying blob info
  if (getDisplayFlag()) {
    displayFrame = motionFrame.clone();
    MemoryObject::Vec::iterator newBlobIter;
    int lineThickness = 2;

    for (newBlobIter = newBlobs->begin(); newBlobIter != newBlobs->end(); ++newBlobIter) {
      const cv::Rect &rect = (*newBlobIter)->getDetectionMask()->getBoundingBox();

      //add black and white box to make box easier to see
      cv::rectangle(displayFrame, cv::Point(rect.x + lineThickness, rect.y + lineThickness),
                  cv::Point(rect.x + rect.width - lineThickness, rect.y + rect.height - lineThickness),
                  CV_RGB(0, 0, 0),
                  lineThickness, 8, 0);
      cv::rectangle(displayFrame, cv::Point(rect.x, rect.y),
                  cv::Point(rect.x + rect.width, rect.y + rect.height),
                  CV_RGB(255, 255, 255),
                  lineThickness, 8, 0);
    }

    ade::Display::displayFrame(displayFrame, getDisplayName());
  }

  // send MO notifications
  sendDetectionNotifications(newBlobs);
}
