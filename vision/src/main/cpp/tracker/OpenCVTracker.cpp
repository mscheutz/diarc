/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "OpenCVTracker.hpp"
#include "display/Display.hpp"
#include "capture/util/CaptureUtilities.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>

using namespace ade::stm;

OpenCVTracker::OpenCVTracker(const std::string &trackerType,
                             const long long &processorId,
                             const int imgWidth,
                             const int imgHeight)
    : ObjectTracker(processorId, imgWidth, imgHeight) {
  visionProcessName = "OpenCVTracker";
  logger = log4cxx::Logger::getLogger("ade.tracker.OpenCVTracker");
  type = trackerType;

//  cv::setBreakOnError(true);
}

OpenCVTracker::~OpenCVTracker() {
  trackers.clear();
}

void OpenCVTracker::loadConfig(const std::string &config) {
}

void OpenCVTracker::haveNewImage(CaptureNotification::ConstPtr notification) {
  try {
    LOG4CXX_TRACE(logger, "[haveNewImage] method entered.");
    const cv::Mat currFrame = notification->captureData->frame;

    //perform tracking iteration on all trackers
    cv::Rect2d bbox;
    bool found;

    boost::lock_guard<boost::mutex> lock(trackers_mutex);
    LOG4CXX_TRACE(logger, boost::format("%d OpenCV trackers exist.") % trackers.size());
    for (auto trackers_itr = trackers.begin(); trackers_itr != trackers.end(); ++trackers_itr) {
      // update tracker
      auto tracker = trackers_itr->second;
      if (currFrame.cols != 320 || currFrame.rows != 240 || currFrame.type() != CV_8UC3) {
        LOG4CXX_FATAL(logger, "[haveNewImage] currFrame is not expected dimensions/type!");
      }
      found = tracker->update(currFrame, bbox);

      //update MemoryObject info
      LOG4CXX_TRACE(logger, boost::format("tracked object #%lld.") % trackers_itr->first);
      MemoryObject::Ptr trackedMO = trackedObjects->getById(trackers_itr->first);

      if (found) {
        LOG4CXX_TRACE(logger, "tracking successful");
        MemoryObjectMask::Ptr trackingMask(new MemoryObjectMask(notification->captureData, bbox));
        LOG4CXX_TRACE(logger, "created new tracking mask.");
        trackedMO->addNewTrackingResult(trackingMask, 1.0f); // maybe increase confidence, not set to 1.0?
        LOG4CXX_TRACE(logger, "added new tracking mask to tracked object.");
      } else {
        //decreases confidence
        trackedMO->decayTrackingConfidence();
      LOG4CXX_DEBUG(logger, boost::format("tracking unsuccessful. confidence: %f.") % trackedMO->getTrackingConfidence());
      }

      //send notifications that trackedMO was updated
      sendTrackingNotifications(trackedMO);
    }
  } catch (cv::Exception &e) {
    std::cerr << "[haveNewImage]" << e.what() << std::endl;
    std::cerr << "[haveNewImage]" << e.msg << std::endl;
  }

}

bool OpenCVTracker::canTrack(const MemoryObject::Ptr &newMemObj) {
  cv::Rect box = newMemObj->getDetectionMask()->getBoundingBox();
  if ((box.width < 5) || (box.height < 5)) {
    LOG4CXX_DEBUG(logger, boost::format("[canTrack] ignoring object with size less than 10x10. object size: %dx%d.") %
        box.width % box.height);
    return false;
  }

  return true;
}

void OpenCVTracker::startTracking(const MemoryObject::Ptr &newMemObj) {
  try {
    LOG4CXX_TRACE(logger, "[startTracking] method entered.");
    boost::lock_guard<boost::mutex> lock(trackers_mutex);

    // explicitly call base class method
    ObjectTracker::startTracking(newMemObj);

    // instantiate new Tracker
    LOG4CXX_DEBUG(logger, boost::format("Adding tracker for tokenID: %d.") % newMemObj->getId());

    //cv::Tracker::create is not supported for openCV versions < 3.2
    #if (CV_MAJOR_VERSION < 4 && CV_MINOR_VERSION < 3)
      cv::Ptr<cv::Tracker> tracker = cv::Tracker::create(type);
    #elif (CV_MAJOR_VERSION < 5 && CV_MINOR_VERSION < 5)
      // for versions between 3.3 and 4.4 (inclusive)
      cv::Ptr<cv::Tracker> tracker;
      if (type == "BOOSTING")
        tracker = cv::TrackerBoosting::create();
      else if (type == "MIL")
        tracker = cv::TrackerMIL::create();
      else if (type == "KCF")
        tracker = cv::TrackerKCF::create();
      else if (type == "TLD")
        tracker = cv::TrackerTLD::create();
      else if (type == "MEDIANFLOW")
        tracker = cv::TrackerMedianFlow::create();
      else if (type == "MOSSE")
        tracker = cv::TrackerMOSSE::create();
      else if (type == "CSRT")
        tracker = cv::TrackerCSRT::create();
      else
        LOG4CXX_ERROR(logger, boost::format("OpenCV Tracker Type not supported %s.") % type)
    #else
      // moved to legacy in version 4.5.1
      cv::Ptr<cv::legacy::Tracker> tracker;
      if (type == "BOOSTING")
        tracker = cv::legacy::TrackerBoosting::create();
      else if (type == "MIL")
        tracker = cv::legacy::TrackerMIL::create();
      else if (type == "KCF")
        tracker = cv::legacy::TrackerKCF::create();
      else if (type == "TLD")
        tracker = cv::legacy::TrackerTLD::create();
      else if (type == "MEDIANFLOW")
        tracker = cv::legacy::TrackerMedianFlow::create();
      else if (type == "MOSSE")
        tracker = cv::legacy::TrackerMOSSE::create();
      else if (type == "CSRT")
        tracker = cv::legacy::TrackerCSRT::create();
      else
        LOG4CXX_ERROR(logger, boost::format("OpenCV Tracker Type not supported %s.") % type);
    #endif

    // init new tracker with bounding box detected object
    const cv::Mat detectionFrame = newMemObj->getCaptureData()->frame;

    cv::Rect2i bbox = newMemObj->getDetectionMask()->getBoundingBox();

    if (logger->isDebugEnabled()) {
      LOG4CXX_DEBUG(logger, boost::format("frame: %dx%d. box:(%d,%d) %dx%d.")
          % detectionFrame.cols % detectionFrame.rows
          % bbox.x % bbox.y % bbox.width % bbox.height);
    }
    tracker->init(detectionFrame, bbox);

    // finally, add it to local tracker map
    trackers[newMemObj->getId()] = tracker;
  } catch (cv::Exception &e) {
    std::cerr << "[startTracking]" << e.what() << std::endl;
    std::cerr << "[startTracking]" << e.msg << std::endl;
  }
}

void OpenCVTracker::stopTracking(const MemoryObject::Ptr &existingMemObj) {
  try {
    LOG4CXX_TRACE(logger, "[stopTracking] method entered.");
    boost::lock_guard<boost::mutex> lock(trackers_mutex);

    // explicitly call base class method
    ObjectTracker::stopTracking(existingMemObj);

    long long id = existingMemObj->getId();
    auto trackers_itr = trackers.find(id);

    // check if MO is a top-level MO being tracked (or just a child that's not explicitly tracked)
    if (trackers_itr == trackers.end()) {
      LOG4CXX_DEBUG(logger, boost::format("Tracker does not exist for tokenID: %d.") % id);
      return;
    } else {
      LOG4CXX_DEBUG(logger, boost::format("Removing Tracker for tokenID: %d.") % id);
    }

    // remove tracker
    trackers.erase(trackers_itr);

  } catch (cv::Exception &e) {
    std::cerr << "[stopTracking]" << e.what() << std::endl;
    std::cerr << "[stopTracking]" << e.msg << std::endl;
  }
}
