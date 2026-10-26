/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "TapnetTracker.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "display/Display.hpp"
#include "pyvis/PythonUtils.hpp"

using namespace diarc::stm;

TapnetTracker::TapnetTracker(const long long &processorId, const int imgWidth, const int imgHeight)
        : ObjectTracker(processorId, imgWidth, imgHeight) {
  trackingConfidenceThreshold = 0.2;
  visionProcessName = "TapnetTracker";
  logger = log4cxx::Logger::getLogger("diarc.tracker.TapnetTracker");

  tracker_initialized = false;
  createTracker();
}

TapnetTracker::~TapnetTracker() {
  destroyTracker();
}

void TapnetTracker::loadConfig(const std::string &config) {
}

void TapnetTracker::createTracker() {
  if (!tracker_initialized) {
    LOG4CXX_DEBUG(logger, "[createTracker]");

    PythonHelper* pyInstance = PythonHelper::getInstance();
    try {
      LOG4CXX_DEBUG(logger, "[createTracker] waiting on GIL...");
      pyInstance->acquireGIL();
      LOG4CXX_DEBUG(logger, boost::format("[createTracker] GIL acquired: %d") % PyGILState_Check());

      LOG4CXX_DEBUG(logger, "[createTracker] 2");
      // Equivalent to "from diarc_vision.tapnet.tapnet_wrapper import Tapnet"
      py::module_ module = pyInstance->importModule("diarc_vision.tapnet.tapnet_wrapper");
      LOG4CXX_DEBUG(logger, "[createTracker] 3");
      TapnetClass = module.attr("Tapnet");
      LOG4CXX_DEBUG(logger, "[createTracker] 4");
      // Construct a Python object of class TapnetClass
      tapnet = TapnetClass(py::arg("device") = "cpu");
      LOG4CXX_DEBUG(logger, "[createTracker] 5");
    } catch (py::error_already_set& e) {
      LOG4CXX_ERROR(logger, boost::format("[createTracker] %s") % e.what());
    }

    pyInstance->releaseGIL();
    tracker_initialized = true;
    LOG4CXX_DEBUG(logger, boost::format("[createTracker] done. GIL released: %d") % PyGILState_Check());
  }
}

void TapnetTracker::destroyTracker() {
  if (tracker_initialized) {

    LOG4CXX_DEBUG(logger, "[destroyTracker]...");
    PythonHelper* pyInstance = PythonHelper::getInstance();
    pyInstance->acquireGIL();

    TapnetClass = py::object();
    tapnet = py::object();

    pyInstance->releaseGIL();
    tracker_initialized = false;
    LOG4CXX_DEBUG(logger, "[destroyTracker] done");
  }
}

void TapnetTracker::haveNewImage(CaptureNotification::ConstPtr notification) {
  LOG4CXX_TRACE(logger, "[haveNewImage] method entered.");
  const cv::Mat currFrame = notification->captureData->frame;

  float trackingThresh = 0.5;
  float trackingConf;

  tapnet.attr("update_tracks")(PythonUtils::convertMatToNumpy(currFrame));

  for (const auto& trackerEntry : trackerMap) {
    int tracker_id = trackerEntry.first;
    int mo_id = trackerEntry.second;
    MemoryObject::Ptr trackedMO = trackedObjects->getById(mo_id);

    py::object py_updated_pos = tapnet.attr("get_track_position")(tracker_id);
    if (py_updated_pos.is_none()) {
      // no track update -- decreases confidence
      trackedMO->decayTrackingConfidence();
      LOG4CXX_TRACE(logger, boost::format("tracking unsuccessful. confidence: %f.") % trackedMO->getTrackingConfidence());
    } else {
      // update MO with updated track position and confidence
      LOG4CXX_TRACE(logger, "tracking successful.");
      cv::Rect orig_bbox =  trackedMO->getDetectionMask()->getBoundingBox();
      py::tuple py_updated_pos_tuple = py_updated_pos.cast<py::tuple>();
      // bbox values outside of image will be corrected in MemoryObjectMask
      int x = py_updated_pos_tuple[0].cast<int>() - orig_bbox.width/2;
      int y = py_updated_pos_tuple[1].cast<int>() - orig_bbox.height/2;
      cv::Rect new_bbox = cv::Rect(x, y, orig_bbox.width, orig_bbox.height);
      MemoryObjectMask::Ptr trackingMask(new MemoryObjectMask(notification->captureData, new_bbox));
      trackedMO->addNewTrackingResult(trackingMask, trackingConf);
    }

    //send notifications that trackedMO was updated
    sendTrackingNotifications(trackedMO);
  }
}

bool TapnetTracker::canTrack(const MemoryObject::Ptr &newMemObj) {
  cv::Rect box = newMemObj->getDetectionMask()->getBoundingBox();
  int min_size = 5;
  if ((box.width < min_size) || (box.height < min_size)) {
    LOG4CXX_DEBUG(logger, boost::format("[canTrack] ignoring object with size less than %dx%d. object size: %dx%d.") %
                          min_size % min_size % box.width % box.height);
    return false;
  }

  return true;
}

void TapnetTracker::startTracking(const MemoryObject::Ptr &newMemObj) {
  LOG4CXX_TRACE(logger, "[startTracking] method entered.");

  //explicitly call base class method
  ObjectTracker::startTracking(newMemObj);

  // add object to tracker
  LOG4CXX_DEBUG(logger, boost::format("Adding point to Tapnet tracker for tokenID: %d.") % newMemObj->getId());

  //init new CMT
  const cv::Mat detectionFrame = newMemObj->getCaptureData()->frame;

  cv::Rect box = newMemObj->getDetectionMask()->getBoundingBox();
  LOG4CXX_DEBUG(logger, boost::format("frame: %dx%d. box:(%d,%d)%dx%d.")
                        % detectionFrameGray.cols % detectionFrameGray.rows
                        % box.x % box.y % box.width % box.height);

  // select center of bbox as track point
  int x = box.x + box.width/2;
  int y = box.y + box.height/2;
  py::tuple point =  py::make_tuple(x,y);

  // create numpy frame
  py::array_t<uchar> pyFrame = PythonUtils::convertMatToNumpy(detectionFrame);

  // call tapnet to add new track point
  py::object py_track_id = tapnet->add_track_point(point, pyFrame);

  if (py_track_id.is_none()) {
    LOG4CXX_ERROR(logger, "[startTracking] Can NOT track object with TAPNET until an existing track is removed.");
    ObjectTracker::stopTracking(existingMemObj);
    return;
  }

  //finally, add it to local tracker map
  int track_id = py_track_id.cast<int>();
  trackerMap[newMemObj->getId()] = track_id;
}

void TapnetTracker::stopTracking(const MemoryObject::Ptr &existingMemObj) {
  LOG4CXX_TRACE(logger, "[stopTracking] method entered.");

  //explicitly call base class method
  ObjectTracker::stopTracking(existingMemObj);

  long long mo_id = existingMemObj->getId();
  auto& trackerMap_itr = trackerMap.find(mo_id);
  // check if MO is a top-level MO being tracked (or just a child that's not explicitly tracked)
  if (trackerMap_itr != tapnetMap.end()) {
    LOG4CXX_DEBUG(logger, boost::format("Removing Tapnet tracker for tokenID: %d.") % mo_id);
    tapnet.attr("remove_track_point")(trackerMap_itr->second);
    trackerMap.erase(trackerMap_itr);
  } else {
    LOG4CXX_DEBUG(logger, boost::format("Tapnet tracker does not exist for tokenID: %d.") % mo_id);
  }

}

