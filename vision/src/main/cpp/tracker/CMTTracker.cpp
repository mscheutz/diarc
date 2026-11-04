/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "CMTTracker.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "display/Display.hpp"
#include "capture/util/CaptureUtilities.hpp"

using namespace diarc::stm;

CMTTracker::CMTTracker(const long long &processorId, const int imgWidth, const int imgHeight)
        : ObjectTracker(processorId, imgWidth, imgHeight),
          cmts(),
          lastFrameGray() {
  trackingConfidenceThreshold = 0.2;
  visionProcessName = "CMTTracker";
  logger = log4cxx::Logger::getLogger("diarc.tracker.CMTTracker");
}

CMTTracker::~CMTTracker() {
  std::tr1::unordered_map<long long, cmt::CMT*>::const_iterator itr;
  for (itr = cmts.begin(); itr != cmts.end(); ++itr) {
    delete itr->second;
  }
  cmts.clear();
}

void CMTTracker::loadConfig(const std::string &config) {
}

void CMTTracker::haveNewImage(CaptureNotification::ConstPtr notification) {
  LOG4CXX_TRACE(logger, "[haveNewImage] method entered.");
  const cv::Mat currFrame = notification->captureData->frame;
  cv::Mat currFrameGray;
  cv::cvtColor(currFrame, currFrameGray, cv::COLOR_RGB2GRAY);

  //perform tracking iteration on all CMT trackers
  cv::Rect bbox;
  std::vector<cv::Point2f> pts1;
  std::vector<cv::Point2f> pts2;
  float trackingThresh = 0.5;
  float trackingConf;

  boost::lock_guard<boost::mutex> lock(cmts_mutex);
  CMT_MAP::iterator cmts_itr;
  LOG4CXX_TRACE(logger, boost::format("%d CMT trackers exist.") % cmts.size());
  for (cmts_itr = cmts.begin(); cmts_itr != cmts.end(); ++cmts_itr) {
    pts1.clear();
    pts2.clear();
    cmt::CMT *cmt = cmts_itr->second;
    cmt->processFrame(currFrameGray);
    trackingConf = cmt->getConfidence();

    //update MemoryObject info
    LOG4CXX_TRACE(logger, boost::format("tracked object #%lld.") % cmts_itr->first);
    MemoryObject::Ptr trackedMO = trackedObjects->getById(cmts_itr->first);

    if (trackingConf > trackingThresh) {
      LOG4CXX_TRACE(logger, boost::format("tracking successful. confidence: %f.") % trackingConf);
      bbox = cmt->bb_rot.boundingRect();
      MemoryObjectMask::Ptr trackingMask(new MemoryObjectMask(notification->captureData, bbox));
      LOG4CXX_TRACE(logger, "created new tracking mask.");
      trackedMO->addNewTrackingResult(trackingMask, trackingConf);
      LOG4CXX_TRACE(logger, "added new tracking mask to tracked object.");
    } else {
      //decreases confidence
      trackedMO->decayTrackingConfidence();
      LOG4CXX_TRACE(logger, boost::format("tracking unsuccessful. confidence: %f.") % trackedMO->getTrackingConfidence());
    }

    //send notifications that trackedMO was updated
    sendTrackingNotifications(trackedMO);
  }

  // update last frame that was processed
  //currFrameGray.copyTo(lastFrameGray);
}

bool CMTTracker::canTrack(const MemoryObject::Ptr &newMemObj) {
  cv::Rect box = newMemObj->getDetectionMask()->getBoundingBox();
  if ((box.width < 5) || (box.height < 5)) {
    LOG4CXX_DEBUG(logger, boost::format("[canTrack] ignoring object with size less than 10x10. object size: %dx%d.") %
                          box.width % box.height);
    return false;
  }

  return true;
}

void CMTTracker::startTracking(const MemoryObject::Ptr &newMemObj) {
  LOG4CXX_TRACE(logger, "[startTracking] method entered.");
  boost::lock_guard<boost::mutex> lock(cmts_mutex);

  //explicitly call base class method
  ObjectTracker::startTracking(newMemObj);

  //instantiate new CMT
  LOG4CXX_DEBUG(logger, boost::format("Adding CMT for tokenID: %d.") % newMemObj->getId());
  cmt::CMT *cmt = new cmt::CMT();
  cmt->consensus.estimate_scale = true;
  cmt->consensus.estimate_rotation = true;

  //init new CMT
  const cv::Mat detectionFrame = newMemObj->getCaptureData()->frame;
  cv::Mat detectionFrameGray;
  cv::cvtColor(detectionFrame, detectionFrameGray, cv::COLOR_RGB2GRAY);

  cv::Rect box = newMemObj->getDetectionMask()->getBoundingBox();
  LOG4CXX_DEBUG(logger, boost::format("frame: %dx%d. box:(%d,%d)%dx%d.")
                        % detectionFrameGray.cols % detectionFrameGray.rows
                        % box.x % box.y % box.width % box.height);
  cmt->initialize(detectionFrameGray, box);

  // attempt to bring new tracker up to date with last used tracking frame (so that all CMTs are "in-step")
  //if (!lastFrameGray.empty()) {
  //  LOG4CXX_DEBUG(logger, boost::format("[startTracking] lastFrameGray: %d x %d. detectionFrameGray: %d x %d.")
  //          % lastFrameGray.rows % lastFrameGray.cols % detectionFrameGray.rows % detectionFrameGray.cols);
  //  cmt.processFrame(lastFrameGray, detectionFrameGray);
  //} else {
  //  // should only happen if no objects have been tracked yet
  //  detectionFrameGray.copyTo(lastFrameGray);
  //}

  //finally, add it to local tracker map
  cmts[newMemObj->getId()] = cmt;
}

void CMTTracker::stopTracking(const MemoryObject::Ptr &existingMemObj) {
  LOG4CXX_TRACE(logger, "[stopTracking] method entered.");
  boost::lock_guard<boost::mutex> lock(cmts_mutex);

  //explicitly call base class method
  ObjectTracker::stopTracking(existingMemObj);

  long long id = existingMemObj->getId();
  CMT_MAP::iterator cmts_itr = cmts.find(id);

  // check if MO is a top-level MO being tracked (or just a child that's not explicitly tracked)
  if (cmts_itr == cmts.end()) {
    LOG4CXX_DEBUG(logger, boost::format("CMT does not exist for tokenID: %d.") % id);
    return;
  } else {
    LOG4CXX_DEBUG(logger, boost::format("Removing CMT for tokenID: %d.") % id);
  }

  //clean up tracker
  delete cmts_itr->second;
  cmts.erase(cmts_itr);
}

void CMTTracker::displayResults(CaptureData::ConstPtr capture) {
  bool showCMTboxes = false;
  if (showCMTboxes) {
    LOG4CXX_TRACE(logger, "[displayResults] CMTTracker method entered.");

    //get image to display
    capture->frame.copyTo(displayFrame);

    // go through all trackers
    boost::lock_guard<boost::mutex> lock(cmts_mutex);
    CMT_MAP::iterator cmts_itr;
    for (cmts_itr = cmts.begin(); cmts_itr != cmts.end(); ++cmts_itr) {
      drawCMTStatus(displayFrame, *(cmts_itr->second));
    }

    diarc::Display::displayFrame(displayFrame, getDisplayName());
  } else {
    ObjectTracker::displayResults(capture);
  }
}

void CMTTracker::drawCMTStatus(cv::Mat& im, cmt::CMT &cmt) {
  //Visualize the output
  //It is ok to draw on im itself, as CMT only uses the grayscale image
  for (size_t i = 0; i < cmt.points_active.size(); i++) {
    cv::circle(im, cmt.points_active[i], 2, cv::Scalar(255, 0, 0));
  }

  cv::Point2f vertices[4];
  cmt.bb_rot.points(vertices);
  for (int i = 0; i < 4; i++) {
    cv::line(im, vertices[i], vertices[(i + 1) % 4], cv::Scalar(255, 0, 0));
  }
}
