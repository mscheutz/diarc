/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   OpenCVTracker.hpp
 * Author: Evan Krause
 *
 * Created on May 15 2023
 *
 */

#ifndef OPENCVTRACKER_HPP
#define  OPENCVTRACKER_HPP

#include "ObjectTracker.hpp"
#include <opencv2/tracking.hpp>

#if (CV_MAJOR_VERSION >= 4 && CV_MINOR_VERSION >= 5)
#include <opencv2/tracking/tracking_legacy.hpp>
#endif

class OpenCVTracker : public ObjectTracker {
public:
  OpenCVTracker(const std::string &trackerType, const long long &processorId, const int imgWidth, const int imgHeight);
  virtual ~OpenCVTracker();

  void loadConfig(const std::string &config);

protected:
  virtual void haveNewImage(CaptureNotification::ConstPtr notification);

private:
  virtual bool canTrack(const diarc::stm::MemoryObject::Ptr &newMemObj);
  virtual void startTracking(const diarc::stm::MemoryObject::Ptr &newMemObj);
  virtual void stopTracking(const diarc::stm::MemoryObject::Ptr &existingMemObj);

  //! tracker type
  std::string type;

  //! typeId to tracker instance map
#if (CV_MAJOR_VERSION >= 4 && CV_MINOR_VERSION >= 5)
  std::tr1::unordered_map<long long, cv::Ptr<cv::legacy::Tracker>> trackers;
#else
  std::tr1::unordered_map<long long, cv::Ptr<cv::Tracker>> trackers;
#endif

  //! lock to access trackers
  boost::mutex trackers_mutex;
};

#endif  /* OPENCVTRACKER_HPP */

