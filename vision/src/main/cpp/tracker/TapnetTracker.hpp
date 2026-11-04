/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef TAPNETTRACKER_HPP
#define	TAPNETTRACKER_HPP

#include "ObjectTracker.hpp"
#include <pybind11/pytypes.h>

class TapnetTracker : public ObjectTracker
{
public:
  TapnetTracker(const long long& processorId, const int imgWidth, const int imgHeight);
  ~TapnetTracker();
  
  void loadConfig(const std::string& config);
  
protected:
    virtual void haveNewImage(CaptureNotification::ConstPtr notification);

private:
  virtual bool canTrack(const diarc::stm::MemoryObject::Ptr& newMemObj);
  virtual void startTracking(const diarc::stm::MemoryObject::Ptr& newMemObj);
  virtual void stopTracking(const diarc::stm::MemoryObject::Ptr& existingMemObj);
  virtual void displayResults(CaptureData::ConstPtr capture);

  void createTracker();
  void destroyTracker();

  bool tracker_initialized;
  pybind11::object TapnetClass;
  pybind11::object tapnet;

  std::tr1::unordered_map<long long, int> trackerMap;
};


#endif	/* TAPNETTRACKER_HPP */

