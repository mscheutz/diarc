/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef RELATIONTRACKER_HPP
#define RELATIONTRACKER_HPP

#include "ObjectTracker.hpp"

/* RelationTracker implementation that does not actual visual tracking but instead
* manages a MemoryObject's relations (decaying the confidence if it's not continually re-detected
* and removing them) that has been detected to have a specified relation
* and exposes that MemoryObject via TrackedObjects as the relation's typeId.
 */

class RelationTracker : public ObjectTracker {
public:
  RelationTracker(const long long &processorId, const int imgWidth, const int imgHeight);

  ~RelationTracker();

protected:

  virtual void haveNewImage(CaptureNotification::ConstPtr notification);

  virtual void haveTrackedMemoryObject(diarc::stm::MemoryObject::Ptr trackedObject);

  virtual int mergeObject(diarc::stm::MemoryObject::Ptr newlyDetectedObject);

  virtual void removeLowConfidenceObjects();

private:
};

#endif  //RELATIONTRACKER_HPP
