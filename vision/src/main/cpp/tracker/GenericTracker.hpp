/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef GENERICTRACKER_HPP
#define GENERICTRACKER_HPP

#include "ObjectTracker.hpp"

/* Generic Tracker implementation that does naive rudimentary matching as a
 * way to "track" memory objects between frames.
 */

class GenericTracker : public ObjectTracker {
public:
  GenericTracker(const long long &processorId, const int imgWidth, const int imgHeight);

  virtual ~GenericTracker();

protected:
  virtual void haveNewImage(CaptureNotification::ConstPtr notification);

  virtual void removeLowConfidenceObjects();

  virtual int mergeObject(diarc::stm::MemoryObject::Ptr newObject);

private:

  bool compare(const diarc::stm::MemoryObject::Ptr &mo1, const diarc::stm::MemoryObject::Ptr &mo2);

  bool shouldBeTracked(const diarc::stm::MemoryObject::Ptr &mo);
};

#endif  //GENERICTRACKER_HPP
