/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

//
// Created by evan on 2/25/20.
//

#ifndef MOTIONDETECTOR_HPP
#define MOTIONDETECTOR_HPP

#include "ObjectDetector.hpp"

class MotionDetector : public ObjectDetector {
 public:
  typedef boost::shared_ptr<MotionDetector> Ptr;
  typedef boost::shared_ptr<const MotionDetector> ConstPtr;

  MotionDetector(const long long& processorId, const int imgWidth, const int imgHeight);
  ~MotionDetector();

  //! override because this detector never wants capture notifications (uses motion notifications)
  virtual void registerForCaptureNotification();

  //! override because this detector never wants capture notifications (uses motion notifications)
  virtual void unregisterForCaptureNotification();

 protected:
  virtual void handleMotionNotification(MotionNotification::ConstPtr notification);

 private:
};


#endif  //MOTIONDETECTOR_HPP