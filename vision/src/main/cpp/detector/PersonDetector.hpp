/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef PERSONDETECTOR_HPP
#define PERSONDETECTOR_HPP

#include "NeuralDetector.hpp"

class PersonDetector : public NeuralDetector {
  public:
    PersonDetector(const long long &processorId, const int imgWidth, const int imgHeight);
    ~PersonDetector();
  protected:
    virtual void handleCaptureNotification(CaptureNotification::ConstPtr notification);
  private:
};
#endif