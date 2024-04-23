/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef ELEVATORDOORDETECTOR_HPP
#define ELEVATORDOORDETECTOR_HPP

#include "NeuralDetector.hpp"

class ElevatorDoorDetector : public NeuralDetector {
  public:
    ElevatorDoorDetector(const long long &processorId, const int imgWidth, const int imgHeight);
    ~ElevatorDoorDetector();
  protected:
  private:
};
#endif