/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "ElevatorDoorDetector.hpp"

ElevatorDoorDetector::ElevatorDoorDetector(const long long &processorId, const int imgWidth, const int imgHeight)
        : NeuralDetector(processorId, imgWidth, imgHeight)
{
  visionProcessName = "ElevatorDoorDetector";
  logger = log4cxx::Logger::getLogger("diarc.detector.ElevatorDoorDetector");
}

ElevatorDoorDetector::~ElevatorDoorDetector() {}
