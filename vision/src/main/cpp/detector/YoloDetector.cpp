/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "YoloDetector.hpp"

#include "capture/util/CaptureUtilities.hpp"

using namespace ade::stm;

YoloDetector::YoloDetector(const long long &processorId, const int imgWidth, const int imgHeight)
        : NeuralDetector(processorId, imgWidth, imgHeight) {
  visionProcessName = "YoloDetector";
  logger = log4cxx::Logger::getLogger("ade.detector.YoloDetector");
  confidence_thresh = 0.1;
  nms_thresh = 0.4;
}

YoloDetector::~YoloDetector() {}
