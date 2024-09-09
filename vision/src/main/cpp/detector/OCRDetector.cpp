/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "OCRDetector.hpp"

OCRDetector::OCRDetector(const long long &processorId, const int imgWidth, const int imgHeight)
        : NeuralDetector(processorId, imgWidth, imgHeight)
{
  visionProcessName = "OCRDetector";
  logger = log4cxx::Logger::getLogger("diarc.detector.OCRDetector");
}

OCRDetector::~OCRDetector() {}
