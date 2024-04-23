/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef OCRDETECTOR_HPP
#define OCRDETECTOR_HPP

#include "NeuralDetector.hpp"

class OCRDetector : public NeuralDetector {
  public:
    OCRDetector(const long long &processorId, const int imgWidth, const int imgHeight);
    ~OCRDetector();
  protected:
  private:
};
#endif