/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef YOLODETECTOR_HPP
#define YOLODETECTOR_HPP

#include "NeuralDetector.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>
#include <unordered_map>

class YoloDetector : public NeuralDetector {
public:
  YoloDetector(const long long& processorId, const int imgWidth, const int imgHeight);
  virtual ~YoloDetector();
protected:
};

#endif  //YOLODETECTOR_HPP