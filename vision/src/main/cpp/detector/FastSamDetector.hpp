/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
*/

#ifndef FASTSAMDETECTOR_HPP
#define FASTSAMDETECTOR_HPP

#include "ObjectDetector.hpp"
#include "stm/MemoryObject.hpp"

#include <pybind11/pytypes.h>


class FastSamDetector : public ObjectDetector {
public:
  typedef boost::shared_ptr<FastSamDetector> Ptr;
  typedef boost::shared_ptr<const FastSamDetector> ConstPtr;

  FastSamDetector(const long long& processorId, const int imgWidth, const int imgHeight);
  ~FastSamDetector() override;

  void init() override;
  void cleanup() override;

protected:
  void handleCaptureNotification(CaptureNotification::ConstPtr notification) override;
  void handleSaliencyNotification(SaliencyNotification::ConstPtr notification) override;

private:
  cv::Mat salmap;
  pybind11::object Sam2DSegmenter;
  pybind11::object segmenter;
};

#endif  //FASTSAMDETECTOR_HPP
