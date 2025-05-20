/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef FASTSAMDETECTOR_HPP
#define FASTSAMDETECTOR_HPP

#include <boost/python/numpy.hpp>

#include "ObjectDetector.hpp"
#include "stm/MemoryObject.hpp"


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
  // boost::python::object convertCvMatToPyMat(cv::Mat& cvMat);
  // cv::Mat convertPyMatToCvMat(boost::python::object& pyMat);

  boost::python::numpy::ndarray convertMatToNDArray(const cv::Mat& mat);
  cv::Mat convertNDArrayToMat(const boost::python::numpy::ndarray& ndarr);
};

#endif  //FASTSAMDETECTOR_HPP
