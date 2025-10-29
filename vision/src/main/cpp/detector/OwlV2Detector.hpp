/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
*/

#ifndef OWLV2DETECTOR_HPP
#define OWLV2DETECTOR_HPP

#include "ObjectDetector.hpp"
#include "stm/MemoryObject.hpp"

#include <pybind11/pytypes.h>

class OwlV2Detector : public ObjectDetector {
public:
  typedef boost::shared_ptr<OwlV2Detector> Ptr;
  typedef boost::shared_ptr<const OwlV2Detector> ConstPtr;

  OwlV2Detector(const long long& processorId, const int imgWidth, const int imgHeight);
  ~OwlV2Detector() override;

  void init() override;
  void cleanup() override;

protected:
  void handleCaptureNotification(CaptureNotification::ConstPtr notification) override;

private:
  void createDetector();
  void destroyDetector();

  bool detector_initialized;
  pybind11::object OwlV2DetectorClass;
  pybind11::object owlV2Detector;

  diarc::stm::MemoryObject::VecPtr createMemoryObjects(pybind11::list masks, pybind11::list confs, CaptureData::ConstPtr captureData);
};

#endif  //OWLV2DETECTOR_HPP
