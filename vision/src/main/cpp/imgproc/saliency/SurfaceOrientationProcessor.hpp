/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

// class to calculate relative surface
// author: ep

#ifndef SURFACEORIENTATIONPROCESSOR_HPP
#define SURFACEORIENTATIONPROCESSOR_HPP

#include "SaliencyProcessor.hpp"
#include <v4r/AttentionModule/AttentionModule.hpp>
#include <v4r/EPUtils/EPUtils.hpp>

//services
#include "imgproc/utilsDetect/PlaneDetectionProcessor.hpp"

class SurfaceOrientationProcessor : public SaliencyProcessor {
public:
  typedef boost::shared_ptr<SurfaceOrientationProcessor> Ptr;
  typedef boost::shared_ptr<const SurfaceOrientationProcessor> ConstPtr;

  SurfaceOrientationProcessor(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight, const bool isStereo);
  ~SurfaceOrientationProcessor();

protected:
  virtual void handlePlaneNotification(PlaneNotification::ConstPtr notification);

private:
  int mapConfig(std::string &orientation);
  bool haveNewImage(ExtractedPlane::Ptr plane);
};

#endif  //RELATIVEHEIGHTPROCESSOR_HPP
