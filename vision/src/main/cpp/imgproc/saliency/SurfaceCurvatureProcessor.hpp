/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

// class to calculate surface curvature
// author: ep

#ifndef SURFACECURVATURE_HPP
#define SURFACECURVATURE_HPP

#include "SaliencyProcessor.hpp"
#include <v4r/AttentionModule/AttentionModule.hpp>
#include <v4r/EPUtils/EPUtils.hpp>

//services
#include "imgproc/utilsDetect/PlaneDetectionProcessor.hpp"

class SurfaceCurvatureProcessor : public SaliencyProcessor {
public:
  typedef boost::shared_ptr<SurfaceCurvatureProcessor> Ptr;
  typedef boost::shared_ptr<const SurfaceCurvatureProcessor> ConstPtr;

  SurfaceCurvatureProcessor(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight, const bool isStereo);
  ~SurfaceCurvatureProcessor();

protected:
  virtual void handlePlaneNotification(PlaneNotification::ConstPtr notification);

private:
  int mapConfig(std::string &curvature);
  bool haveNewImage(ExtractedPlane::Ptr plane);
};

#endif  //SURFACECURVATURE_HPP
