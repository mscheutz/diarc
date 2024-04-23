/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

// class to calculate relative surface
// author: ep

#ifndef RELATIVEHEIGHTPROCESSOR_HPP
#define RELATIVEHEIGHTPROCESSOR_HPP

#include "SaliencyProcessor.hpp"
#include <v4r/AttentionModule/AttentionModule.hpp>
#include <v4r/EPUtils/EPUtils.hpp>

//services
#include "imgproc/utilsDetect/PlaneDetectionProcessor.hpp"

class RelativeHeightProcessor : public SaliencyProcessor {
public:
  typedef boost::shared_ptr<RelativeHeightProcessor> Ptr;
  typedef boost::shared_ptr<const RelativeHeightProcessor> ConstPtr;

  RelativeHeightProcessor(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight, const bool isStereo);
  ~RelativeHeightProcessor();

protected:
  virtual void handlePlaneNotification(PlaneNotification::ConstPtr notification);

private:
  int mapConfig(std::string &height);

  AttentionModule::SurfaceHeightSaliencyMap surfaceHeightSaliencyMap;
};

#endif  //RELATIVEHEIGHTPROCESSOR_HPP
