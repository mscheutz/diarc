/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

// class to calculate plane
// author: EP
// date September 2012

#ifndef PLANEDETECTIONPROCESSOR_HPP
#define PLANEDETECTIONPROCESSOR_HPP

#include "common/notification/CaptureNotification.hpp"
#include "imgproc/ImageProcessor.hpp"

class PlaneProcessor : public ImageProcessor {
public:
  typedef boost::shared_ptr<PlaneProcessor> Ptr;
  typedef boost::shared_ptr<const PlaneProcessor> ConstPtr;

  PlaneProcessor(const long long& processorId, const unsigned int imgWidth,
          const unsigned int imgHeight, const bool isStereo);
  ~PlaneProcessor();
  
  virtual void loadConfig(const std::string& config);
  
protected:
  virtual void handleCaptureNotification(CaptureNotification::ConstPtr notification);

 private:
  bool shouldCalculatePlane(const cv::Mat &newTransform);

  cv::Mat lastTransform;
  pcl::ModelCoefficients::Ptr lastPlaneCoefficients;
  float table_height;
  float dist_threshold;
  /**
   * Set to true to override shouldCalculatePlane, which will recalculate plane every iteration.
   */
  bool alwaysCalcPlane;
};

#endif  //PLANEDETECTIONPROCESSOR_HPP