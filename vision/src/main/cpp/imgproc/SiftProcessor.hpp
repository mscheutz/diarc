/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef SIFTPROCESSOR_HPP
#define SIFTPROCESSOR_HPP

#include "ImageProcessor.hpp"

#include <boost/thread/shared_mutex.hpp>
#include "siftFeatures/SiftFeatures.hpp"              /* sift feature detection/extraction and display of keypoints */

class SiftProcessor : public ImageProcessor {
public:
  typedef boost::shared_ptr<SiftProcessor> Ptr;
  typedef boost::shared_ptr<const SiftProcessor> ConstPtr;

  SiftProcessor(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight, const bool isStereo);
  virtual ~SiftProcessor();

  virtual void cleanup();
  
protected:
  virtual void handleCaptureNotification(CaptureNotification::ConstPtr notification);

private:
  SiftFeatureExtractor* siftHelper;
};

#endif  //SIFTPROCESSOR_HPP