/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

// class to location saliency
// author: ep

#ifndef LOCATIONSALIENCYPROCESSOR_HPP
#define LOCATIONSALIENCYPROCESSOR_HPP

#include "SaliencyProcessor.hpp"
#include <v4r/AttentionModule/AttentionModule.hpp>
#include <v4r/EPUtils/EPUtils.hpp>

#include <math.h>

class LocationSaliencyProcessor : public SaliencyProcessor {
public:
  typedef boost::shared_ptr<LocationSaliencyProcessor> Ptr;
  typedef boost::shared_ptr<LocationSaliencyProcessor> ConstPtr;

  LocationSaliencyProcessor(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight, const bool isStereo);

protected:
  virtual void handleCaptureNotification(CaptureNotification::ConstPtr notification);

private:
  int mapConfig(const std::string &location);
  AttentionModule::LocationSaliencyMap locationSaliencyMap;

};

#endif  //LOCATIONSALIENCYPROCESSOR_HPP
