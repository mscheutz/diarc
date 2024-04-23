/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

// class to calculate orientation saliency
// author: ep

#ifndef ORIENTATIONPROCESSOR_HPP
#define ORIENTATIONPROCESSOR_HPP

#include "SaliencyProcessor.hpp"
#include <v4r/AttentionModule/AttentionModule.hpp>
#include <v4r/EPUtils/EPUtils.hpp>

class OrientationProcessor : public SaliencyProcessor {
public:
  OrientationProcessor(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight, const bool isStereo);
  ~OrientationProcessor();

protected:
  virtual void handleCaptureNotification(CaptureNotification::ConstPtr notification);

private:
  int mapConfig(const std::string &orientation);
  AttentionModule::OrientationSaliencyMap orientationSaliencyMap;

};

typedef boost::shared_ptr<OrientationProcessor> OrientationProcessorPtr;

#endif  //ORIENTATIONPROCESSOR_HPP
