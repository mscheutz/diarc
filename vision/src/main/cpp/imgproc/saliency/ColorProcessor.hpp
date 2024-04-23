/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

// class to calculate color saliency
// author: ep

#ifndef COLORPROCESSOR_HPP
#define COLORPROCESSOR_HPP

#include "SaliencyProcessor.hpp"
#include <v4r/AttentionModule/AttentionModule.hpp>
#include <v4r/EPUtils/EPUtils.hpp>

#include <map>

class ColorProcessor : public SaliencyProcessor {
public:
  typedef boost::shared_ptr<ColorProcessor> Ptr;
  typedef boost::shared_ptr<const ColorProcessor> ConstPtr;

  ColorProcessor(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight, const bool isStereo);
  ~ColorProcessor();

  virtual void loadConfig(const std::string& config);

protected:
  virtual void handleCaptureNotification(CaptureNotification::ConstPtr notification);

private:
  std::map<std::string, cv::Scalar> colorMap;
  AttentionModule::ColorSaliencyMap colorSaliencyMap;
};

#endif  //COLORPROCESSOR_HPP
