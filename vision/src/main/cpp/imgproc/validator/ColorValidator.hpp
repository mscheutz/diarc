/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * Class for color validation
 *
 * @author Ekaterina Potapova (EP)
 * @date Nov 2012
 */

#ifndef COLORVALIDATOR_HPP
#define COLORVALIDATOR_HPP

#include "ObjectValidator.hpp"

#include <map>

class ColorValidator : public ObjectValidator {
public:
  typedef boost::shared_ptr<ColorValidator> Ptr;
  typedef boost::shared_ptr<const ColorValidator> ConstPtr;

  ColorValidator(const long long& processorId, const unsigned int imgWidth,
          const unsigned int imgHeight, const bool isStereo);
  ~ColorValidator();

  virtual void loadConfig(const std::string& config);

protected:
  virtual void handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification);

private:
  std::map<std::string, cv::Scalar> colorMap;

  bool calculateColor(const cv::Mat& image, const std::vector<int>& indices, const cv::Scalar& color, float &confidence, cv::Mat_<float>& imageMask);
};

#endif  //COLORVALIDATOR_HPP
