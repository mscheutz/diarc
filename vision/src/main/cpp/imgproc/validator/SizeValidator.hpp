/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * Class for size validation
 *
 * @author Evan Krause
 * @date May 2022
 */

#ifndef SIZEVALIDATOR_HPP
#define SIZEVALIDATOR_HPP

#include "ObjectValidator.hpp"

#include <map>

class SizeValidator : public ObjectValidator {
public:
  typedef boost::shared_ptr<SizeValidator> Ptr;
  typedef boost::shared_ptr<const SizeValidator> ConstPtr;

  SizeValidator(const long long& processorId, const unsigned int imgWidth,
          const unsigned int imgHeight, const bool isStereo);
  ~SizeValidator();

  virtual void loadConfig(const std::string& config);

protected:
  virtual void handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification);

private:
  std::map<std::string, std::vector<double>> sizeMap;

  bool isInSizeRange(const std::vector<double>& targetSize, const cv::Point3d& bb);
};

#endif  //SIZEVALIDATOR_HPP
