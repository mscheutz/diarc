/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef COLORBASEDDETECTOR_HPP
#define COLORBASEDDETECTOR_HPP

#include "ObjectDetector.hpp"
#include "colorbased/ColorRange.hpp"

class ColorBasedDetector : public ObjectDetector {
 public:
  typedef boost::shared_ptr<ColorBasedDetector> Ptr;
  typedef boost::shared_ptr<const ColorBasedDetector> ConstPtr;

  ColorBasedDetector(const long long &processorId, const int imgWidth, const int imgHeight);
  ~ColorBasedDetector();

  virtual void loadConfig(const std::string &config);

 protected:
  virtual void handleCaptureNotification(CaptureNotification::ConstPtr notification);
  virtual void handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification);

 private:
  diarc::stm::MemoryObject::VecPtr detectObjects(const cv::Mat &currFrame, CaptureData::ConstPtr capture);

  std::vector<diarc::stm::MemoryObject::Ptr> getMemoryObjects(const cv::Mat &currFrame,
                                                       CaptureData::ConstPtr capture,
                                                       const PredicateHelper &descriptor,
                                                       const std::tr1::unordered_set<long long> &typeIds,
                                                       const std::vector<ColorRange> &colorRanges);

  cv::Mat getColorMask(const cv::Mat &currFrame, const std::vector<ColorRange> &colorRanges);

  /**
   * Map from color label to color ranges.
   */
  std::map<std::string, std::vector<ColorRange> > colorMap;
  /**
   * Set of color supertypes that indicates to search for all color ranges.
   * For instance, a search for supertype "block" would mean to search for all pre-defined color ranges (in colorMap)
   * and each result would have a label of supertype and color (e.g., "block(X)" and "red(X)").
   */
  std::set<std::string> superTypes;
  /**
   * Map from string to opencv colorspace enums. Used for parsing config file.
   */
  std::unordered_map<std::string, cv::ColorConversionCodes> colorSpaceMap;
};


#endif  //COLORBASEDDETECTOR_HPP