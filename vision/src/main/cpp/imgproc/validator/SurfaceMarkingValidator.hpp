/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * Detect markings (e.g., red cross) on a surface.
 *
 * @author Michael Zillich
 * @date August, 2013
 */

#ifndef SURFACE_MARKING_VALIDATOR
#define SURFACE_MARKING_VALIDATOR

#include "ObjectValidator.hpp"

#include <string>
#include <log4cxx/logger.h>
#include "shape_match/shapecontext.h"

class SurfaceMarkingValidator : public ObjectValidator {
public:
  SurfaceMarkingValidator(const long long& processorId, const unsigned int imgWidth,
        const unsigned int imgHeight, const bool isStereo);
  /**
   * Learn a surface marking from an image.
   * @param model  shape model that is added to
   * @param img  the image to learn from
   */
  void learn(ShapeModel &model, const cv::Mat &img);
  /**
   * Detect a given surface marking in an image.
   * @param model the surface marking to detect in the image
   * @param img the image to search
   * @return true if match is found
   */
  bool detect(ShapeModel &model, const cv::Mat &img, float &confidence);
  
  virtual void loadConfig(const std::string& config);
  
protected:
  virtual void handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification);
  
private:
  std::map<std::string, ShapeModel> surfaceMarkingMap;
};

#endif //SURFACE_MARKING_VALIDATOR
