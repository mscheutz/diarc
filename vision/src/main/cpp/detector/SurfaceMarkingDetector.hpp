/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * Detect markings (e.g., red cross) on a surface.
 *
 * @author Michael Zillich
 * @date August, 2013
 */

#ifndef SURFACE_MARKING_DETECTOR
#define SURFACE_MARKING_DETECTOR

#include "ObjectDetector.hpp"

#include <string>
#include <log4cxx/logger.h>
#include "shape_match/shapecontext.h"

class SurfaceMarkingDetector : public ObjectDetector {
public:
  typedef boost::shared_ptr<SurfaceMarkingDetector> Ptr;
  typedef boost::shared_ptr<const SurfaceMarkingDetector> ConstPtr;

  SurfaceMarkingDetector(const long long& processorId, const unsigned int imgWidth,
          const unsigned int imgHeight);

  virtual void loadConfig(const std::string& config);

protected:
  virtual void handleCaptureNotification(CaptureNotification::ConstPtr capture);

private:
  ade::stm::MemoryObject::Ptr createMemoryObject(const cv::Rect& rect,
          const std::string& label, const float& confidence,
          const boost::unordered_set<long long>& typeIds,
          CaptureData::ConstPtr captureData);

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

  std::map<std::string, ShapeModel> surfaceMarkingMap;
};

#endif
