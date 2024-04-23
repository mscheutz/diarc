/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * Class for processing a pointcloud and find the valid grasps for an item
 *
 * @author HRI-Lab at Tufts and NEU agile_grasp team
 * @date October 5, 2015
 */

#ifndef GRASPDETECTOR_HPP
#define GRASPDETECTOR_HPP

#include "ObjectDetector.hpp"
#include "grasp/SmallObjectGrasp.hpp"
#include "grasp/AgileGrasp.hpp"

class GraspDetector : public ObjectDetector {
  typedef pcl::PointXYZ PointType;

public:
  GraspDetector(const long long &processorId, const unsigned int imgWidth, const unsigned int imgHeight);
  ~GraspDetector();

  virtual void loadConfig(const std::string& config);

protected:
  virtual void handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification);

private:
  SmallObjectGrasp smallObjectGrasp;
  AgileGrasp agileGrasp;
};

#endif // GRASPDETECTOR_HPP
