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

#include "Grasp.hpp"
#include "SmallObjectGrasp.hpp"
#ifdef USE_AGILEGRASP
#include "AgileGrasp.hpp"
#endif

#include <log4cxx/logger.h>

namespace diarc {
  namespace grasp {

    class GraspDetector {
      typedef pcl::PointXYZ PointType;

    public:
      GraspDetector();
      ~GraspDetector();

      void loadConfig(const std::string& configFile);

    protected:
      std::vector<Grasp> calculateGraspPoses(diarc::stm::MemoryObject::Ptr &object);

      log4cxx::LoggerPtr logger;

    private:
      SmallObjectGrasp::Ptr smallObjectGrasp;
#ifdef USE_AGILEGRASP
      AgileGrasp::Ptr agileGrasp;
#endif
    };
  }
}
#endif // GRASPDETECTOR_HPP
