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

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <log4cxx/logger.h>

namespace diarc {
  namespace grasp {

    class GraspDetector {
    public:
      typedef boost::shared_ptr<GraspDetector> Ptr;
      typedef boost::shared_ptr<const GraspDetector> ConstPtr;

      static GraspDetector::Ptr getInstance();
      ~GraspDetector();

      void loadConfig(const std::string& configFile);
      std::vector<diarc::grasp::Grasp> calculateGraspOptions(diarc::stm::MemoryObject::Ptr &object);

    protected:
      GraspDetector();

      static boost::mutex instance_mutex;
      static GraspDetector::Ptr instance;
      log4cxx::LoggerPtr logger;

    private:
      typedef pcl::PointXYZ PointType;
      SmallObjectGrasp::Ptr smallObjectGrasp;
#ifdef USE_AGILEGRASP
      AgileGrasp::Ptr agileGrasp;
#endif
    };
  }
}
#endif // GRASPDETECTOR_HPP
