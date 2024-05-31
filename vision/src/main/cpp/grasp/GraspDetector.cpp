
/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * Class for processing a pointcloud and find the valid grasps for an item
 *
 * @author HRI-Lab at Tufts and NEU agile_grasp team
 * @date October 5, 2015
 */

#include "GraspDetector.hpp"
#include "display/Display.hpp"
#include "point_clouds/PointCloudUtilities.hpp"
#include "Grasp.hpp"

#include <jsoncpp/json/reader.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

using namespace diarc::stm;
using namespace diarc::grasp;

GraspDetector::Ptr GraspDetector::getInstance() {
  boost::lock_guard<boost::mutex> lock(instance_mutex);
  if (!instance) {
    instance = GraspDetector::Ptr(new GraspDetector());
  }
   return instance;
}

GraspDetector::GraspDetector() {
  logger = log4cxx::Logger::getLogger("diarc.detector.GraspDetector");

  smallObjectGrasp = SmallObjectGrasp::Ptr(new SmallObjectGrasp());
#if defined(USE_AGILEGRASP)
  agileGrasp = AgileGrasp::Ptr(new AgileGrasp());
#endif
}

GraspDetector::~GraspDetector() {
}

std::vector<Grasp> GraspDetector::calculateGraspOptions(diarc::stm::MemoryObject::Ptr &object) {
  LOG4CXX_TRACE(logger, "[calculateGraspPoses] method entered.");

  //pcl::ScopeTime t("Grasp Point Detection Time.");

  //create a PointCloud object_cloud and fill it in with data
  pcl::PointCloud<PointType>::Ptr object_cloud(new pcl::PointCloud<PointType>);
  pcl::copyPointCloud(*(object->getDetectionMask()->getObjectPointCloud()), *object_cloud);

  // if size of object is "small enough" then just make naive grasp calculation for pr2-style gripper
  std::vector<Grasp> grasps;
  if (smallObjectGrasp->canCalculateGraspPoses(object)) {
    LOG4CXX_DEBUG(logger, "Calculating small object grasps.");
    grasps = smallObjectGrasp->calculateGraspPoses(object);
    LOG4CXX_DEBUG(logger, boost::format("Detected %d small object grasp options.") % grasps.size());
  } else {
    #if defined(USE_AGILEGRASP)
      LOG4CXX_DEBUG(logger, "Calculating agile grasps.");
      grasps = agileGrasp->calculateGraspPoses(object);
      LOG4CXX_DEBUG(logger, boost::format("Detected %d agile grasp options.") % grasps.size());
    #else
      LOG4CXX_WARN(logger, "No small object grasps found, and Agile Grasp is not available.");
    #endif
  }

  return grasps;
}

void GraspDetector::loadConfig(const std::string& configFile) {
  LOG4CXX_TRACE(logger, "[loadConfig] method entered.");

  //get directory
  unsigned found = configFile.find_last_of("/\\"); // check if config file is filename or path
  std::string dir = configFile.substr(0, found + 1);
  std::string model_file;
  std::string backEnd;
  std::string type;
  uint64_t totalClasses;

  Json::CharReaderBuilder builder;
  Json::Value root;
  JSONCPP_STRING errs;
  std::ifstream fileStream(configFile, std::ifstream::binary);
  if (!Json::parseFromStream(builder, fileStream, &root, &errs)) {
    LOG4CXX_ERROR(logger, boost::format("Error parsing json file: %s") % configFile.c_str());
    return;
  }
}


