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
#include <boost/format.hpp>

using namespace diarc::grasp;

GraspDetector::Ptr GraspDetector::instance = NULL;
boost::mutex GraspDetector::instance_mutex;

GraspDetector::Ptr GraspDetector::getInstance() {
  boost::lock_guard<boost::mutex> lock(instance_mutex);
  if (!instance) {
    instance = GraspDetector::Ptr(new GraspDetector());
  }
  return instance;
}

GraspDetector::GraspDetector() {
  logger = log4cxx::Logger::getLogger("diarc.grasp.GraspDetector");
}

GraspDetector::~GraspDetector() {
}

std::vector<Grasp> GraspDetector::calculateGraspOptions(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                                        const cv::Mat& transform) {
  LOG4CXX_TRACE(logger, "[calculateGraspPoses] method entered.");

  //pcl::ScopeTime t("Grasp Point Detection Time.");

  // if size of object is "small enough" then just make naive grasp calculation for pr2-style gripper
  std::vector<Grasp> grasps;
  // if (smallObjectGrasp->canCalculateGraspPoses(cloud, transform)) {
  //   LOG4CXX_DEBUG(logger, "Calculating small object grasps.");
  //   grasps = smallObjectGrasp->calculateGraspPoses(cloud, transform);
  //   LOG4CXX_DEBUG(logger, boost::format("Detected %d small object grasp options.") % grasps.size());
  // } else {
#if defined(USE_GPD)
    LOG4CXX_DEBUG(logger, "Calculating GPD grasps.");
    grasps = gpdGrasp->calculateGraspPoses(cloud, transform);
    LOG4CXX_DEBUG(logger, boost::format("Detected %d gpd grasp options.") % grasps.size());
#elif defined(USE_AGILEGRASP)
    LOG4CXX_DEBUG(logger, "Calculating agile grasps.");
    grasps = agileGrasp->calculateGraspPoses(cloud, transform);
    LOG4CXX_DEBUG(logger, boost::format("Detected %d agile grasp options.") % grasps.size());
#else
    LOG4CXX_WARN(logger, "No small object grasps found, and Agile Grasp is not available.");
#endif
  // }

  return grasps;
}

void GraspDetector::initialize(const std::string& configFile) {
  LOG4CXX_DEBUG(logger, boost::format("[loadConfig] configFile: %s.") % configFile);

  //get directory
  unsigned found = configFile.find_last_of("/\\"); // check if config file is filename or path
  std::string dir = configFile.substr(0, found + 1);

  Json::CharReaderBuilder builder;
  Json::Value root;
  JSONCPP_STRING errs;
  std::ifstream fileStream(configFile, std::ifstream::binary);
  if (!Json::parseFromStream(builder, fileStream, &root, &errs)) {
    LOG4CXX_ERROR(logger, boost::format("Error parsing json file: %s") % configFile.c_str());
  }

  // grasp options
  for (auto graspInfo: root["grasps"]) {
    std::string name = graspInfo["name"].asString();
    int priority = graspInfo["priority"].asInt();
    std::string configFile = dir + graspInfo.get("config", "").asString();

    if (name == "small_object") {
      smallObjectGrasp = SmallObjectGrasp::Ptr(new SmallObjectGrasp(configFile));
    } else if (name == "agile") {
#if defined(USE_AGILEGRASP)
      agileGrasp = AgileGrasp::Ptr(new AgileGrasp(configFile));
#endif
    } else if (name == "gpd") {
#if defined(USE_GPD)
      gpdGrasp = GpdGrasp::Ptr(new GpdGrasp(configFile));
#endif
    } else {
      LOG4CXX_ERROR(logger, boost::format("Unknown grasp type: %s") % name.c_str());
    }
  }
}
