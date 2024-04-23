
/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * Class for processing a pointcloud and find the valid grasps for an item
 *
 * @author HRI-Lab at Tufts and NEU agile_grasp team
 * @date October 5, 2015
 */

//PointType is a pcl::PointXYZ

#include "GraspDetector.hpp"
#include "stm/GraspValidationResult.hpp"
#include "display/Display.hpp"
#include "point_clouds/PointCloudUtilities.hpp"
#include "grasp/GraspPose.hpp"
#include <pcl/common/time.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

//includes for parsing XML
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

using namespace ade::stm;

GraspDetector::GraspDetector(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight)
: ObjectDetector(processorId, imgWidth, imgHeight) {

  visionProcessName = "GraspDetector";
  logger = log4cxx::Logger::getLogger("ade.detector.GraspDetector");
}

GraspDetector::~GraspDetector() {
}

void GraspDetector::handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification) {
  LOG4CXX_TRACE(logger, "[handleMemoryObjectNotification] method entered.");

  //pcl::ScopeTime t("Grasp Point Detection Time.");

  MemoryObject::Ptr object = notification->object;

  // if not supposed to be processing object's typeId
  DescriptorsByTypeConstPtr types = getTypes();
  DescriptorsByType::const_iterator types_itr;
  types_itr = types->find(object->getTypeId());
  if (types_itr == types->end()) {
    LOG4CXX_TRACE(logger, "[handleMemoryObjectNotification] exiting in types_itr loop.");
    return;
  }

  //create a PointCloud object_cloud and fill it in with data
  pcl::PointCloud<PointType>::Ptr object_cloud(new pcl::PointCloud<PointType>);
  pcl::copyPointCloud(*(object->getDetectionMask()->getObjectPointCloud()), *object_cloud);

  // if size of object is "small enough" then just make naive grasp calculation for pr2-style gripper
  std::vector<GraspPose> grasps;
  if (smallObjectGrasp.canCalculateGraspPoses(object)) {
    LOG4CXX_DEBUG(logger, "Calculating small object grasps.");
    grasps = smallObjectGrasp.calculateGraspPoses(object);
  } else {
    LOG4CXX_DEBUG(logger, "Calculating agile grasps.");
    grasps = agileGrasp.calculateGraspPoses(object);
  }

  // add all grasps to memory object

  // set confidence and descriptors for search typeids
  float confidence = 1.0f; //TODO: set this properly

  if (logger->isDebugEnabled()) {
    LOG4CXX_DEBUG(logger, boost::format("TypeId: %ld.") % types_itr->first);
    PredicateHelper::Set::const_iterator descriptor_itr;
    for (descriptor_itr = types_itr->second.begin(); descriptor_itr != types_itr->second.end(); ++descriptor_itr) {
      LOG4CXX_DEBUG(logger, boost::format("Descriptor: %s.") % descriptor_itr->toString());
    }
  }
  PredicateHelper::Set::const_iterator descriptor_itr = types_itr->second.begin();

  GraspValidationResult::Ptr graspValidation;
  for (const auto& grasp : grasps) {

    graspValidation = GraspValidationResult::Ptr(new GraspValidationResult(
        confidence, *descriptor_itr, object->getCaptureData(),
        grasp.points, grasp.orientation));

    MemoryObject::Ptr graspMO(new MemoryObject(types_itr->first, descriptor_itr->getArg(0), object->getCaptureData(), graspValidation));
    graspMO->addValidationResult(graspValidation);

    object->addChild(graspMO);
  }

  //pass along notification to the system
  LOG4CXX_DEBUG(logger, "Sending notifications.");
  sendDetectionNotifications(object);

  if (getDisplayFlag()) {
    displayMemoryObject(object, descriptor_itr->getArg(0), true);
  }
}

void GraspDetector::loadConfig(const std::string & config) {
  LOG4CXX_TRACE(logger, "[loadConfig] method entered.");

  //XML parameters look into vision/native/data/detectors/grasp/grasp.xml

  //XML steps to read it
  unsigned position = config.find_last_of("/\\");
  std::string dir = config.substr(0, position + 1);

  //populate tree structure pt
  using boost::property_tree::ptree;
  ptree pt;
  read_xml(config, pt);

  std::string predicateName;

  //parse tree

  BOOST_FOREACH(ptree::value_type const& predicateNode, pt.get_child("processor")) {
    if (predicateNode.first.compare("predicate") == 0) {
      LOG4CXX_DEBUG(logger, boost::format("[loadConfig] name:%s.") % predicateNode.first);

      predicateName = predicateNode.second.get<std::string> ("<xmlattr>.name", "unknown");
      LOG4CXX_DEBUG(logger, boost::format("[loadConfig] predicateName: %s.") % predicateName);

    } else if (predicateNode.first.compare("config") == 0) {
      std::string openposeConfigFile =
          dir + predicateNode.second.get<std::string>("<xmlattr>.agilegrasp", "unknown");
      agileGrasp.loadConfig(openposeConfigFile);

    }
  }

}


