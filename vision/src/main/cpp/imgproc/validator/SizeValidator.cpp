/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "SizeValidator.hpp"
#include "capture/calibration/Cameras.hpp"
#include "display/Display.hpp"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>

using namespace ade::stm;

SizeValidator::SizeValidator(const long long& processorId, const unsigned int imgWidth,
        const unsigned int imgHeight, const bool isStereo)
: ObjectValidator(processorId, imgWidth, imgHeight, isStereo) {
  visionProcessName = "SizeValidator";
  logger = log4cxx::Logger::getLogger("ade.imgproc.validator.SizeValidator");
}

SizeValidator::~SizeValidator() {
}

void SizeValidator::handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification) {
  LOG4CXX_TRACE(logger, "[handleMemoryObjectNotification] method entered.");
  MemoryObject::Ptr object = notification->object;
  //get captured frames info
  const cv::Point3d& objectSize = object->getDetectionMask()->getDimensions();

  // get current descriptors to process
  TypesByDescriptorConstPtr descriptors = getDescriptors();

  int result;
  TypesByDescriptor::const_iterator descriptors_itr;
  for (descriptors_itr = descriptors->begin(); descriptors_itr != descriptors->end(); ++descriptors_itr) {
    std::map<std::string, std::vector<double>>::iterator it = sizeMap.find(descriptors_itr->first.getName());
    if (it == sizeMap.end()) {
      continue;
    }
    result = isInSizeRange(it->second, objectSize);
    if (!result) {
      LOG4CXX_DEBUG(logger, boost::format("Object is not of size: %s") % it->first);
      continue;
    }

    LOG4CXX_DEBUG(logger,  boost::format("Object detected of size: %s") % it->first);
    object->addValidationResult(1.0f, descriptors_itr->first);
  }

  sendValidationNotifications(object);

  if (getDisplayFlag()) {
    displayMemoryObjectValidation(object, true);
  }
}

void SizeValidator::loadConfig(const std::string& config) {
  // populate tree structure pt
  using boost::property_tree::ptree;
  ptree pt;
  read_xml(config, pt);

  std::string predicateName;

  // parse tree

  BOOST_FOREACH(ptree::value_type const& predicateNode, pt.get_child("processor")) {
    if (predicateNode.first.compare("predicate") == 0) {
      predicateName = predicateNode.second.get<std::string> ("<xmlattr>.name", "unknown");

      double min = predicateNode.second.get<double>("min", 0.0);
      double max = predicateNode.second.get<double>("max", 0.0);
      LOG4CXX_DEBUG(logger, boost::format("[loadConfig] predicateName: %s. Range: (%f,%f)") % predicateName % min % max);

      std::vector<double> range = {min,max};
      sizeMap.insert(std::pair<std::string, std::vector<double> > (predicateName, range));
    }
  }
}

bool SizeValidator::isInSizeRange(const std::vector<double>& targetSize, const cv::Point3d& bb) {
  double min = targetSize.at(0);
  double max = targetSize.at(1);

  LOG4CXX_DEBUG(logger,  boost::format("BB size: (%f,%f,%f). Target range: (%f,%f)") % bb.x % bb.y % bb.z % min % max);

  // get largest dimension of bb
  double maxDim = (bb.x > bb.y) ? bb.x : bb.y;
  maxDim = (bb.z > maxDim) ? bb.z : maxDim;

  if (maxDim > min && maxDim < max) {
    return true;
  }

  return false;
}
