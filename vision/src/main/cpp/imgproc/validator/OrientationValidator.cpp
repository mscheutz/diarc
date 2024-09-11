
/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "OrientationValidator.hpp"

#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>

#include "capture/calibration/Cameras.hpp"
#include "display/Display.hpp"

using namespace diarc::stm;

OrientationValidator::OrientationValidator(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight, const bool isStereo)
: ObjectValidator(processorId, imgWidth, imgHeight, isStereo) {
  visionProcessName = "OrientationValidator";
  logger = log4cxx::Logger::getLogger("diarc.imgproc.validator.OrientationValidator");
}

OrientationValidator::~OrientationValidator() {
}


void OrientationValidator::handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification) {
  LOG4CXX_TRACE(logger, "[handleMemoryObjectNotification] method entered.");

  //EAK: this is just a stubbed out Validator that needs to be implemented.
  // Take a look at one of the other Validators as an example.
}
