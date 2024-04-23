/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "SpatialRelationValidator.hpp"

#include <cfloat>
#include <cmath>
#include <functional>
#include <pcl/common/time.h>
#include <pcl/registration/distances.h>

using namespace ade::stm;

SpatialRelationValidator::SpatialRelationValidator(const long long& processorId, const unsigned int imgWidth,
        const unsigned int imgHeight, const bool isStereo)
: ObjectValidator(processorId, imgWidth, imgHeight, isStereo) {
  visionProcessName = "SpatialRelationValidator";
  logger = log4cxx::Logger::getLogger("ade.imgproc.validator.SpatialRelationValidator");
}

SpatialRelationValidator::~SpatialRelationValidator() {
}

void SpatialRelationValidator::loadConfig(const std::string& config) {
  // TODO: move these hardcoded "on", "near", and "part_of" into config file
}

void SpatialRelationValidator::handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification) {
  LOG4CXX_TRACE(logger, "[handleMemoryObjectNotification] method entered.");
  if (logger->isDebugEnabled()) {
    pcl::ScopeTime t("Spatial Relation Validation Time.");
  }

  MemoryObject::Ptr object = notification->object;

  // get current descriptors to process
  DescriptorsByTypeConstPtr descriptorsByType = getTypes();
  DescriptorsByType::const_iterator descriptorsByType_iter = descriptorsByType->find(object->getTypeId());
  PredicateHelper::Set::const_iterator descriptors_itr;

  bool relationsFound = false;
  if (descriptorsByType_iter != descriptorsByType->end()) {
    //iterate through the (type's) descriptors
    for (descriptors_itr = descriptorsByType_iter->second.begin(); descriptors_itr != descriptorsByType_iter->second.end(); ++descriptors_itr) {
      if (descriptors_itr->getNumArgs() != 2) {
        LOG4CXX_ERROR(logger, boost::format("%s does not have 2 args.") % descriptors_itr->toString());
        relationsFound = false;
        break;
      }

      // get MOs for each arg
      std::string arg0 = descriptors_itr->getArg(0);
      std::string arg1 = descriptors_itr->getArg(1);
      MemoryObject::Vec arg0_MOs = object->getMemoryObjects(arg0);
      MemoryObject::Vec arg1_MOs = object->getMemoryObjects(arg1);
      MemoryObject::Vec::iterator arg0_MOs_iter;
      MemoryObject::Vec::iterator arg1_MOs_iter;
      LOG4CXX_TRACE(logger, boost::format("%s MOs: %lu. %s MOs: %lu.") % arg0 % arg0_MOs.size() % arg1 % arg1_MOs.size());

      // calculate spatial relation for each pair of MOs
      relationsFound = false;
      for (arg0_MOs_iter = arg0_MOs.begin(); arg0_MOs_iter != arg0_MOs.end(); ++arg0_MOs_iter) {
        for (arg1_MOs_iter = arg1_MOs.begin(); arg1_MOs_iter != arg1_MOs.end(); ++arg1_MOs_iter) {
          if (calculateSpatialRelation(*descriptors_itr, *arg0_MOs_iter, *arg1_MOs_iter)) {
            relationsFound = true;
          }
        }
      }
      if (!relationsFound) {
        break; //if didn't find this relation, don't check the rest
      }
    }
  } else {
    LOG4CXX_ERROR(logger, boost::format("No descriptor found for typeId: %d.") % object->getTypeId());
  }

  // if non-zero confidence relation found, send notifications and display results
  if (relationsFound) {
    sendValidationNotifications(object);

    if (getDisplayFlag()) {
      displayMemoryObjectRelations(object);
    }
  }
}

bool SpatialRelationValidator::calculateSpatialRelation(const PredicateHelper& descriptor,
        MemoryObject::Ptr& mo0, MemoryObject::Ptr& mo1) {
  // TODO: move these hardcoded "on", "near", and "partOf" into config file
  if (descriptor.getName().compare("near") == 0) {
    return calculateNearRelation(descriptor, mo0, mo1);
  } else if (descriptor.getName().compare("on") == 0 || descriptor.getName().compare("partOf") == 0 || descriptor.getName().compare("part_of") == 0) {	//"part_of" for backwards compatibility
    return calculateOnRelation(descriptor, mo0, mo1);
  } else {
    LOG4CXX_ERROR(logger, boost::format("[calculateSpatialRelation] can not process: %s.") % descriptor.toString());
    return false;
  }
}

bool SpatialRelationValidator::calculateNearRelation(const PredicateHelper& descriptor,
        MemoryObject::Ptr& mo0, MemoryObject::Ptr& mo1) {
  LOG4CXX_DEBUG(logger, "[calculateNearRelation] method entered.");

  std::vector<int> indices0 = mo0->getDetectionMask()->getIndicesMask();
  std::vector<int> indices1 = mo1->getDetectionMask()->getIndicesMask();
  int x0, y0, x1, y1, index0, index1;
  float distance, distConf, currConf, bestConf = 0;
  int imgWidth = mo0->getDetectionMask()->getImageMask().cols;
  for (int i0 = 0; i0 < indices0.size(); ++i0) {
    index0 = indices0[i0];
    x0 = (index0 % imgWidth);
    y0 = (index0 / imgWidth);
    for (int i1 = 0; i1 < indices1.size(); ++i1) {
      index1 = indices1[i1];
      x1 = (index1 % imgWidth);
      y1 = (index1 / imgWidth);
      distance = static_cast<float> (pcl::distances::l2(mo0->getDetectionMask()->getObjectPoint(index0).getVector4fMap(), mo1->getDetectionMask()->getObjectPoint(index1).getVector4fMap()));
      distConf = std::exp(-1 * distance); //e^(-x)
      currConf = mo0->getDetectionMask()->getImageMask()(y0, x0) * mo1->getDetectionMask()->getImageMask()(y1, x1) * distConf;
      if (currConf > bestConf) {
        bestConf = currConf;
      }
    }
  }

  LOG4CXX_DEBUG(logger, boost::format("[calculateNearRelation] adding near relation between %d and %d with conf: %f.") % mo0->getId() % mo1->getId() % bestConf);
  mo0->addRelation(bestConf, descriptor, mo1);
  return true;
}

bool SpatialRelationValidator::calculateOnRelation(const PredicateHelper& descriptor,
        MemoryObject::Ptr& mo0, MemoryObject::Ptr& mo1) {
  LOG4CXX_DEBUG(logger, "[calculateOnRelation] method entered.");

  std::vector<int> indices0 = mo0->getDetectionMask()->getIndicesMask();
  std::vector<int> indices1 = mo1->getDetectionMask()->getIndicesMask();
  int x0, y0, x1, y1, index0, index1;
  float distance;
  float distThresh = 0.07;//0.01; //meters
  int onCount = 0;
  int imgWidth = mo0->getDetectionMask()->getImageMask().cols;
  // for each point in mo0, find closest point in mo1
  LOG4CXX_DEBUG(logger, boost::format("[calculateOnRelation] mo0 %s indices: %d. mo0 %s indices: %d.")
          % mo0->getVariableName() % indices0.size() % mo1->getVariableName() % indices1.size());
  for (int i0 = 0; i0 < indices0.size(); ++i0) {
    index0 = indices0[i0];
    x0 = (index0 % imgWidth);
    y0 = (index0 / imgWidth);
    // pcl::PointXYZ p0 = mo0->getDetectionMask()->getObjectPoint(index0);
    for (int i1 = 0; i1 < indices1.size(); ++i1) {
      index1 = indices1[i1];
      x1 = (index1 % imgWidth);
      y1 = (index1 / imgWidth);
      // pcl::PointXYZ p1 = mo1->getDetectionMask()->getObjectPoint(index1);
      //      LOG4CXX_TRACE(logger, boost::format("[calculateOnRelation] mo0 index: %d. (x,y) = (%d,%d). (x,y,z) = (%f,%f,%f).")
      //              % index0 % x0 % y0 % p0.x % p0.y % p0.z);
      //      LOG4CXX_TRACE(logger, boost::format("[calculateOnRelation] mo1 index: %d. (x,y) = (%d,%d). (x,y,z) = (%f,%f,%f).")
      //              % index1 % x1 % y1 % p1.x % p1.y % p1.z);
      distance = static_cast<float> (pcl::distances::l2(mo0->getDetectionMask()->getObjectPoint(index0).getVector4fMap(), mo1->getDetectionMask()->getObjectPoint(index1).getVector4fMap()));
      //      LOG4CXX_TRACE(logger, boost::format("[calculateOnRelation] distance: %f.") % distance);
      if (distance < distThresh) {
        ++onCount;
        break;
      }
    }
  }

  LOG4CXX_DEBUG(logger, boost::format("[calculateOnRelation] onCount: %f.") % onCount);
  if (onCount > 0) {
    //TODO: how should this "on" confidence be calculated?
    mo0->addRelation(0.95, descriptor, mo1);
    return true;
  } else {
    return false;
  }
}
