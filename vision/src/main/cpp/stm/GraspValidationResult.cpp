
/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "GraspValidationResult.hpp"

using namespace ade::stm;

GraspValidationResult::GraspValidationResult(const float& confidence, const PredicateHelper& descriptor, CaptureData::ConstPtr captureData,
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr points, const Eigen::Quaternionf& orientation)
: ValidationResult(confidence, descriptor, captureData, points),
orientation_(orientation) {

}

GraspValidationResult::~GraspValidationResult() {

}

const Eigen::Quaternionf& GraspValidationResult::getOrientation() const {
  return orientation_;
}