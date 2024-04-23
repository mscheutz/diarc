
/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "ValidationResult.hpp"

using namespace ade::stm;
//using namespace ade::fol;

ValidationResult::ValidationResult(const float& confidence, const PredicateHelper& descriptor,
        MemoryObjectMask::ConstPtr mask)
: MemoryObjectMask(*mask),
confidence_(confidence),
descriptor_(descriptor) {
}

ValidationResult::ValidationResult(const float& confidence, const PredicateHelper& descriptor,
        CaptureData::ConstPtr& captureData, const cv::Mat_<float>& imageMask)
: MemoryObjectMask(captureData, imageMask),
confidence_(confidence),
descriptor_(descriptor) {
}

ValidationResult::ValidationResult(const float& confidence, const PredicateHelper& descriptor,
        CaptureData::ConstPtr& captureData, const std::vector<int>& indicesMask)
: MemoryObjectMask(captureData, indicesMask),
confidence_(confidence),
descriptor_(descriptor) {
}

ValidationResult::ValidationResult(const float& confidence, const PredicateHelper& descriptor,
        CaptureData::ConstPtr& captureData, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloudMask)
: MemoryObjectMask(captureData, cloudMask),
confidence_(confidence),
descriptor_(descriptor) {
}

ValidationResult::~ValidationResult() {

}

float ValidationResult::getConfidence() const {
  return (confidence_);
}

const PredicateHelper& ValidationResult::getDescriptor() const {
  return (descriptor_);
}