/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   ValidationResult.hpp
 * Author: Evan Krause <evan.krause@tufts.edu>
 *
 * Created on March 7, 2016, 1:38 PM
 */

#ifndef VALIDATIONRESULT_HPP
#define	VALIDATIONRESULT_HPP

#include "common/CaptureData.hpp"
#include "common/fol/PredicateHelper.hpp"
#include "MemoryObjectMask.hpp"

#include <boost/format.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/unordered_set.hpp>
#include <opencv2/opencv.hpp>
#include <tr1/unordered_map>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace diarc {
  namespace stm {

    class ValidationResult : public MemoryObjectMask {
    public:
      typedef boost::shared_ptr<ValidationResult> Ptr;
      typedef boost::shared_ptr<const ValidationResult> ConstPtr;
      typedef std::vector<ValidationResult::ConstPtr> Vec;
      typedef std::tr1::unordered_map<PredicateHelper, Vec, std::tr1::hash<PredicateHelper>, predhelper_equal_to> VecByPredicate;

      ValidationResult(const float& confidence, const PredicateHelper& descriptor,
              MemoryObjectMask::ConstPtr mask);
      ValidationResult(const float& confidence, const PredicateHelper& descriptor,
              CaptureData::ConstPtr& captureData, const cv::Mat_<float>& imageMask);
      ValidationResult(const float& confidence, const PredicateHelper& descriptor,
              CaptureData::ConstPtr& captureData, const std::vector<int>& indicesMask);
      ValidationResult(const float& confidence, const PredicateHelper& descriptor,
              CaptureData::ConstPtr& captureData, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloudMask);

      virtual ~ValidationResult();

      float getConfidence() const;
      const PredicateHelper& getDescriptor() const;

    protected:
      //! detection or validation confidence [0 1]
      float confidence_;

      //! descriptor (e.g., red(X), box(X), near(X,Y), etc)
      PredicateHelper descriptor_;

    };

  } //namespace stm
} //namespace diarc

#endif	/* VALIDATIONRESULT_HPP */

