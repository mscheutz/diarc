/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   GraspValidationResult.hpp
 * Author: Evan Krause <evan.krause@tufts.edu>
 *
 * Created on March 7, 2016, 1:49 PM
 */

#ifndef GRASPVALIDATIONRESULT_HPP
#define	GRASPVALIDATIONRESULT_HPP

#include "ValidationResult.hpp"

namespace ade {
  namespace stm {

    class GraspValidationResult : public ValidationResult {
    public:
      typedef boost::shared_ptr<GraspValidationResult> Ptr;
      typedef boost::shared_ptr<const GraspValidationResult> ConstPtr;

      GraspValidationResult(const float& confidence, const PredicateHelper& descriptor, CaptureData::ConstPtr captureData,
              pcl::PointCloud<pcl::PointXYZ>::ConstPtr points, const Eigen::Quaternionf& orientation);

      virtual ~GraspValidationResult();
      
      const Eigen::Quaternionf& getOrientation() const;

    private:

      //! quaternion(s) specifying orientation(s) of end effector(s)
      Eigen::Quaternionf orientation_;
    };

  } //namespace stm
} //namespace ade

#endif	/* GRASPVALIDATIONRESULT_HPP */

