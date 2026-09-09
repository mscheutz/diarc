/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

//
// Created by evan on 7/13/20.
//

#ifndef DIARC_VISION_GRASP_GPDGRASP_HPP
#define DIARC_VISION_GRASP_GPDGRASP_HPP

#include "Grasp.hpp"

#include <vector>
#include <log4cxx/logger.h>
#include <opencv2/opencv.hpp>

#include <gpd/grasp_detector.h>

namespace diarc {
  namespace grasp {

    class GpdGrasp {
      typedef pcl::PointXYZRGBA PointType;

     public:
      typedef boost::shared_ptr<GpdGrasp> Ptr;
      typedef boost::shared_ptr<const GpdGrasp> ConstPtr;

      GpdGrasp(const std::string &config);
      ~GpdGrasp();

      std::vector<Grasp> calculateGraspPoses(pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, const cv::Mat &transform);

     private:

      //! Computes the pose for parallel jaw gripper: quaternion and point
      void calculateGraspPose(const std::unique_ptr<gpd::candidate::Hand>& hand, const pcl::PointCloud<pcl::PointXYZ>::Ptr& points, Eigen::Quaternionf& quat);

      //! GPD detector
      std::shared_ptr<gpd::GraspDetector> detector;

      log4cxx::LoggerPtr logger;
    };
  }
}

#endif //DIARC_VISION_GRASP_GPDGRASP_HPP
