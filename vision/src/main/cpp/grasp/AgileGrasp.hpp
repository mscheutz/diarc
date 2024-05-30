/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

//
// Created by evan on 7/13/20.
//

#ifndef DIARC_VISION_GRASP_AGILEGRASP_HPP
#define DIARC_VISION_GRASP_AGILEGRASP_HPP

#include "stm/MemoryObject.hpp"
#include "Grasp.hpp"

#include <agile_grasp/localization.h>
#include <vector>
#include <log4cxx/logger.h>

namespace diarc {
  namespace grasp {

    class AgileGrasp {
      typedef pcl::PointXYZ PointType;
      typedef boost::shared_ptr<Localization> LocalizationPtr;

     public:
      typedef boost::shared_ptr<AgileGrasp> Ptr;
      typedef boost::shared_ptr<const AgileGrasp> ConstPtr;

      AgileGrasp();
      ~AgileGrasp();

      void loadConfig(const std::string &config);

      std::vector<Grasp> calculateGraspPoses(diarc::stm::MemoryObject::Ptr& object);

     private:

      //! Computes the pose for PR2-like gripper: quaternion and points
      void calculateGraspPose(const Handle& antipodal, pcl::PointCloud<pcl::PointXYZ>::Ptr& points, Eigen::Quaternionf & quat);

      //! Computes the normalized Quaternion for the orientation of the gripper to grasp an object
      //! GraspHypothesis antipodal contains axis and approach vectors to compute the
      //! referential of a rotation matrix. Then it is transformed to Quaternion.
      Eigen::Quaternionf calculateEndEffectorOrientation(const Handle & antipodal);

      //! displays the point cloud annotated with calculated grasp info from agile
      //! the grasp info displayed here does not contain and coordinate frame transforms
      //! into the ADE preferred coordinate frame
      void plotAntipodalFrames(const std::vector<Handle>& antipodals, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

      LocalizationPtr localization;
      int min_inliers;
      int plotting_mode;
      bool pcl_viz;
      std::string svm_file;
      bool antipodal;
      bool clustering_pc;
      double handle_min_length;

      log4cxx::LoggerPtr logger;
    };
  }
}

#endif //DIARC_VISION_GRASP_AGILEGRASP_HPP
