/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

//
// Created by evan on 7/13/20.
//

#ifndef ADE_VISION_DETECTOR_GRASP_SMALLOBJECTGRASPS_HPP
#define ADE_VISION_DETECTOR_GRASP_SMALLOBJECTGRASPS_HPP

#include "stm/MemoryObject.hpp"
#include "GraspPose.hpp"
#include <log4cxx/logger.h>

class SmallObjectGrasp {

 public:
  SmallObjectGrasp();
  ~SmallObjectGrasp();
  //! determine if calculateGraspPoses will result in useful grasps for the MO
  bool canCalculateGraspPoses(ade::stm::MemoryObject::Ptr &object);

  //! calculate grasp pose (currently only one is calculated) for MO
  //! NOTE: it is assumed that canCalculateGraspPoses is called before this method is called, with the same MO
  std::vector<GraspPose> calculateGraspPoses(ade::stm::MemoryObject::Ptr &object);

 private:
  //! calculate the minimum bounding box in base coordinate frame and store relevant results in class fields
  void calculateMinBB(ade::stm::MemoryObject::Ptr &object);

  GraspPose createGraspPose(pcl::PointCloud<pcl::PointXYZ>::ConstPtr objectCloud,
                            const Eigen::Affine3f &RXYZ,
                            const Eigen::Vector3f &location);
  pcl::PointXYZ findClosestPointInCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const pcl::PointXYZ &point);

  /////////////////////////////
  // NOTE: all of these fields filled in during call to calculateMinBB

  Eigen::Affine3f cameraTrans;
  Eigen::Vector3f mass_center_base;

  // minimum 2D BB of object projected onto x-y plane of base frame
  cv::RotatedRect rrect;
  cv::Point2f rrPts[4]; //The order is bottomLeft, topLeft, topRight, bottomRight

  // max z-value of object in base frame
  float z_max;

  /////////////////////////////

  // bb width threshold (meters)
  float size_thresh;

  // gripper depth (meters)
  float gripper_depth;

  // debugging
  std::vector<Eigen::Affine3f> debuggingOrientations;

  log4cxx::LoggerPtr logger;
};

#endif //ADE_VISION_DETECTOR_GRASP_SMALLOBJECTGRASPS_HPP
