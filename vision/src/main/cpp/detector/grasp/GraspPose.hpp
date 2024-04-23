/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

//
// Created by evan on 7/13/20.
//

#ifndef ADE_VISION_DETECTOR_GRASP_GRASPPOSE_HPP
#define ADE_VISION_DETECTOR_GRASP_GRASPPOSE_HPP

class GraspPose {
 public:
  GraspPose(pcl::PointCloud<pcl::PointXYZ>::Ptr points_, Eigen::Quaternionf orientation_)
      : points(points_),
        orientation(orientation_) {
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr points;
  Eigen::Quaternionf orientation;
};

#endif //ADE_VISION_DETECTOR_GRASP_GRASPPOSE_HPP
