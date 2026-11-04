/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

//
// Created by evan on 7/13/20.
//

#ifndef DIARC_VISION_GRASP_GRASP_HPP
#define DIARC_VISION_GRASP_GRASP_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace diarc {
  namespace grasp {

    class Grasp {
      public:
       Grasp(pcl::PointCloud<pcl::PointXYZ>::Ptr points_, Eigen::Quaternionf orientation_)
           : points(points_),
             orientation(orientation_) {
       }
       pcl::PointCloud<pcl::PointXYZ>::Ptr points;
       Eigen::Quaternionf orientation;
    };
  }
}

#endif //DIARC_VISION_GRASP_GRASP_HPP
