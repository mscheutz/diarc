/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

//
// Created by evan on 7/13/20.
//

#ifndef DIARC_VISION_GRASP_SMALLOBJECTGRASPS_HPP
#define DIARC_VISION_GRASP_SMALLOBJECTGRASPS_HPP

#include "Grasp.hpp"
#include <log4cxx/logger.h>
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>

namespace diarc {
  namespace grasp {

    class SmallObjectGrasp {
    public:
      typedef boost::shared_ptr<SmallObjectGrasp> Ptr;
      typedef boost::shared_ptr<const SmallObjectGrasp> ConstPtr;

      SmallObjectGrasp();
      ~SmallObjectGrasp();

      //! determine if calculateGraspPoses will result in useful grasps for the MO
      bool canCalculateGraspPoses(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const cv::Mat &transform);

      //! calculate grasp pose (currently only one is calculated) for MO
      //! NOTE: it is assumed that canCalculateGraspPoses is called before this method is called, with the same MO
      std::vector<Grasp> calculateGraspPoses(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const cv::Mat &transform);

     private:
      //! calculate the minimum bounding box in base coordinate frame and store relevant results in class fields
      void calculateMinBB(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const cv::Mat &transform);

      Grasp createGraspPose(pcl::PointCloud<pcl::PointXYZ>::ConstPtr objectCloud,
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
  }
}

#endif //DIARC_VISION_GRASP_SMALLOBJECTGRASPS_HPP
