/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

//
// Created by evan on 7/13/20.
//

#include "GpdGrasp.hpp"

#include "point_clouds/PCLFunctions.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/eigen.hpp>
#include <boost/thread.hpp>

using namespace diarc::grasp;

GpdGrasp::GpdGrasp(const std::string &config) {
  logger = log4cxx::Logger::getLogger("diarc.grasp.GpdGrasp");

  // std::string cfg_filename = "/home/evan/code/diarc/diarc/vision/src/main/resources/config/edu/tufts/hrilab/vision/grasp/gpd/gpd.cfg";
  LOG4CXX_DEBUG(logger, boost::format("[GpdGrasp] configFileName: %s.") % config);
  detector = std::shared_ptr<gpd::GraspDetector>(new gpd::GraspDetector(config));
}

GpdGrasp::~GpdGrasp() {

}

std::vector<Grasp> GpdGrasp::calculateGraspPoses(pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, const cv::Mat &transform) {

  std::string pcd_filename = "/dev/shm/grasp_object.pcd";
  pcl::io::savePCDFile(pcd_filename, *cloud);

  pcl::PointCloud<PointType>::Ptr object_cloud(new pcl::PointCloud<PointType>);
  pcl::copyPointCloud(*cloud, *object_cloud);

  //get camera Transform Base of the robot
  // Eigen::Matrix4d cam_tf; //Matrix4d <double,4,4>
  // cv2eigen(transform, cam_tf);

  Eigen::MatrixXi camera_source = Eigen::MatrixXi::Ones(1, cloud->size()); // all points in cloud are from camera 0
  Eigen::Matrix3Xd view_points(3, 1);
  view_points << 0.0, 0.0, 0.0;
  gpd::util::Cloud gpdCloud(object_cloud, camera_source, view_points);

  // gpd::util::Cloud gpdCloud(pcd_filename, view_points);

  // Preprocess the point cloud.
  detector->preprocessPointCloud(gpdCloud);

  // Detect grasp poses.
  std::vector<std::unique_ptr<gpd::candidate::Hand>> hands = detector->detectGrasps(gpdCloud);

  //objects to collect the quaternion and points for all grasp info that will populate grasps
  pcl::PointCloud<pcl::PointXYZ>::Ptr points;
  Eigen::Quaternionf orientation;

  // if there are handles (grasps) found
  std::vector<Grasp> grasps;
  if (hands.size() > 0) {
    for (auto &hand : hands) {
      // reset reused data structures
      points.reset(new pcl::PointCloud<pcl::PointXYZ>());

      // calculate the grasp
      calculateGraspPose(hand, points, orientation);

      // add grasp to results
      grasps.push_back(Grasp(points, orientation));
    }

    //plot coordinate frames for debugging purposes
    // if (pcl_viz) { //to turn on/off, make changes in config XML
    //   pcl::PointCloud<PointType>::Ptr object_cloud_display(new pcl::PointCloud<PointType>);
    //   pcl::copyPointCloud(*cloud, *object_cloud_display);
    //   plotAntipodalFrames(handles, object_cloud_display);
    // }
  }

  return grasps;
}

void GpdGrasp::calculateGraspPose(const std::unique_ptr<gpd::candidate::Hand>& hand, const pcl::PointCloud<pcl::PointXYZ>::Ptr& points, Eigen::Quaternionf& quat) {

  // compute the quaternion
  Eigen::Matrix3f orient = hand->getOrientation().cast<float>();
  quat = Eigen::Quaternionf(orient);
  quat.normalize();
  LOG4CXX_DEBUG(logger, boost::format("[QUATERNION PR2 x,y,z,w] [ %.3f, %.3f, %.3f , %.3f] ") % quat.x() % quat.y() % quat.z() % quat.w());

  // point for grasp
  Eigen::Vector3f hand_position = hand->getPosition().cast<float>();
  pcl::PointXYZ point(hand_position(0), hand_position(1), hand_position(2));
  points->push_back(point);
}
