/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

//
// Created by evan on 7/13/20.
//
// Edited by Eric on 9/17/20
// 
#include "SmallObjectGrasp.hpp"
#include "point_clouds/PointCloudUtilities.cpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <math.h>

using namespace diarc::stm;
using namespace diarc::grasp;

SmallObjectGrasp::SmallObjectGrasp()
    : size_thresh(0.07),
      gripper_depth(0.07) {
  logger = log4cxx::Logger::getLogger("diarc.detector.grasp.SmallObjectGrasp");
}

SmallObjectGrasp::~SmallObjectGrasp() {

}

bool SmallObjectGrasp::canCalculateGraspPoses(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const cv::Mat &transform) {

  calculateMinBB(cloud, transform);

  double bb_width = cv::norm(rrPts[1] - rrPts[2]);
  double bb_length = cv::norm(rrPts[0] - rrPts[1]);
  LOG4CXX_DEBUG(logger, boost::format("bb_width: %f.") % bb_width);
  LOG4CXX_DEBUG(logger, boost::format("bb_length: %f.") % bb_length);
  if (bb_width < size_thresh || bb_length < size_thresh) {
    return true;
  }

  return false;
}

void SmallObjectGrasp::calculateMinBB(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const cv::Mat &transform) {

// based on transform (4x4) inverse (ie, transpose) rotation matrix from robot base coordinates to the camera coordinates
  const double *transform_data = transform.ptr<double>(0);
  Eigen::Matrix4f cameraTransformMat;
  cameraTransformMat << transform_data[0], transform_data[4], transform_data[8], transform_data[12],
      transform_data[1], transform_data[5], transform_data[9], transform_data[13],
      transform_data[2], transform_data[6], transform_data[10], transform_data[14],
      transform_data[3], transform_data[7], transform_data[11], transform_data[15];
  cameraTrans.matrix() = cameraTransformMat;

  // transform object cloud into robot base frame (which is assumed to be parallel to table plane)
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*cloud,
                           *transformed_cloud,
                           cameraTrans.inverse());

  std::vector<cv::Point2f> points_2d;
  cv::Point2f pt_2d;
  for (auto &point : transformed_cloud->points) {
    if (point.x == point.x && point.y == point.y && point.z == point.z) {
      // not nan
      pt_2d.x = point.x;
      pt_2d.y = point.y;
      points_2d.push_back(pt_2d);
    }
  }

  // find minimum 2d bounding rectangle of projected points (on x-y plane)
  cv::Mat points_mat(points_2d);
  rrect = cv::minAreaRect(points_mat);
  rrect.points(rrPts);

  if (logger->isDebugEnabled()) {
    LOG4CXX_DEBUG(logger, boost::format("object bb (2D): (%f,%f), (%f,%f), (%f,%f), (%f,%f).")
        % rrPts[0].x % rrPts[0].y
        % rrPts[1].x % rrPts[1].y
        % rrPts[2].x % rrPts[2].y
        % rrPts[3].x % rrPts[3].y);
  }

  // find z-height of min bb, using cloud that has been transformed into base frame,
  // that way, using the axis-aligned BB will give a reasonable z-range
  pcl::PointXYZ min_point;
  pcl::PointXYZ max_point;
  Eigen::Vector3f mass_center_base_tmp;
  diarc::pc::util::calculateBoundingBox(transformed_cloud, min_point, max_point, mass_center_base_tmp);
  z_max = max_point.z;

  // construct object center of mass with z-value from 3D BB, and x- and y-values from 2D min BB
  cv::Point2f center_bb = rrect.center;
//  mass_center_base = Eigen::Vector3f(center_bb.x, center_bb.y, mass_center_base_tmp(2)); // using center of mass
  mass_center_base =
      Eigen::Vector3f(center_bb.x, center_bb.y, min_point.z + (max_point.z - min_point.z) / 2.0); // using z midpoint
}

std::vector<Grasp> SmallObjectGrasp::calculateGraspPoses(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const cv::Mat &cam_transform) {

  // create vector from center to mid-point of longest side of minimum bb (i.e., parallel to shortest side)
  double bb_width = cv::norm(rrPts[1] - rrPts[2]);
  double bb_length = cv::norm(rrPts[0] - rrPts[1]);
  cv::Point2f mid_pt_bb;
  if (bb_width < bb_length) {
    mid_pt_bb = (rrPts[0] + rrPts[1]) / 2;
  } else {
    mid_pt_bb = (rrPts[1] + rrPts[2]) / 2;
    // setting bb_width to always be less than bb_length to make side-grasp calculations easier
    double tmp = bb_width;
    bb_width = bb_length;
    bb_length = tmp;
  }
  cv::Point2f center_bb = rrect.center;
  Eigen::Vector3f orient_vector_2d = Eigen::Vector3f(mid_pt_bb.x - center_bb.x, mid_pt_bb.y - center_bb.y, 0);

  // align gripper's y-axis (direction of open/close) with vector created above
  orient_vector_2d.normalize();
  double y_angle = acos(orient_vector_2d.dot(Eigen::Vector3f::UnitY()));

  // Identity in base coordinate frame
  Eigen::Quaternionf q(1, 0, 0, 0); // w,x,y,z

  // translate to object's center of mass
  Eigen::Affine3f t(Eigen::Translation3f(mass_center_base(0), mass_center_base(1), mass_center_base(2))); //x,y,z
  Eigen::Affine3f transform = (t * q);

  // calculate grasps using 2D BB and center-of-mass (from 3D BB)
  std::vector<Grasp> grasps;

  //////////////// top-down grasp

  // TODO: should the other 3 top-down grasps be added too?

  // rotate z-y_angle to align with minimum bb, then y-90 so x-axis is facing down
  Eigen::Affine3f Y = Eigen::Affine3f(Eigen::AngleAxisf(0.5 * M_PI, Eigen::Vector3f::UnitY()));
  Eigen::Affine3f Z = Eigen::Affine3f(Eigen::AngleAxisf(y_angle, Eigen::Vector3f::UnitZ()));
  Eigen::Affine3f RXYZ = transform * Z * Y;

  // make sure gripper isn't reaching too far into object
//  float z_val = (abs(mass_center_base(2) - z_max) < gripper_depth) ? mass_center_base(2) : z_max - gripper_depth;
  float z_val = z_max;
  Eigen::Vector3f position(mass_center_base(0), mass_center_base(1), z_val);

  grasps.push_back(createGraspPose(cloud, RXYZ, position));

  ///////////////// 2 grasps along width axis (i.e., short side) of 2D non-axis-aligned BB

  // this should always be the case
  if (bb_width < size_thresh) {
    // TODO: adjust position x- and y- offsets to account for elongated objects (make sure gripper isn't reaching too far into object)
    if (bb_length / 2 > gripper_depth) {
      LOG4CXX_WARN(logger, "Grasp points are likely unreachable by gripper (bb_width).")
    }

    // rotate z-y_angle so y-axis is aligned along minor axis of 2D BB (i.e., gripper is grasping along short edge)
    Z = Eigen::Affine3f(Eigen::AngleAxisf(y_angle, Eigen::Vector3f::UnitZ()));
    RXYZ = transform * Z;
    grasps.push_back(createGraspPose(cloud, RXYZ, mass_center_base));

    // rotate z-y_angle+180
    Z = Eigen::Affine3f(Eigen::AngleAxisf(y_angle + M_PI, Eigen::Vector3f::UnitZ()));
    RXYZ = transform * Z;
    grasps.push_back(createGraspPose(cloud, RXYZ, mass_center_base));
  }


  ///////////////// 2 grasps along length axis (i.e., long side) of 2D non-axis-aligned BB

  if (bb_length < size_thresh) {
    // TODO: adjust position x- and y- offsets to account for elongated objects (make sure gripper isn't reaching too far into object)
    if (bb_width / 2 > gripper_depth) {
      LOG4CXX_WARN(logger, "Grasp points are likely unreachable by gripper (bb_length).")
    }

    // rotate z90 so y-axis is aligned along major axis of 2D BB (i.e., gripper is grasping along long edge)
    Z = Eigen::Affine3f(Eigen::AngleAxisf(y_angle + 0.5 * M_PI, Eigen::Vector3f::UnitZ()));
    RXYZ = transform * Z;
    grasps.push_back(createGraspPose(cloud, RXYZ, mass_center_base));

    // rotate z270
    Z = Eigen::Affine3f(Eigen::AngleAxisf(y_angle + 1.5 * M_PI, Eigen::Vector3f::UnitZ()));
    RXYZ = transform * Z;
    grasps.push_back(createGraspPose(cloud, RXYZ, mass_center_base));
  }

  //  ////////////////////// DEBUG VIZ CODE /////////////////////////////

//  //create the pcl viewer
//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Grasp Debug"));
//
//  //add cloud to viewer
//  viewer->addPointCloud<pcl::PointXYZRGB>(object->getCaptureData()->cloudRGB, "scene_cloud");
//  viewer->addPointCloud<pcl::PointXYZ>(cloud, "object_cloud");
//
//  viewer->initCameraParameters();
//  viewer->addCoordinateSystem(0.1, "camera", 0);
//  viewer->setPosition(0, 0);
//  viewer->setSize(640, 480);
//
//  for (int i = 0; i < grasps.size(); ++i) {
////  for (int i = 0; i < 2; ++i) {
//    pcl::PointXYZ center(grasps.at(i).points->at(0).x, grasps.at(i).points->at(0).y, grasps.at(i).points->at(0).z);
//    std::string pt_id = "grasp_center_" + std::to_string(i);
//    std::string orient_id = "grasp_orient_" + std::to_string(i);
//    viewer->addSphere(center, 0.01, 255, 0, 0, pt_id, 0);
//    viewer->addCoordinateSystem(0.1, debuggingOrientations[i], orient_id, 0);
//  }
//
//  //runviewer until closed
//  while (!viewer->wasStopped()) {
//    viewer->spinOnce(100);
//    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//  }
//
//  viewer->close();
//  ////////////////////// DEBUG VIZ CODE /////////////////////////////

  debuggingOrientations.clear();
  return grasps;
}

Grasp SmallObjectGrasp::createGraspPose(pcl::PointCloud<pcl::PointXYZ>::ConstPtr objectCloud,
                                            const Eigen::Affine3f &orientation,
                                            const Eigen::Vector3f &location) {
  // transform into camera coordinate frame
  Eigen::Affine3f finalTrans = cameraTrans * orientation;

  // debugging
  debuggingOrientations.push_back(finalTrans);

  // extract rotation as quaternion
  Eigen::Matrix4f pose_gripper = finalTrans.matrix();
  Eigen::Matrix3f rotation = pose_gripper.block(0, 0, 3, 3);
  Eigen::Quaternionf quat = Eigen::Quaternionf(rotation);
  quat.normalize();

  // calculate grasp point
  Eigen::Vector3f mass_center_cam;
  mass_center_cam = cameraTrans * location;
  pcl::PointXYZ point(mass_center_cam[0], mass_center_cam[1], mass_center_cam[2]);

  // TODO: fix this to project only in the z-axis (i.e., down)
  // "project" point onto object point cloud
//  point = findClosestPointInCloud(objectCloud, point);

  pcl::PointCloud<pcl::PointXYZ>::Ptr
      grasp_points = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  grasp_points->push_back(point);
  grasp_points->push_back(point);

  LOG4CXX_DEBUG(logger, boost::format("grasp orientation (%f,%f,%f,%f).")
      % quat.x() % quat.y() % quat.z() % quat.w());
  LOG4CXX_DEBUG(logger, boost::format("grasp position (%d,%d,%d).")
      % point.x % point.y % point.z);

  return Grasp(grasp_points, quat);
}

pcl::PointXYZ SmallObjectGrasp::findClosestPointInCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                                        const pcl::PointXYZ &point) {
  pcl::PointXYZ closestPt = point;
  float smallestDist = MAXFLOAT;
  float dist;

  for (int i = 0; i < cloud->points.size(); ++i) {
    dist = pcl::euclideanDistance(cloud->points[i], point);
    if (dist < smallestDist) {
      smallestDist = dist;
      closestPt = cloud->points[i];
    }
  }

  return closestPt;
}
