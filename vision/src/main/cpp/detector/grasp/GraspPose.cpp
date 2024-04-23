/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

//
// Created by evan on 7/13/20.
//

#include "SmallObjectGrasp.hpp"
#include "point_clouds/PointCloudUtilities.cpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <math.h>

using namespace ade::stm;

SmallObjectGrasp::SmallObjectGrasp() {
  logger = log4cxx::Logger::getLogger("ade.detector.grasp.SmallObjectGrasp");
}

SmallObjectGrasp::~SmallObjectGrasp() {

}

bool SmallObjectGrasp::canCalculateGraspPoses(pcl::PointCloud<PointType>::ConstPtr object_cloud) {
  pcl::PointXYZ min_point;
  pcl::PointXYZ max_point;

  ade::pc::util::calculateBoundingBox(object_cloud, min_point, max_point, mass_center);
  float size_thresh = 0.7; //m
  LOG4CXX_DEBUG(logger, boost::format("max_point: %f %f %f.") % max_point.x % max_point.y % max_point.z);
  LOG4CXX_DEBUG(logger, boost::format("min_point: %f %f %f.") % min_point.x % min_point.y % min_point.z);
  LOG4CXX_DEBUG(logger, boost::format("mass_center: %f %f %f.") % mass_center(0) % mass_center(1) % mass_center(2));
  LOG4CXX_DEBUG(logger, boost::format("bb size: %f %f %f.")
      % (max_point.x - min_point.x) % (max_point.y - min_point.y) % (max_point.z - min_point.z));
  if (max_point.x - min_point.x < size_thresh
      && max_point.y - min_point.y < size_thresh
      && max_point.z - min_point.z < size_thresh) {
    return true;
  }

  return false;
}

std::vector<GraspPose> SmallObjectGrasp::calculateGraspPoses(MemoryObject::Ptr& object) {

  // based on transform (4x4) inverse (ie, transpose) rotation matrix from robot base coordinates to the camera coordinates
  const double *transform_data = object->getCaptureData()->transform.ptr<double>(0);
  Eigen::Matrix4f cameraTransformMat;
  cameraTransformMat << transform_data[0], transform_data[4], transform_data[8], transform_data[12],
      transform_data[1], transform_data[5], transform_data[9], transform_data[13],
      transform_data[2], transform_data[6], transform_data[10], transform_data[14],
      transform_data[3], transform_data[7], transform_data[11], transform_data[15];
  Eigen::Affine3f cameraTrans;
  cameraTrans.matrix() = cameraTransformMat;

  /////////////////////////////////////
  // calculate orientation
  // project object could into table plane (actually robot base frame which is assumed to be parallel to table)
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr object_cloud = object->getDetectionMask()->getObjectPointCloud();
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud <pcl::PointXYZ>);
  pcl::transformPointCloud(*object_cloud, *transformed_cloud, cameraTrans.inverse());

  std::vector <cv::Point2f> points_2d;
  cv::Point2f pt_2d;
  pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud(new pcl::PointCloud <pcl::PointXYZ>);
  pcl::PointXYZ pt_3d;
  for (auto &point : transformed_cloud->points) {
    if (point.x == point.x && point.y == point.y && point.z == point.z) {
      // not nan
      pt_2d.x = point.x;
      pt_2d.y = point.y;
      points_2d.push_back(pt_2d);
      pt_3d.x = point.x;
      pt_3d.y = point.y;
      pt_3d.z = 0;
      projected_cloud->push_back(pt_3d);
    }
  }

  // find minimum 2d bounding rectangle of projected points (on x-y plane)
  cv::Mat points_mat(points_2d);
  cv::RotatedRect rrect = cv::minAreaRect(points_mat);
  //The order is bottomLeft, topLeft, topRight, bottomRight.
  cv::Point2f rrPts[4];
  rrect.points(rrPts);

  // create vector from center to mid-point of shortest side of minimum bb
  double bb_width = cv::norm(rrPts[1] - rrPts[2]);
  double bb_height = cv::norm(rrPts[0] - rrPts[1]);
  cv::Point2f center_bb = rrect.center;
  cv::Point2f mid_pt_bb = (bb_width < bb_height) ? ((rrPts[1] + rrPts[2]) / 2) : ((rrPts[0] + rrPts[1]) / 2);
  Eigen::Vector3f orient_vector_2d = Eigen::Vector3f(mid_pt_bb.x - center_bb.x, mid_pt_bb.y - center_bb.y, 0);

  // align gripper's y-axis (direction of open/close) with vector created above
  orient_vector_2d.normalize();
  double y_angle = acos(orient_vector_2d.dot(Eigen::Vector3f::UnitY()));

  ///////////////////////////////////////////////////////////

  // Identity in base coordinate frame
  Eigen::Quaternionf q(1, 0, 0, 0); // w,x,y,z

  // transform object's center of mass from camera to base frame
  Eigen::Vector3f mass_center_base_tmp;
  mass_center_base_tmp = cameraTrans.inverse() * mass_center;
  Eigen::Vector3f mass_center_base(center_bb.x, center_bb.y, mass_center_base_tmp(2));

  // translate to object's center of mass
  Eigen::Affine3f t(Eigen::Translation3f(mass_center_base(0), mass_center_base(1), mass_center_base(2))); //x,y,z
  Eigen::Affine3f transform = (t * q);

  Eigen::Vector3f mass_center_cam;
  mass_center_cam = cameraTrans * mass_center_base;

  ///////////////////////////////////////////////////////////

  // rotate y90 so x-axis is facing down
  Eigen::Affine3f Y = Eigen::Affine3f(Eigen::AngleAxisf(0.5 * M_PI, Eigen::Vector3f::UnitY()));
  Eigen::Affine3f Z = Eigen::Affine3f(Eigen::AngleAxisf(y_angle, Eigen::Vector3f::UnitZ()));
  Eigen::Affine3f X = Eigen::Affine3f(Eigen::AngleAxisf(0.5 * M_PI, Eigen::Vector3f::UnitX()));
  Eigen::Affine3f RXYZ = transform * Z * Y * X;

  // transform into camera coordinate frame
  Eigen::Affine3f finalTrans = cameraTrans * RXYZ;

  // extract rotation as quaternion
  Eigen::Matrix4f pose_gripper = finalTrans.matrix();
  Eigen::Matrix3f rotation = pose_gripper.block(0, 0, 3, 3);
  Eigen::Quaternionf orientation = Eigen::Quaternionf(rotation);
  orientation.normalize();

  // calculate grasp point using center of mass
  pcl::PointCloud<pcl::PointXYZ>::Ptr
      grasp_points = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
  pcl::PointXYZ point(mass_center_cam[0], mass_center_cam[1], mass_center_cam[2]);
  grasp_points->push_back(point);
  grasp_points->push_back(point);

  // populate return structure
  std::vector <GraspPose> grasps;
  grasps.push_back(GraspPose(grasp_points, orientation));
  return grasps;

  ////////////////////// DEBUG VIZ CODE /////////////////////////////
//  LOG4CXX_WARN(logger, "Debug viz code.");
//
//  //create the pcl viewer
//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Grasp Debug"));
//
//  //add cloud to viewer
//  viewer->addPointCloud<pcl::PointXYZ> (object_cloud, "object_cloud");
////  viewer->addPointCloud<pcl::PointXYZ> (projected_cloud, "projected_cloud");
//
////  pcl::PointXYZ center (point.x, point.y, point.z);
//
////  Eigen::Vector3f y_dir_vec_base(0,1,0);
////  Eigen::Vector3f y_dir_vec = finalTrans * y_dir_vec_base;
////  pcl::PointXYZ min_axis (y_dir_vec(0) + center.x, y_dir_vec(1) + center.y, y_dir_vec(2) + center.z);
////  viewer->addLine (center, min_axis, 1.0f, 0.0f, 0.0f, "y_dir");
//  viewer->addCoordinateSystem(0.1, finalTrans, "grasp", 0);
////
////  pcl::PointCloud<pcl::PointXYZ>::Ptr polygon_cloud(new pcl::PointCloud<pcl::PointXYZ>);
////  polygon_cloud->push_back(pcl::PointXYZ(rrPts[0].x, rrPts[0].y, 0.0));
////  polygon_cloud->push_back(pcl::PointXYZ(rrPts[1].x, rrPts[1].y, 0.0));
////  polygon_cloud->push_back(pcl::PointXYZ(rrPts[2].x, rrPts[2].y, 0.0));
////  polygon_cloud->push_back(pcl::PointXYZ(rrPts[3].x, rrPts[3].y, 0.0));
////  viewer->addPolygon<pcl::PointXYZ>(polygon_cloud, 0.0, 1.0, 0.0, "min bb 2d", 0);
//
//  viewer->initCameraParameters();
//  viewer->addCoordinateSystem(0.1, "grasp", 0);
//  viewer->setPosition(0, 0);
//  viewer->setSize(640, 480);
//
//  //runviewer until closed
//  while (!viewer->wasStopped()) {
//    viewer->spinOnce(100);
//    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//  }
//
//  viewer->close();

  ///////////////////////////////////////////////////////////////////
}
