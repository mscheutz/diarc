/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @file PCLFunctions.hpp
 * @date September 2012
 * @version 0.1
 * @brief Calculations with PCL.
 */

#ifndef PCLFUNCTIONS_HPP
#define PCLFUNCTIONS_HPP

#ifndef EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#endif

#ifndef EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#define EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif

#include <math.h>
// PCL includes
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/pcl_macros.h>

#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/model_outlier_removal.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/filters/crop_hull.h>

#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/kdtree/kdtree_ann.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/grid_projection.h>
//#include <pcl/surface/marching_cubes_greedy.h>
//#include <pcl/surface/marching_cubes_greedy_dot.h>
//#include <pcl/surface/poisson.h>
#include <pcl/surface/concave_hull.h>
//#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <stdio.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>

#include <boost/format.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/lexical_cast.hpp>

// Euclidian modulus
#define MOD(x, m) ((x%m + m)%m)

/**
 * @brief Self-made exceptions for functions using PCL library 
 * @param PCL_PASS_THROUGH_FILTER Exception thrown when pcl::PassThrough fails to filter point cloud.
 * @param PCL_SEGMENT_PLANE Exception is thrown when plane segmentation fails.
 * @param PCL_COMPUTE_NORMALS Exception is thrown when pcl::NormalEstimation fails to compute normals for the point cloud.
 * @param PCL_COMPUTE_CURVATURE Exception is thrown when pcl::CurvatureEstimation fails to compute curvature for the point cloud.
 * @param PCL_CREATE_MESH Exception is thrown when creation of a mesh for the point cloud fails.
 */

namespace pcl_selfmade_exceptions {

  enum PCL_SELFMADE_EXCEPTIONS {
    PCL_PASS_THROUGH_FILTER = 1,
    PCL_SEGMENT_PLANE,
    PCL_COMPUTE_NORMALS,
    PCL_COMPUTE_CURVATURE,
    PCL_CREATE_MESH,
  };

}

//EAK: most of these methods either don't actually need to be templated or aren't
//using the template type like they're supposed to. Also, even the methods
//that don't need to be templated won't compile if they arent ...

/**
 * @brief Compute normals for points in the point cloud 
 * @param cloud Point cloud in PCL format.
 * @param normals Normals of the point cloud (modified).
 * @param ksearch_radius Search radius for pcl::NormalEstimation (default 50).
 * @return Returns true for success.
 */
template<class T>
bool ComputePointNormals(typename pcl::PointCloud<T>::ConstPtr cloud,
                         pcl::PointCloud<pcl::Normal>::Ptr normals,
                         int ksearch_radius = 50) {
  try {
    // Extract point cloud normals
    pcl::NormalEstimation<T, pcl::Normal> ne;
    typename pcl::search::KdTree<T>::Ptr tree(new pcl::search::KdTree<T>());
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setKSearch(ksearch_radius);
    // calculate the min value of the 
    ne.setViewPoint(0.0, 0.0, 0.0);
    ne.compute(*normals);
    if (!normals->size()) {
      PCL_ERROR("[ComputePointNormals] Error while computing normals of the point cloud!\n");
      throw pcl_selfmade_exceptions::PCL_COMPUTE_NORMALS;
    }
  } catch (...) {
    return (false);
  }
  return (true);
}

/**
 * @brief Compute normals for points in the point cloud 
 * @param cloud Point cloud in PCL format.
 * @param indices Indices in the point cloud to be used.
 * @param normals Normals of the point cloud (modified).
 * @param ksearch_radius Search radius for pcl::NormalEstimation (default 50).
 * @return Returns true for success.
 */
template<class T>
bool ComputePointNormals(typename pcl::PointCloud<T>::ConstPtr cloud,
                         pcl::PointIndices::ConstPtr indices,
                         pcl::PointCloud<pcl::Normal>::Ptr normals,
                         int ksearch_radius = 50) {
  try {
    // Extract point cloud normals
    pcl::NormalEstimation<T, pcl::Normal> ne;
    typename pcl::search::KdTree<T>::Ptr tree(new pcl::search::KdTree<T>());
    pcl::PointCloud<pcl::Normal> temp;
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setIndices(indices);
    ne.setKSearch(ksearch_radius);
    // calculate the min value of the 
    ne.setViewPoint(0.0, 0.0, 0.0);
    ne.compute(*normals);

    if (!normals->size()) {
      PCL_ERROR("[ComputePointNormals] Error while computing normals of the point cloud!\n");
      throw pcl_selfmade_exceptions::PCL_COMPUTE_NORMALS;
    }
  } catch (...) {
    return (false);
  }
  return (true);
}

/**
* Erode a convex hull to make it smaller than the original. This is used mainly by the filterPointsOnPlane method.
* @tparam PointT
* @param cloud
* @param indices_plane
* @param coeffs_plane
* @param cloud_hull [modified]
*/
template<class PointT>
void erodeConvexHull(const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
                     const pcl::PointIndices::ConstPtr& indices_plane,
                     const pcl::ModelCoefficients::ConstPtr& coeffs_plane,
                     typename pcl::PointCloud<PointT>::Ptr& cloud_hull) {
  // check that the plane normal is pointing to the camera
  // select the point on the plane and create vector to the camera
  Eigen::Vector3f plane_norm(coeffs_plane->values[0], coeffs_plane->values[1], coeffs_plane->values[2]);
  plane_norm.norm();
  int idx = indices_plane->indices.at(0);
  Eigen::Vector3f plane2camera_vect(-cloud->points.at(idx).x, -cloud->points.at(idx).y, -cloud->points.at(idx).z);
  plane2camera_vect.normalize();
  // if dot product is negative then flip the normal
  float cos_ang = plane_norm.dot(plane2camera_vect);
  if (cos_ang < 0) {
    plane_norm[0] = -plane_norm[0];
    plane_norm[1] = -plane_norm[1];
    plane_norm[2] = -plane_norm[2];
  }

  typename pcl::PointCloud<PointT>::Ptr eroded_hull(new pcl::PointCloud<PointT>);
  Eigen::Vector3f edge_vector, erosion_dir;
  PointT translated_point;

  // Total erosion distance: distance * iterations (here: 5cm)
  static const float erosion_distance = 0.01;
  static const int erosion_iterations = 5;

  for (int iter = 0; iter < erosion_iterations; iter++) {
    eroded_hull->clear();
    eroded_hull->reserve(cloud_hull->width);

    for (int i = 0; i < cloud_hull->width; i++) {

      // Edge for point i is approximated by looking at previous and next neighbors on the hull
      edge_vector = cloud_hull->at(MOD(i + 1, cloud_hull->width)).getVector3fMap()
                    - cloud_hull->at(MOD(i - 1, cloud_hull->width)).getVector3fMap();

      // Vector perpendicular to the edge of the hull, pointing towards the inside of the hull.
      erosion_dir = edge_vector.cross(plane_norm);
      erosion_dir.normalize();

      // Translate point on edge of hull
      translated_point = cloud_hull->at(i);
      translated_point.x -= erosion_distance * erosion_dir[0];
      translated_point.y -= erosion_distance * erosion_dir[1];
      translated_point.z -= erosion_distance * erosion_dir[2];

      // Store point in eroded hull pointcloud
      eroded_hull->push_back(translated_point);
    }

    // Calc convex hull, reduces errors due to edge approximation
    pcl::ConvexHull<PointT> chull;
    chull.setInputCloud(eroded_hull);
    chull.reconstruct(*cloud_hull);
  }
}

/**
* Helper method to visualize the results from the filterPointsOnPlane method
* @tparam PointT
* @param cloud
* @param indices_plane
* @param indices_objects
* @param indices_real_objects
* @param cloud_hull
*/
template<class PointT>
void visualizeResults(const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
                      const pcl::PointIndices::ConstPtr& indices_plane,
                      const pcl::PointIndices::ConstPtr& indices_objects,
                      const pcl::PointIndices::ConstPtr& indices_real_objects,
                      const typename pcl::PointCloud<PointT>::ConstPtr& cloud_hull) {

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloud(new pcl::PointCloud<pcl::PointXYZ>);
  extract.setInputCloud(cloud);
  extract.setIndices(indices_real_objects);
  extract.setNegative(false);
  extract.filter(*objectCloud);

  static boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  if (boost::shared_ptr<pcl::visualization::PCLVisualizer>() == viewer) {
    viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(
            new pcl::visualization::PCLVisualizer("filterPointsOnPlane"));
    viewer->initCameraParameters();
    viewer->addCoordinateSystem(0.1);
    viewer->addPointCloud(cloud, "capture");
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_cloud_hull(cloud_hull, 255, 0, 0);
    //viewer->addPointCloud(cloud_hull, handler_cloud_hull, "chull");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_objects(objectCloud, 0, 255, 0);
    viewer->addPointCloud(objectCloud, handler_objects, "objects");
    for (size_t i = 0; i < cloud_hull->size(); ++i) {
      PointT point = cloud_hull->points[i];
      //viewer->addSphere(point, 0.001, 255, 0, 0, "chull_" + boost::lexical_cast<size_t>(i));

      PointT nextPoint;
      if (i == cloud_hull->size() - 1) {
        nextPoint = cloud_hull->points[0];
      } else {
        nextPoint = cloud_hull->points[i + 1];
      }
      viewer->addLine(point, nextPoint, 255, 0, 0, "chull_line_" + boost::lexical_cast<size_t>(i));
    }
  } else {
    viewer->updatePointCloud(cloud, "capture");
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_cloud_hull(cloud_hull, 255, 0, 0);
    //viewer->updatePointCloud(cloud_hull, handler_cloud_hull, "chull");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_objects(objectCloud, 0, 255, 0);
    viewer->updatePointCloud(objectCloud, handler_objects, "objects");

    // first remove all previous chull spheres
    viewer->removeAllShapes();

    for (size_t i = 0; i < cloud_hull->size(); ++i) {
      PointT point = cloud_hull->points[i];
      //viewer->addSphere(point, 0.001, 255, 0, 0, "chull_" + boost::lexical_cast<size_t>(i));

      PointT nextPoint;
      if (i == cloud_hull->size() - 1) {
        nextPoint = cloud_hull->points[0];
      } else {
        nextPoint = cloud_hull->points[i + 1];
      }
      viewer->addLine(point, nextPoint, 255, 0, 0, "chull_line_" + boost::lexical_cast<size_t>(i));
    }

  }
  while (!viewer->wasStopped()) {
    viewer->spinOnce();
  }
  viewer->resetStoppedFlag();
}

//template <class T>
//bool SplitPlanesAndMesh(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PolygonMesh::Ptr mesh);

//template <class T>
//bool MeshSimplificationWithPlanes(pcl::PolygonMesh::Ptr mesh, pcl::PolygonMesh::Ptr mesh_simplified, pcl::PointIndices::Ptr mesh_indices_to_filter = pcl::PointIndices::Ptr());

/**
 * @brief Filter point cloud using pcl::PassThrough filter
 * @param cloud Point cloud in PCL format.
 * @param cloud_filtered Filtered point cloud (modified).
 * @param dim Dimension to filter (default z)
 * @param minLimit Minimum value in the dimension to be included (default 0.0).
 * @param maxLimit Maximum value in the dimension to be included (default 5.0).
 * @param filteredValue Value to filter in the point cloud (default NaN).
 * @return Returns true for success.
 */
template<class T>
bool FilterPointCloud(typename pcl::PointCloud<T>::ConstPtr cloud,
                      typename pcl::PointCloud<T>::Ptr cloud_filtered,
                      std::string dim = "z", float minLimit = 0.0, float maxLimit = 5.0,
                      float filteredValue = std::numeric_limits<float>::quiet_NaN()) {
  try {
    // Filter PointCloud
    pcl::PassThrough<T> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName(dim.c_str());
    pass.setFilterLimits(minLimit, maxLimit);
    pass.setUserFilterValue(filteredValue);
    pass.filter(*cloud_filtered);

    if (!cloud_filtered->size()) {
      PCL_ERROR("[FilterPointCloud] After filtering not enough data.\n");
      throw pcl_selfmade_exceptions::PCL_PASS_THROUGH_FILTER;
    }
  } catch (...) {
    return (false);
  }
  return (true);
}

/**
 * @brief Filter point cloud using pcl::PassThrough filter
 * @param cloud Point cloud in PCL format.
 * @param indices Indices of the filtered point cloud (modified).
 * @param dim Dimension to filter (default z)
 * @param minLimit Minimum value in the dimension to be included (default 0.0).
 * @param maxLimit Maximum value in the dimension to be included (default 5.0).
 * @param filteredValue Value to filter in the point cloud (default NaN).
 * @return Returns true for success.
 */
template<class T>
bool FilterPointCloud(const typename pcl::PointCloud<T>::ConstPtr& cloud,
                      pcl::PointIndices::Ptr indices,
                      std::string dim = "z", float minLimit = 0.0, float maxLimit = 5.0,
                      float filteredValue = std::numeric_limits<float>::quiet_NaN()) {
  pcl::PointCloud<T> temp;
  try {
    // Filter PointCloud
    pcl::PassThrough<T> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName(dim.c_str());
    pass.setFilterLimits(minLimit, maxLimit);
    // Original point cloud shouldn't be changed
    pass.setKeepOrganized(true);
    pass.setUserFilterValue(filteredValue);

    pass.filter(temp);

    if (!temp.size()) {
      PCL_ERROR("[FilterPointCloud] After filtering not enough data.\n");
      throw pcl_selfmade_exceptions::PCL_PASS_THROUGH_FILTER;
    }
  } catch (...) {
    return (false);
  }

  // @ep: change this copying
  indices->indices.clear();
  for (unsigned int idx = 0; idx < temp.size(); ++idx) {
    if (!std::isnan(temp.points.at(idx).x) && !std::isnan(temp.points.at(idx).y) &&
        !std::isnan(temp.points.at(idx).z)) {
      indices->indices.push_back(idx);
    }
  }
  return (true);
}

// Filter candidate points for object clustering based on plane surface color
//   Current depth data noise and smoothing causes erratic points to be included in object clusters
// TODO: have to find range of plane surface color to filter out (Personally done by using the blob picker in the vision GUI)
// Note: Will not allow objects with the same color as the plane to be detected,
//          Perhaps don't use results of this function if it filters out too many points
template<class T>
bool isValidObjectPoint(const T point,
                        std::array<float,6> colorMaskRanges) {
  if (std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z))
    return false;
  // std::uint32_t rgb = *reinterpret_cast<float*>(&point.rgb);
  // std::uint8_t r = (rgb >> 16) & 0x0000ff;
  // std::uint8_t g = (rgb >> 8)  & 0x0000ff;
  // std::uint8_t b = (rgb)       & 0x0000ff;
  // printf("pcl point xyz: %f,%f,%f\n", point.x, point.y, point.z);
  // printf("pcl point rgb: %d,%d,%d\n", point.r, point.g, point.b);

  return !(point.r> colorMaskRanges[0] && point.r< colorMaskRanges[3] && point.g> colorMaskRanges[1] && point.g< colorMaskRanges[4] && point.b> colorMaskRanges[2] && point.b< colorMaskRanges[5]);
}

/**
 * @brief Segment plane in the point cloud given existing plane coefficients
 * @param cloud Point cloud in PCL format.
 * @param plane_indices Indices of the plane in the point cloud (modified).
 * @param objects_indices Indices of the objects in the point cloud [everything except plane] (modified).
 * @param coefficients Coefficients of the initially detected plane
 * @param dist_threshold SAC Segmentation distance threshold for plane inliers.
 * @param normals Pre-computed cloud normals (if empty they will be calculated in this method).
 * @return Returns true for success.
 */
template<class T>
bool SegmentPlaneGivenCoeffs(const typename pcl::PointCloud<T>::ConstPtr &cloud,
                             pcl::PointIndices::Ptr &plane_indices,
                             pcl::PointIndices::Ptr &objects_indices,
                             pcl::ModelCoefficients::ConstPtr coefficients,
                             const float dist_threshold = 0.01,
                             pcl::PointCloud<pcl::Normal>::Ptr normals = pcl::PointCloud<pcl::Normal>::Ptr()) {
  try {
    if (cloud->size() < 3) {
      fprintf(stderr, "[SegmentPlaneGivenCoeffs] cloud too small. only has %lu points.\n", cloud->size());
      return false;
    }

    // if plane indices aren't already calculated, filter plane indices based on plane coefficients
    if (plane_indices->indices.empty()) {
      //compute normals
      if (!normals) {
        normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>());
        ComputePointNormals<T>(cloud, normals);
      }

      pcl::ModelOutlierRemoval<T> filter;
      filter.setModelCoefficients(*coefficients);
      filter.setThreshold(dist_threshold);
      filter.setModelType(pcl::SACMODEL_NORMAL_PLANE);
      filter.setInputCloud(cloud);
      filter.setInputNormals(normals);
      filter.filter(plane_indices->indices);
    }

    if (plane_indices->indices.empty()) {
      fprintf(stderr, "[SegmentPlaneGivenCoeffs] Not enough points in the plane.");
      return false;
    }

    //EAK: TODO: if we're clustering here, do we really need to do the expensive
    //SAC segmentation above ??
    //cluster plane and keep largest cluster as plane. this helps filter out
    //horizontal surfaces not actually part of desired plane
    typename pcl::search::KdTree<T>::Ptr tree(new pcl::search::KdTree<T>);
    pcl::IndicesPtr tmpPlaneIndices = pcl::IndicesPtr(new std::vector<int>(plane_indices->indices));
    tree->setInputCloud(cloud, tmpPlaneIndices);
    std::vector<pcl::PointIndices> clusters;
    pcl::extractEuclideanClusters<T>(*cloud, *tmpPlaneIndices, tree, 0.05, clusters);
    int largestClusterSize = 0;
    int largestIndex = -1;
    for (int i = 0; i < clusters.size(); ++i) {
      if (clusters[i].indices.size() > largestClusterSize) {
        largestClusterSize = clusters[i].indices.size();
        largestIndex = i;
      }
    }
    plane_indices->indices = clusters[largestIndex].indices;

    if (plane_indices->indices.empty()) {
      fprintf(stderr, "[SegmentPlaneGivenCoeffs] Not enough points in the plane after clustering.");
      return false;
    }

    // Given plane indices find objects indices
    std::vector<bool> used_indices(cloud->size(), false);
    for (int i = 0; i < plane_indices->indices.size(); ++i) {
      int index = plane_indices->indices.at(i);
      used_indices.at(index) = true;
    }


    objects_indices->indices.reserve(cloud->size() - plane_indices->indices.size());
    for (int i = 0; i < used_indices.size(); ++i) {
      if (!used_indices.at(i) &&
          !std::isnan(cloud->points[i].x) && !std::isnan(cloud->points[i].y) && !std::isnan(cloud->points[i].z)) {
        objects_indices->indices.push_back(i);
      }

      // TODO: populate color mask based on color of plane points
      //Experimental: additionally filter out noisy points not considered plane indices but still have surface color
      // Dont add to plane indices, only used so they aren't considered as candidates for object points
//      std::array<float,6> colorMaskRanges;
//      if (!used_indices.at(i) &&
//          isValidObjectPoint(cloud->points[i], colorMaskRanges)) {
//        objects_indices->indices.push_back(i);
//      }
    }

    if (!(objects_indices->indices.size())) {
      fprintf(stderr, "[SegmentPlaneGivenCoeffs] Not enough points outside the plane.");
      return false;
    }

  } catch (...) {
    PCL_ERROR("[SegmentPlaneGivenCoeffs][ERROR] caught exception returning false.\n");
    return (false);
  }
  return (true);
}

/**
 * @brief Segment plane in the point cloud
 * @param cloud Point cloud in PCL format.
 * @param plane_indices Indices of the plane in the point cloud (modified).
 * @param objects_indices Indices of the objects in the point cloud [everything except plane] (modified).
 * @param coefficients Coefficients of the plane (modified).
 * @param transform Transformation matrix (4x4) from plane to camera coordinate systems.
 * @param dist_threshold SAC Segmentation distance threshold for plane inliers.
 * @param table_height Height of the table we are searching for (default 0.7).
 * @param table_height_epsilon Possible error in the distance from the camera to the plane.
 * @param angle_range Possible error in direction of the table plane.
 * @return Returns true for success.
 */
template<class T>
bool SegmentPlane(const typename pcl::PointCloud<T>::ConstPtr& cloud,
                  pcl::PointIndices::Ptr& plane_indices,
                  pcl::PointIndices::Ptr& objects_indices,
                  pcl::ModelCoefficients::Ptr& coefficients,
                  const cv::Mat &transform,
                  const float dist_threshold = 0.01,
                  const float table_height = 0.7,
                  const float table_height_epsilon = 0.1,
                  const float angle_range = M_PI / 6.0) {
  try {
    if (cloud->size() < 3) {
      fprintf(stderr, "[SegmentPlane] cloud too small. only has %lu points.\n", cloud->size());
      return false;
    }
    //plane parameters
    const double *transform_data = transform.ptr<double>(0);

    //(in meters) desired projected (onto plane) distance from sensor
    float desired_dist_from_orig = transform_data[11] - table_height;
    if (transform_data[11] < table_height) {
      std::string msg = boost::str(
          boost::format("[SegmentPlane] Sensor height (%d m) below table height (%d m). Setting to find floors.\n")
              % transform_data[11] % table_height);
      PCL_ERROR(msg.c_str());
      desired_dist_from_orig = transform_data[11];
    }
    // fprintf(stderr, "[SegmentPlane] desired_dist_from_orig: %f\n", desired_dist_from_orig);

    //calc desired plane norm based on transform (4x4)
    // inverse (ie, transpose) rotation matrix from robot base coordinates to the camera coordinates
    Eigen::Matrix3f rot;
    rot << transform_data[0], transform_data[4], transform_data[8],
            transform_data[1], transform_data[5], transform_data[9],
            transform_data[2], transform_data[6], transform_data[10];

    // plane normal in the base coordinate system
    Eigen::Vector3f base_up(0, 0, 1);
    Eigen::Vector3f desired_plane_norm = rot * base_up;
    desired_plane_norm.normalize();
    // fprintf(stderr, "[SegmentPlane] desired_plane_norm: %f %f %f\n", desired_plane_norm[0], desired_plane_norm[1], desired_plane_norm[2]);

    //compute normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    ComputePointNormals<T>(cloud, normals);

    //filter cloud based on a specified normal and specified height. 
    //functionality doesn't exist in PCL
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    Eigen::Vector3f point_norm;
    pcl::PointXYZ origin(0, 0, 0);
    float angle, d, dist_from_orig;
    for (int i = 0; i < normals->points.size(); ++i) {
      point_norm[0] = normals->points.at(i).normal_x;
      point_norm[1] = normals->points.at(i).normal_y;
      point_norm[2] = normals->points.at(i).normal_z;
      point_norm.normalize();
      angle = std::acos(point_norm.dot(desired_plane_norm));
      // printf("[SegmentPlane] index: %d angle: %f\n", i, angle);
      if ((angle < angle_range) || (angle > M_PI - angle_range)) {
        //filter for height
        //        d = -(cloud->points[i].x * point_norm[0] + cloud->points[i].y * point_norm[1] + cloud->points[i].z * point_norm[2]);
        //        dist_from_orig = pcl::pointToPlaneDistance<pcl::PointXYZ > (origin, point_norm[0], point_norm[1], point_norm[2], d);

        dist_from_orig = -(cloud->points[i].x * desired_plane_norm[0] + cloud->points[i].y * desired_plane_norm[1] +
                           cloud->points[i].z * desired_plane_norm[2]);
        // fprintf(stderr, "[SegmentPlane] dist from origin: %f m\n", dist_from_orig);
        if (dist_from_orig < desired_dist_from_orig + table_height_epsilon &&
            dist_from_orig > desired_dist_from_orig - table_height_epsilon) {
          //fprintf(stderr, "[SegmentPlane] point correct distance to origin: %f m\n", dist_from_orig);
          inliers->indices.push_back(i);
        }
      }
    }

    if (inliers->indices.size() < 50) { //no real reason for 50 - just seemed "big enough"
      std::cerr << "[SegmentPlane][WARNING]: Not enough inliers in the plane!" << std::endl;
      return false;
    }

    // Segment Plane
    // Create the segmentation object
    pcl::SACSegmentationFromNormals<T, pcl::Normal> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(
            pcl::SACMODEL_NORMAL_PLANE); //SACMODEL_PERPENDICULAR_PLANE);//SACMODEL_NORMAL_PLANE);SACMODEL_PLANE);
    seg.setNormalDistanceWeight(0.0); //0.1
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(dist_threshold); //0.04 //0.01
    seg.setInputCloud(cloud);
    seg.setInputNormals(normals);
    seg.setIndices(inliers);
    seg.segment(*plane_indices, *coefficients);

    //check normal (we want the normal facing towards the camera and there's no guarantee of it's direction)
    Eigen::Vector3f plane_norm(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    plane_norm.normalize();
    float cos_ang = plane_norm.dot(desired_plane_norm);
    float ang_btwn = std::acos(cos_ang);
    //printf("[SegmentPlane] plane norm: %f %f %f angle: %f\n", plane_norm[0], plane_norm[1], plane_norm[2], ang_btwn * (180.0 / M_PI));

    //if norm is facing wrong way, flip it
    if (ang_btwn > (M_PI - angle_range)) {
      //printf("[SegmentPlane] wrong normal direction...flipping.\n\n");
      coefficients->values[0] = -coefficients->values[0];
      coefficients->values[1] = -coefficients->values[1];
      coefficients->values[2] = -coefficients->values[2];
      coefficients->values[3] = -coefficients->values[3];
    } else if (ang_btwn > angle_range) {
      PCL_ERROR("[SegmentPlane][ERROR] Wrong normal!\n");
      throw pcl_selfmade_exceptions::PCL_SEGMENT_PLANE;
    }

    return SegmentPlaneGivenCoeffs<T>(cloud, plane_indices, objects_indices, coefficients, dist_threshold, normals);

  } catch (...) {
    PCL_ERROR("[SegmentPlane][ERROR] caught exception returning false.\n");
    return (false);
  }
}

/**
 * @brief Segment plane in the point cloud
 * @param cloud Point cloud in PCL format.
 * @param indices Indices to be used in the point cloud.
 * @param plane_indices Indices of the plane in the point cloud (modified).
 * @param objects_indices Indices of the objects in the point cloud [everything except plane] (modified).
 * @param coefficients Coefficients of the plane (modified).
 * @param distanceThreshold Distance threshold for the RANSAC algorithm (default 0.01).
 * @return Returns true for success.
 */
template<class T>
bool SegmentPlane(const typename pcl::PointCloud<T>::ConstPtr& cloud,
                  const pcl::PointIndices::ConstPtr& indices,
                  pcl::PointIndices::Ptr& plane_indices,
                  pcl::PointIndices::Ptr& objects_indices,
                  pcl::ModelCoefficients::Ptr& coefficients,
                  float distanceThreshold = 0.01) {
  try {
    // Segment Plane
    // Create the segmentation object
    pcl::SACSegmentation<T> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud);
    seg.setIndices(indices);
    seg.segment(*plane_indices, *coefficients);

    if (!plane_indices->indices.size()) {
      PCL_ERROR("[SegmentPlane][ERROR] Not enough points in the plane.\n");
      throw 1;
    }

    // Extract Objects Indices
    std::vector<int> set;
    set.resize(cloud->size());

    for (unsigned int idx = 0; idx < set.size(); ++idx) {
      set.at(idx) = 0;
    }

    for (unsigned int idx = 0; idx < plane_indices->indices.size(); ++idx) {
      set.at(plane_indices->indices.at(idx)) = 1;
    }

    objects_indices->indices.clear();
    for (unsigned int idx = 0; idx < indices->indices.size(); ++idx) {
      if (set.at(indices->indices.at(idx)) == 0) {
        objects_indices->indices.push_back(indices->indices.at(idx));
        set.at(indices->indices.at(idx)) == 1;
      }
    }

  } catch (...) {
    return (false);
  }
  return (true);
}

/**
 * @brief Segment plane in the point cloud using normals
 * @param cloud Point cloud in PCL format.
 * @param indices Indices to be used in the point cloud.
 * @param plane_indices Indices of the plane in the point cloud (modified).
 * @param objects_indices Indices of the objects in the point cloud [everything except plane] (modified).
 * @param coefficients Coefficients of the plane (modified).
 * @param distanceThreshold Distance threshold for the RANSAC algorithm (default 0.01).
 * @param maxIterations Maximum number of iterations for RANSAC (default 100).
 * @param normalDistanceWeight Weight of the distance for normals (default 0.05).
 * @param angle_range Range in which normals are considered to be collinear.
 * @return Returns true for success.
 */
template<class PointT>
bool SegmentPlane2(const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
                   const pcl::PointIndices::ConstPtr& indices,
                   pcl::PointIndices::Ptr& plane_indices,
                   pcl::PointIndices::Ptr& objects_indices,
                   pcl::ModelCoefficients::Ptr& coefficients,
                   float distanceThreshold = 0.01,
                   int maxIterations = 100,
                   float normalDistanceWeight = 0.05,
                   float angle_range = M_PI / 6.0
) {
  try {
    // extract filtered point cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    typename pcl::PointCloud<PointT>::Ptr cloud_new(new typename pcl::PointCloud<PointT>);
    extract.filter(*cloud_new);

    //compute normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    ComputePointNormals<PointT>(cloud_new, normals);

    // Segment Plane
    // Create the segmentation object
    typename pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    //typename pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    seg.setEpsAngle(angle_range);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(normalDistanceWeight);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud_new);
    seg.setInputNormals(normals);
    // Get results
    seg.segment(*plane_indices, *coefficients);

    if (!plane_indices->indices.size()) {
      PCL_ERROR("[SegmentPlane][ERROR] Not enough points in the plane.\n");
      throw pcl_selfmade_exceptions::PCL_SEGMENT_PLANE;
    }

    std::vector<bool> used_indices(indices->indices.size(), false);
    for (int i = 0; i < plane_indices->indices.size(); ++i) {
      int index = plane_indices->indices.at(i);
      used_indices.at(index) = true;
      plane_indices->indices.at(i) = indices->indices.at(index);
    }

    objects_indices->indices.reserve(indices->indices.size() - plane_indices->indices.size());
    for (int i = 0; i < used_indices.size(); ++i) {
      if (!used_indices.at(i)) {
        int index = indices->indices.at(i);
        used_indices.at(i) = true;
        objects_indices->indices.push_back(index);
      }
    }

    if (!(objects_indices->indices.size())) {
      PCL_ERROR("[SegmentPlane][ERROR] Not enough points outside the plane.\n");
      throw pcl_selfmade_exceptions::PCL_SEGMENT_PLANE;
    }

  } catch (...) {
    return (false);
  }
  return (true);
}

/**
 * @brief Filter out points that are not on the plane
 * @param cloud Point cloud in PCL format.
 * @param indices_plane Indices of the plane.
 * @param indices_objects Indices of objects.
 * @param coeffs_plane Coefficients of the plane.
 * @param indices_real_objects Indices of objects strictly on the plane (modified).
 * @param height Maximum height of the objects (default 0.35 m).
 * @return Returns true for success.
 */
template<class PointT>
bool filterPointsOnPlane(const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
                         const pcl::PointIndices::ConstPtr& indices_plane,
                         const pcl::PointIndices::ConstPtr& indices_objects,
                         const pcl::ModelCoefficients::ConstPtr& coeffs_plane,
                         pcl::PointIndices::Ptr& indices_real_objects,
                         float height = 0.35) {
  //TODO: modify this method to use pcl::CropHull
  try {
    if (indices_plane->indices.size() < 3) {
      PCL_ERROR("[filterPointsOnPlane][ERROR] Not enough points in cloud plane!\n");
      return false;
    }

    // Project the plane inliers onto plane to make finding the convex hull easier
    typename pcl::PointCloud<PointT>::Ptr plane_projected(new pcl::PointCloud<PointT>());
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud);
    proj.setIndices(indices_plane);
    proj.setModelCoefficients(coeffs_plane);
    proj.filter(*plane_projected);


    // EW: ^ doesn't seem to do anything - at least in sim demo case
    // std::string msg = boost::str(
    //           boost::format("[filterPointsOnPlane] midoint projected plane indices size: %d.") % plane_projected->size());
    // PCL_ERROR(msg.c_str());

    //calc convex hull 
    typename pcl::PointCloud<PointT>::Ptr cloud_hull(new pcl::PointCloud<PointT>);
    pcl::ConvexHull<PointT> chull;
    // chull.setDimension (2); // not necessarily needed, but we need to check the dimensionality of the output
    chull.setInputCloud(plane_projected);
    chull.reconstruct(*cloud_hull);
    if (chull.getDimension() != 2) {
      PCL_ERROR("[filterPointsOnPlane] Convex hull not of dimension 2.\n");
      return false;
    }

    // EW: Doesn't seem to work for walls behind table, not sure about walls up against table
    // Erosion on convex hull to avoid having walls next to table detected as objects
    erodeConvexHull<PointT>(cloud, indices_plane, coeffs_plane, cloud_hull);

    // TODO: this fix may not be needed it future versions of PCL (currently needed in released 1.7 version)
    // this is a potential fix for chull being required to be explicitly closed (cloud_hull[first] == cloud_hull[last])
    cloud_hull->push_back(cloud_hull->points[0]);

    //Find objects strictly on plane (inside)
    pcl::ExtractPolygonalPrismData<PointT> prism;
    prism.setInputCloud(cloud);
    prism.setIndices(indices_objects);
    prism.setInputPlanarHull(cloud_hull);
    prism.setHeightLimits(0, height);
    prism.segment(*indices_real_objects);

    // NOTE: uncomment this to view results -- this does not play nice with other displays as it doesn't
    // run on vision's display thread.
    // visualizeResults<PointT>(cloud, indices_plane, indices_objects, indices_real_objects, cloud_hull);

  } catch (...) {
    return (false);
  }
  return (true);
}

///**
// * @brief Calculate area of the polygon
// * @param cloud Point cloud in PCL format.
// * @param cloud_filtered Filtered point cloud (modified).
// * @param dim Dimension to filter (default z)
// * @param minLimit Minimum value in the dimension to be included (default 0.0).
// * @param maxLimit Maximum value in the dimension to be included (default 3.0).
// * @param filteredValue Value to filter in the point cloud (default NaN).
// * @return Returns plygon area.
// */
//template<class PointT>
//double polygonArea(typename pcl::PointCloud<PointT>::ConstPtr points,
//                   const std::vector<pcl::Vertices> &polygons = std::vector<pcl::Vertices>()) {
//  if (points->points.size() == 0) {
//    return 0;
//  }
//
//  if (polygons.size() == 0) {
//    double area = 0; // Accumulates area in the loop
//    int numPoints = points->points.size();
//    int j = numPoints - 1; // The last vertex is the 'previous' one to the first
//
//    for (int i = 0; i < numPoints; i++) {
//      PointT point_i = points->points[i];
//      PointT point_j = points->points[j];
//      area += (point_j.x + point_i.x) * (point_j.y - point_i.y);
//      j = i; //j is previous vertex to i
//    }
//    return std::abs(area / 2.0);
//  } else {
//    std::vector<pcl::Vertices>::const_iterator poly_itr;
//    double total_area = 0;
//    double poly_area = 0;
//    int count = 0;
//    for (poly_itr = polygons.begin(); poly_itr != polygons.end(); ++poly_itr) {
//      const std::vector<uint32_t> &vertices = (*poly_itr).vertices;
//      int numPoints = vertices.size();
//      int j = numPoints - 1; // The last vertex is the 'previous' one to the first
//
//      for (int i = 0; i < numPoints; i++) {
//        PointT point_i = points->points[vertices[i]];
//        PointT point_j = points->points[vertices[j]];
//        poly_area += (point_j.x + point_i.x) * (point_j.y - point_i.y);
//        j = i; //j is previous vertex to i
//      }
//      //printf("%d poly area: %f\n", count++, poly_area);
//      total_area = std::abs(poly_area);
//    }
//    return std::abs(total_area / 2.0);
//  }
//}


//*****************************************************************************
//                             EXPERIMENTAL
//*****************************************************************************

/**
 * Compute mesh for input point cloud. Returns polygons that consist of indices
 * into input point cloud.
 */
template<class PointT>
bool ComputePolygonMesh(typename pcl::PointCloud<PointT>::ConstPtr cloud, std::vector<pcl::Vertices> &polygons) {
  try {

    pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals(new pcl::PointCloud<pcl::PointNormal>);

    //kdtree
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    //Normal estimation
    pcl::NormalEstimation<PointT, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);

    //Concatenate the XYZ and normal fields
    pcl::concatenateFields(*cloud, *normals, *points_with_normals);

    // Greedy Meshing /////////////////////////////////////////////////////
    // Create search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(points_with_normals);
    // Initialize objects
    //pcl::PolygonMesh::Ptr mesh_dense(new pcl::PolygonMesh);
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(0.2); //0.025);

    // Set typical values for the parameters
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18); // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
    gp3.setNormalConsistency(false);
    // Get result
    gp3.setInputCloud(points_with_normals);
    gp3.setSearchMethod(tree2);

    gp3.reconstruct(polygons);

    //attempt to simply mesh by replacing dense planes with convex hull 
    //MeshSimplificationWithPlanes<PointT > (mesh_dense, mesh);
    //printf("final mesh vertices: %d polygons: %d\n\n\n\n\n\n\n\n\n\n\n", mesh->cloud.data.size() / 16, mesh->polygons.size());

  } catch (...) {
    return (false);
  }
  return (true);
}
/*
template <class T>
bool MeshSimplificationWithPlanes(pcl::PolygonMesh::Ptr mesh, pcl::PolygonMesh::Ptr mesh_simplified, pcl::PointIndices::Ptr mesh_indices_to_filter = pcl::PointIndices::Ptr()) {
    // Get mesh clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_points(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(mesh->cloud, *mesh_points);

    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_simplified_points(new pcl::PointCloud<pcl::PointXYZ>);
    if (mesh_simplified->cloud.data.size() > 0) {
        pcl::fromROSMsg(mesh_simplified->cloud, *mesh_simplified_points);
    }


    // Set mesh indices to include all mesh points if not passed in
    if (pcl::PointIndices::Ptr() == mesh_indices_to_filter) {
        //printf("mesh_indices_to_filter init\n");
        mesh_indices_to_filter = pcl::PointIndices::Ptr(new pcl::PointIndices);
        mesh_indices_to_filter->indices.reserve(mesh_points->points.size());
        for (int i = 0; i < mesh_points->points.size(); ++i) {
            mesh_indices_to_filter->indices.push_back(i);
        }
    }
    //printf("mesh_points size: %d\n", mesh_points->points.size());
    //printf("mesh_indices_to_filter size: %d\n", mesh_indices_to_filter->indices.size());

    // Find plane
    // Compute normals
    pcl::PointCloud<pcl::Normal>::Ptr mesh_normals(new pcl::PointCloud<pcl::Normal > ());
    ComputePointNormals<pcl::PointXYZ > (mesh_points, mesh_normals);

    // Segment plane
    // Create the segmentation object
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    seg.setEpsAngle(M_PI / 6.0);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.05); //0.02); //0.1
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.01); //0.01
    seg.setInputCloud(mesh_points);
    seg.setInputNormals(mesh_normals);
    seg.setIndices(mesh_indices_to_filter);
    // Get results
    pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    seg.segment(*plane_inliers, *coefficients);


    //recursion termination
    if (plane_inliers->indices.size() < 3) {
        //end recursion
        //add remaining un-pruned polygons to simplified mesh ///////////////////////////////////
        //change indices to point to appended plane points
        std::vector<pcl::Vertices>::iterator poly_itr;
        std::vector< uint32_t >::iterator vert_itr;
        uint32_t index_increase = mesh_simplified_points->points.size();
        for (poly_itr = mesh->polygons.begin(); poly_itr != mesh->polygons.end(); ++poly_itr) {
            for (vert_itr = (*poly_itr).vertices.begin(); vert_itr != (*poly_itr).vertices.end(); ++vert_itr) {
                (*vert_itr) += index_increase;
            }
        }
        mesh_simplified->polygons.insert(mesh_simplified->polygons.end(), mesh->polygons.begin(), mesh->polygons.end());

        //append plane points to simplified mesh
 *mesh_simplified_points += *mesh_points;
        pcl::toROSMsg(*mesh_simplified_points, mesh_simplified->cloud);

        //TODO: remove unused points from cloud and update polygons accordingly

        return true;
    }

    //project plane inlier points onto plane (maintaining point cloud ordering/indices)
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setCopyAllData(true);
    proj.setInputCloud(mesh_points);
    proj.setIndices(plane_inliers);
    proj.setModelCoefficients(coefficients);
    proj.filter(*mesh_points);

    // extract plane inlier points bc chull.setIndices doesn't appear to be working (pcl 1.5)
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_points(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(mesh_points);
    extract.setIndices(plane_inliers);
    extract.setNegative(false);
    extract.filter(*plane_points);

    //find polygon(s) representing plane
    //convex hull
    pcl::PolygonMesh::Ptr plane_convex_mesh(new pcl::PolygonMesh());
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_convex_points(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::Vertices> plane_convex_polygons;
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setDimension(2);
    chull.setInputCloud(plane_points);
    //chull.setIndices(plane_inliers);            //DOESNT SEEM TO WORK (in 1.5)
    chull.reconstruct(*plane_convex_mesh);
    pcl::fromROSMsg(plane_convex_mesh->cloud, *plane_convex_points);
    plane_convex_polygons = plane_convex_mesh->polygons;

    //concave hull
    pcl::PolygonMesh::Ptr plane_concave_mesh(new pcl::PolygonMesh());
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_concave_points(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::Vertices> plane_concave_polygons;
    pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
    concave_hull.setDimension(2); //PCL 1.6+
    concave_hull.setAlpha(0.01);
    concave_hull.setInputCloud(plane_points);
    //chull.setIndices(plane_inliers);            //DOESNT SEEM TO WORK (in 1.5)
    concave_hull.reconstruct(*plane_concave_mesh);
    pcl::fromROSMsg(plane_concave_mesh->cloud, *plane_concave_points);
    plane_concave_polygons = plane_concave_mesh->polygons;


    //    printf("plane_inliers size: %d\n", plane_inliers->indices.size());
    //    printf("plane_convex_polygons num: %d first size: %d\n", plane_convex_polygons.size(), plane_convex_polygons[0].vertices.size());
    //    printf("plane_polygon first: \n");
    //    for (int i = 0; i < plane_convex_polygons[0].vertices.size(); ++i) {
    //        printf("%d ", plane_convex_polygons[0].vertices[i]);
    //    }
    //    printf("\n");

    //if plane doesn't meet fitness criteria, don't prune polygons belonging to this plane
    //TODO: find a general way to find good fitness threshold
    double convex_area = polygonArea<pcl::PointXYZ > (plane_convex_points);
    double concave_area = polygonArea<pcl::PointXYZ > (plane_concave_points, plane_concave_polygons);

    
//        printf("concave indices: \n");
//        std::vector<pcl::Vertices>::iterator poly_itr;
//        std::vector< uint32_t >::iterator vert_itr;
//        int count = 0;
//        for (poly_itr = plane_concave_polygons.begin(); poly_itr != plane_concave_polygons.end(); ++poly_itr) {
//            printf("poly %d: ", count++);
//            for (vert_itr = (*poly_itr).vertices.begin(); vert_itr != (*poly_itr).vertices.end(); ++vert_itr) {
//                printf(" %d", (*vert_itr));
//            }
//            printf("\n");
//        }
//        printf("convex area: %f num points: %d\n", convex_area, plane_convex_points->points.size());
//        printf("concave area: %f num points: %d dims: %d\n", concave_area, plane_concave_points->points.size(), concave_hull.getDim());
     

    double plane_fitness = (convex_area - concave_area) / convex_area;
    //double plane_fitness = plane_inliers->indices.size() / convex_area;
    double plane_fitness_thresh = 0.4; //30000.0; //Kinect: 42000.0;//SR: 2000.0;
    //printf("plane_fitness: %f\n\n", plane_fitness);

    ////////TEMP DEBUG DISPLAY///////////////////////////////////////
    if (false) {
        static boost::shared_ptr<pcl::visualization::PCLVisualizer > viewer;
        if (boost::shared_ptr<pcl::visualization::PCLVisualizer > () == viewer) {
            viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer > (new pcl::visualization::PCLVisualizer("Mesh Simplification"));
            viewer->initCameraParameters();
            viewer->addCoordinateSystem(0.1);
        }

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_mesh_points(mesh_points, 0, 255, 0);
        viewer->addPointCloud(mesh_points, handler_mesh_points, "mesh_points");
        while (!viewer->wasStopped()) {
            viewer->spinOnce();
        }

        viewer->resetStoppedFlag();

        viewer->addPolygonMesh(*plane_convex_mesh, "convex");
        while (!viewer->wasStopped()) {
            viewer->spinOnce();
        }
        viewer->resetStoppedFlag();
        viewer->removePolygonMesh("convex");

        viewer->addPolylineFromPolygonMesh(*plane_concave_mesh, "concave");
        while (!viewer->wasStopped()) {
            viewer->spinOnce();
        }
        viewer->resetStoppedFlag();

        viewer->removeShape("concave");
        viewer->removePointCloud("mesh_points");
    }
    ///////////////////////////////////////////////////////////////////////////

    //if convex hull fitness is good enough, use it, else use concave hull
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_hull_points(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::Vertices> plane_hull_polygons;
    if (plane_fitness < plane_fitness_thresh) {
        plane_hull_points = plane_convex_points;
        plane_hull_polygons = plane_convex_polygons;
        //    } else {
        //        plane_hull_points = plane_concave_points;
        //        plane_hull_polygons = plane_concave_polygons;
        //    }

        //iterate through polygons:
        //(1) removing polygons wholly contained in plane
        //(2) find polygons that overlap plane, add inliers to construct plane's polygon
        std::vector<pcl::Vertices> polygons_to_keep;
        pcl::Vertices curr_polygon;
        pcl::Vertices curr_polygon_inliers;
        //pcl::Vertices plane_polygon;
        for (int p = 0; p < mesh->polygons.size(); ++p) {
            curr_polygon = mesh->polygons[p];
            curr_polygon_inliers.vertices.clear();

            //find curr_polygon's plane inliers
            for (int i = 0; i < plane_inliers->indices.size(); ++i) {
                for (int poly_i = 0; poly_i < curr_polygon.vertices.size(); ++poly_i) {
                    if (plane_inliers->indices[i] == curr_polygon.vertices[poly_i]) {
                        curr_polygon_inliers.vertices.push_back(curr_polygon.vertices[poly_i]);
                    }
                }
            }

            //if polygon contains no inliers - keep it
            if (curr_polygon_inliers.vertices.size() == 0) {
                polygons_to_keep.push_back(curr_polygon);
            } else if (curr_polygon_inliers.vertices.size() < curr_polygon.vertices.size()) {
                //else if polygon contains some inliers -> keep it and add inliers to plane polygon
                polygons_to_keep.push_back(curr_polygon);
                //TODO: don't add duplicates
                //plane_polygon.vertices.insert(plane_polygon.vertices.end(), curr_polygon_inliers.vertices.begin(), curr_polygon_inliers.vertices.end());
            }
            //else polygon contains only inliers to plane -> prune it (ie, do nothing)
        }

        //add plane polygons to simplified mesh
        //change indices to point to appended plane points
        std::vector<pcl::Vertices>::iterator poly_itr;
        std::vector< uint32_t >::iterator vert_itr;
        uint32_t index_increase = mesh_simplified_points->points.size();
        for (poly_itr = plane_hull_polygons.begin(); poly_itr != plane_hull_polygons.end(); ++poly_itr) {
            for (vert_itr = (*poly_itr).vertices.begin(); vert_itr != (*poly_itr).vertices.end(); ++vert_itr) {
                (*vert_itr) += index_increase;
            }
        }
        mesh_simplified->polygons.insert(mesh_simplified->polygons.end(), plane_hull_polygons.begin(), plane_hull_polygons.end());

        //append plane points to simplified mesh
 *mesh_simplified_points += *plane_hull_points;
        pcl::toROSMsg(*mesh_simplified_points, mesh_simplified->cloud);


        //update mesh by pruning polygons
        pcl::toROSMsg(*mesh_points, mesh->cloud);
        mesh->polygons = polygons_to_keep;
    } //if hull_fitness

    //update the remaining mesh points to filter
    //mesh_indices_to_filter - plane_inliers;
    pcl::PointIndices::Ptr updated_mesh_indices_to_filter(new pcl::PointIndices());
    bool in_both = false;
    for (int i = 0; i < mesh_indices_to_filter->indices.size(); ++i) {
        in_both = false;
        for (int j = 0; j < plane_inliers->indices.size(); ++j) {
            if (plane_inliers->indices[j] == mesh_indices_to_filter->indices[i]) {
                in_both = true;
                break;
            }
        }

        if (!in_both) {
            updated_mesh_indices_to_filter->indices.push_back(mesh_indices_to_filter->indices[i]);
        }
    }

    //recurse on remaining mesh points
    if (mesh_indices_to_filter->indices.size() > 400) {
        MeshSimplificationWithPlanes<pcl::PointXYZ > (mesh, mesh_simplified, updated_mesh_indices_to_filter);
    } else {
        //end recursion
        //add remaining un-pruned polygons to simplified mesh ///////////////////////////////////
        //change indices to point to appended plane points
        std::vector<pcl::Vertices>::iterator poly_itr;
        std::vector< uint32_t >::iterator vert_itr;
        uint32_t index_increase = mesh_simplified_points->points.size();
        for (poly_itr = mesh->polygons.begin(); poly_itr != mesh->polygons.end(); ++poly_itr) {
            for (vert_itr = (*poly_itr).vertices.begin(); vert_itr != (*poly_itr).vertices.end(); ++vert_itr) {
                (*vert_itr) += index_increase;
            }
        }
        mesh_simplified->polygons.insert(mesh_simplified->polygons.end(), mesh->polygons.begin(), mesh->polygons.end());

        //append plane points to simplified mesh
 *mesh_simplified_points += *mesh_points;
        pcl::toROSMsg(*mesh_simplified_points, mesh_simplified->cloud);


        //TODO: remove unused points from cloud and update polygons accordingly

        return true;
    }

}

template <class T>
bool SplitPlanesAndMesh(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PolygonMesh::Ptr mesh) {
    try {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_plane(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        // Segment Plane
        //ep uncomment -- SegmentPlane2<pcl::PointXYZ > (cloud, cloud_plane, cloud_without_plane, coefficients);

        //remove outliers
        //        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        //        sor.setInputCloud(cloud_plane);
        //        sor.setMeanK(50);
        //        sor.setStddevMulThresh(1.0);
        //        sor.filter(*cloud_plane);

        // Project the model inliers
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_projected(new pcl::PointCloud<pcl::PointXYZ > ());
        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(cloud_plane);
        proj.setModelCoefficients(coefficients);
        proj.filter(*cloud_plane_projected);

        // Create a concave/convex hull representation of the projected inliers
        pcl::PointCloud<pcl::PointXYZ>::Ptr new_mesh_points(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<pcl::Vertices> new_mesh_polygons;
        pcl::ConvexHull<pcl::PointXYZ> chull;
        chull.setDimension(2);
        chull.setComputeAreaVolume(true);
        //pcl::ConcaveHull<pcl::PointXYZ> chull;
        //chull.setAlpha(100);//0.03);//20);//100.0);
        chull.setInputCloud(cloud_plane_projected);
        chull.reconstruct(*new_mesh_points, new_mesh_polygons);

        //check fit of hull to points
        double plane_fitness = cloud_plane->points.size() / chull.getTotalArea();
        double plane_fitness_thresh = 35000.0;
        printf("plane_fitness: %f\n", plane_fitness);
        if (plane_fitness < plane_fitness_thresh) {
            //scrap the plane and do a greedy mesh
            pcl::PointCloud<pcl::PointXYZ>::Ptr existing_mesh_points(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(mesh->cloud, *existing_mesh_points);
 *existing_mesh_points += *cloud_plane;
            pcl::toROSMsg(*existing_mesh_points, mesh->cloud);

            //            pcl::PolygonMesh::Ptr new_mesh(new pcl::PolygonMesh());
            //            new_mesh_points = cloud_plane;
            //            new_mesh_polygons.clear();
            //
            //            //compute normals
            //            pcl::PointCloud<pcl::Normal>::Ptr new_mesh_normals(new pcl::PointCloud<pcl::Normal > ());
            //            ComputePointNormals<pcl::PointXYZ > (new_mesh_points, new_mesh_normals);
            //
            //            pcl::PointCloud<pcl::PointNormal>::Ptr new_mesh_points_normals(new pcl::PointCloud<pcl::PointNormal > ());
            //            pcl::concatenateFields(*new_mesh_points, *new_mesh_normals, *new_mesh_points_normals);
            //
            //            // Create search tree
            //            pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
            //            tree->setInputCloud(new_mesh_points_normals);
            //
            //            // Greedy Meshing /////////////////////////////////////////////////////
            //            // Initialize objects
            //            pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
            //
            //            // Set the maximum distance between connected points (maximum edge length)
            //            gp3.setSearchRadius(0.035); //0.025);
            //
            //            // Set typical values for the parameters
            //            gp3.setMu(2.5);
            //            gp3.setMaximumNearestNeighbors(100);
            //            gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
            //            gp3.setMinimumAngle(M_PI / 18); // 10 degrees
            //            gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
            //            gp3.setNormalConsistency(false);
            //            // Get result
            //            gp3.setInputCloud(new_mesh_points_normals);
            //            gp3.setSearchMethod(tree);
            //            gp3.reconstruct(*new_mesh);
            //            
            //            new_mesh_polygons = new_mesh->polygons;
        } else {
            pcl::ConcaveHull<pcl::PointXYZ> chull;
            chull.setAlpha(0.03); //20);//100.0);
            chull.setInputCloud(cloud_plane_projected);
            chull.reconstruct(*new_mesh_points, new_mesh_polygons);

            //append new mesh to existing mesh
            //add polygons
            //change indices to point to appended points
            std::vector<pcl::Vertices>::iterator poly_itr;
            std::vector< uint32_t >::iterator vert_itr;
            uint32_t index_increase = mesh->cloud.data.size() / 16; //probably a better way to get number of points in cloud
            for (poly_itr = new_mesh_polygons.begin(); poly_itr != new_mesh_polygons.end(); ++poly_itr) {
                for (vert_itr = (*poly_itr).vertices.begin(); vert_itr != (*poly_itr).vertices.end(); ++vert_itr) {
                    (*vert_itr) += index_increase;
                }
            }
            std::vector<pcl::Vertices>::iterator existing_itr = mesh->polygons.end();
            mesh->polygons.insert(existing_itr, new_mesh_polygons.begin(), new_mesh_polygons.end());
            //add cloud points
            pcl::PointCloud<pcl::PointXYZ>::Ptr existing_hull_points(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(mesh->cloud, *existing_hull_points);
 *existing_hull_points += *new_mesh_points;
            pcl::toROSMsg(*existing_hull_points, mesh->cloud);
            printf("hull siz: %d\n", new_mesh_points->points.size());
            printf("new_hull siz: %d\n", existing_hull_points->points.size());
            printf("mesh cloud siz: %d\n", mesh->cloud.data.size() / 16);
        }


        printf("[SplitPlanesAndMesh] remaining point count: %d\n", cloud_without_plane->size());
        if (cloud_without_plane->size() > 400) {
            //recurse on pruned cloud
            SplitPlanesAndMesh<pcl::PointXYZ > (cloud_without_plane, mesh);
        } else {
            //mesh remaining points with greedy mesh
            pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_points(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(mesh->cloud, *mesh_points);
 *mesh_points += *cloud_without_plane;

            //compute normals
            pcl::PointCloud<pcl::Normal>::Ptr mesh_normals(new pcl::PointCloud<pcl::Normal > ());
            ComputePointNormals<pcl::PointXYZ > (mesh_points, mesh_normals);

            pcl::PointCloud<pcl::PointNormal>::Ptr mesh_points_normals(new pcl::PointCloud<pcl::PointNormal > ());
            pcl::concatenateFields(*mesh_points, *mesh_normals, *mesh_points_normals);

            // Create search tree
            pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
            tree->setInputCloud(mesh_points_normals);

            // Greedy Meshing /////////////////////////////////////////////////////
            // Initialize objects
            pcl::PolygonMesh::Ptr new_polygons(new pcl::PolygonMesh());
            pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

            // Set the maximum distance between connected points (maximum edge length)
            gp3.setSearchRadius(0.15); //0.025);

            // Set typical values for the parameters
            gp3.setMu(2.5);
            gp3.setMaximumNearestNeighbors(100);
            gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
            gp3.setMinimumAngle(M_PI / 18); // 10 degrees
            gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
            gp3.setNormalConsistency(false);
            // Get result
            gp3.setInputCloud(mesh_points_normals);
            gp3.setSearchMethod(tree);
            gp3.reconstruct(*new_polygons);

            //append new mesh to existing mesh
            //add polygons
            mesh->polygons.insert(mesh->polygons.end(), new_polygons->polygons.begin(), new_polygons->polygons.end());
            //add cloud points
            pcl::toROSMsg(*mesh_points, mesh->cloud);
        }

    } catch (...) {
        return (false);
    }
    return (true);
}
 */

#endif //PCLFUNCTION_HPP
