/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef FEATURE_CLOUD_HPP_
#define FEATURE_CLOUD_HPP_

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/features/normal_3d.h"
#include "pcl/features/fpfh.h"
#include <pcl/features/vfh.h>

template<typename EstimatorT, typename FeatureT>
class FeatureCloud
{
public:
  // A bit of shorthand
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
  typedef pcl::PointCloud<FeatureT> LocalFeatures;
  typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

  //using typename LocalFeatures;

  FeatureCloud () :
    search_method_xyz_ (new SearchMethod),
    normal_radius_ (0.02),
    feature_radius_ (0.02)
  {}

  ~FeatureCloud () {}

  // Process the given cloud
  void setInputCloud (PointCloud::Ptr xyz)
  {
    xyz_ = xyz;
    processInput ();
  }

  // Load and process the cloud in the given PCD file
  void loadInputCloud (const std::string & pcd_file)
  {
    xyz_ = PointCloud::Ptr (new PointCloud);
    pcl::io::loadPCDFile (pcd_file, *xyz_);
    processInput ();
  }

  // Get a pointer to the cloud 3D points
  PointCloud::Ptr getPointCloud () const
  {
    return (xyz_);
  }

  // Get a pointer to the cloud of 3D surface normals
  SurfaceNormals::Ptr getSurfaceNormals () const
  {
    return (normals_);
  }

  // Get a pointer to the cloud of feature descriptors
  typename LocalFeatures::Ptr getLocalFeatures () const
  {
    return (features_);
  }

  std::string getType() {
      return type;
  }

  void setType(const std::string& type_) {
      type = type_;
  }

protected:
  // Compute the surface normals and local features
  void processInput ()
  {
    computeSurfaceNormals ();
    computeLocalFeatures ();
  }

  // Compute the surface normals
  void computeSurfaceNormals ()
  {
    normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
    norm_est.setInputCloud (xyz_);
    norm_est.setSearchMethod (search_method_xyz_);
    norm_est.setRadiusSearch (normal_radius_);
    norm_est.compute (*normals_);
  }

  // Compute the local feature descriptors
  void computeLocalFeatures ()
  {
    features_ = typename LocalFeatures::Ptr (new LocalFeatures);

    EstimatorT feature_est;
    feature_est.setInputCloud (xyz_);
    feature_est.setInputNormals (normals_);
    feature_est.setSearchMethod (search_method_xyz_);
    //feature_est.setRadiusSearch (feature_radius_);
    feature_est.compute (*features_);
  }

private:
  // Point cloud data
  PointCloud::Ptr xyz_;
  SurfaceNormals::Ptr normals_;
  typename LocalFeatures::Ptr features_;
  SearchMethod::Ptr search_method_xyz_;

  // Parameters
  float normal_radius_;
  float feature_radius_;
  std::string type; //description of cloud points (e.g., hand, face, stop-gesture, etc...)
};

#endif //#ifndef FEATURE_CLOUD_HPP_
