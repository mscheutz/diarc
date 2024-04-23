/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include <pcl/point_cloud.h>
#undef USE_UNORDERED_MAP // for compiler error related to opencv and flann conflicts
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/vfh.h>
#include <pcl/features/crh.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/our_cvfh.h>
#include <pcl/features/board.h>
#include <pcl/common/transforms.h>
#include "stm/Grasp.hpp"

template <typename PointType, typename NormalType, typename DescriptorType>
class GlobalFeaturesModel {
  typedef pcl::Histogram<90> CRH90;

public:

  GlobalFeaturesModel()
  : type_("unknown"),
  cloud_(new pcl::PointCloud<PointType> ()),
  cloudRGB_(new pcl::PointCloud<pcl::PointXYZRGB> ()),
  normals_(new pcl::PointCloud<NormalType> ()),
  descriptors_(new pcl::PointCloud<DescriptorType> ()),
  crh_(new pcl::PointCloud<CRH90>),
  centroid_(),
  grasp_options_(),
  cloudHist_(),
  hasRGB(false) {

  }

  ~GlobalFeaturesModel() {

  }

  std::string type_;
  typename pcl::PointCloud<PointType>::Ptr cloud_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB_;
  typename pcl::PointCloud<NormalType>::Ptr normals_;
  typename pcl::PointCloud<DescriptorType>::Ptr descriptors_;
  pcl::PointCloud<CRH90>::Ptr crh_;
  Eigen::Vector4f centroid_;
  std::vector<ade::stm::Grasp> grasp_options_;
  cv::Mat cloudHist_;
  bool hasRGB;
};