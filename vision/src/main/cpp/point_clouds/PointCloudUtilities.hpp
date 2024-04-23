/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef POINTCLOUDS_HPP
#define POINTCLOUDS_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <capture/calibration/Cameras.hpp>
#include <log4cxx/logger.h>

namespace ade {
  namespace pc {
    namespace util {

      void xyzImageToPointCloud(const cv::Mat xyzImage, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
              pcl::PointIndices::Ptr indices);
      void depthToPointCloud(const cv::Mat depth, CameraParameters::ConstPtr irParams,
              pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr indices);
      void depthToPointCloud(const cv::Mat depth, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
              pcl::PointIndices::Ptr indices);
      void depthAndColorToPointCloud(const cv::Mat depth, const cv::Mat color,
              pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud,
              pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr());
      void xyzAndColorToPointCloud(const cv::Mat xyzImage, const cv::Mat color,
              pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud,
              pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr());
      void pointCloudToDepthAndColor(cv::Mat depth, cv::Mat color,
              pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
      //void pointCloudToDepthAndColor(cv::Mat depth, cv::Mat color,
      //        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

      void calculateBoundingBox(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                pcl::PointXYZ& min_point,
                                pcl::PointXYZ& max_point,
                                Eigen::Vector3f& mass_center);
    } //namespace util
  } //namespace pc
} //namespace ade

#endif
