/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   ExtractedPlane.hpp
 * Author: evan
 *
 * Created on August 28, 2015, 12:59 PM
 */

#ifndef EXTRACTEDPLANE_HPP
#define	EXTRACTEDPLANE_HPP

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#include <boost/shared_ptr.hpp>

/**
 * structure to save object point cloud, plane point cloud, plane coefficients and 
 * normals of the object
 */
class ExtractedPlane {
public:
  typedef boost::shared_ptr<ExtractedPlane> Ptr;
  typedef boost::shared_ptr<const ExtractedPlane> ConstPtr;

  ExtractedPlane();
  ~ExtractedPlane();
  
  void reset();
  void setFrameNumber(unsigned long int _frameNumber);
  void setTransform(const cv::Mat _transform);
  void setCloudRGB(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr _cloudRGB);
  bool computeFileteredObjectsNormals();
  
  //! const getters
  unsigned long long getFrameNumber() const;
  const cv::Mat getTransform() const;
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr getCloudRGB() const;
  pcl::PointIndices::ConstPtr getPlaneIndices() const;
  pcl::PointIndices::ConstPtr getObjectsIndices() const;
  pcl::PointIndices::ConstPtr getFilteredObjectsIndices() const;
  pcl::ModelCoefficients::ConstPtr getPlaneCoefficients() const;
  pcl::PointCloud<pcl::Normal>::ConstPtr getFileteredObjectsNormals() const;
  
  //! getters than can be used to modify this object
  cv::Mat getTransform();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr& getCloudRGB();
  pcl::PointIndices::Ptr& getPlaneIndices();
  pcl::PointIndices::Ptr& getObjectsIndices();
  pcl::PointIndices::Ptr& getFilteredObjectsIndices();
  pcl::ModelCoefficients::Ptr& getPlaneCoefficients();
  pcl::PointCloud<pcl::Normal>::Ptr& getFileteredObjectsNormals();
  
private:
  cv::Mat transform;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB;
  pcl::PointIndices::Ptr indices_plane;
  pcl::PointIndices::Ptr indices_objects;
  pcl::PointIndices::Ptr indices_filtered_objects;
  pcl::ModelCoefficients::Ptr coefficients;
  pcl::PointCloud<pcl::Normal>::Ptr filtered_objects_normals;
  unsigned long long frameNumber;
  bool fileteredObjectsNormalsComputed;
};

#endif	/* EXTRACTEDPLANE_HPP */

