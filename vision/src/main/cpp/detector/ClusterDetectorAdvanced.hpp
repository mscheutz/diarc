/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * Advanced detector for 3D point clusters.
 * Can handle cluttered scenes and does not depend on supporting plane.
 *
 * @author Michael Zillich
 * @date Feb 2013
 */

#ifndef CLUSTERDETECTORADVANCED_HPP
#define CLUSTERDETECTORADVANCED_HPP

#include <vector>
#include <set>
#include <map>
#include <pcl/visualization/pcl_visualizer.h>
#include <v4r/SurfaceSegmenter/segmentation.hpp>
#include "stm/MemoryObject.hpp"
#include "imgproc/ImageProcessor.hpp"
#include "imgproc/PlaneProcessor.hpp"
#include "ObjectDetector.hpp"

class ClusterDetectorAdvanced : public ObjectDetector {
public:
  typedef boost::shared_ptr<ClusterDetectorAdvanced> Ptr;
  typedef boost::shared_ptr<const ClusterDetectorAdvanced> ConstPtr;

  ClusterDetectorAdvanced(const long long& processorId, const int imgWidth, const int imgHeight);
  ~ClusterDetectorAdvanced();

  void display(ade::stm::MemoryObject::VecPtr newObjects);

  virtual void init();
  virtual void cleanup();

protected:
  virtual void handleCaptureNotification(CaptureNotification::ConstPtr notification);
  virtual void handleSaliencyNotification(SaliencyNotification::ConstPtr notification);

private:
  //! the segmenter needs several config files with stored learning parameters, this is the path to these files
  std::string objectModelFilename;
  std::string scalingParamsFilename;
  //! the advanced clutter tolerant segmenter
  segmentation::Segmenter *segmenter;
  //! local copy of most recent saliency map
  cv::Mat salmap;
  //! if any saliency notifications have been received (determines segmentAttention vs normal segment)
  bool saliencyReceived;

  virtual void loadConfig(const std::string& config);

  ade::stm::MemoryObject::VecPtr detectClustersNonincremental(CaptureNotification::ConstPtr capture);
  ade::stm::MemoryObject::VecPtr createMemoryObjects(const std::vector<surface::SurfaceModel::Ptr>& surfs, CaptureNotification::ConstPtr capture);

  //debug only
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr displayCloud;
  void localIterativeDisplay(ade::stm::MemoryObject::VecPtr newObjects);
};

#endif  //CLUSTERDETECTORADVANCED_HPP
