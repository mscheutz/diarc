/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

// class to make object clusters detection
// author: ep

#ifndef CLUSTERDETECTOR_HPP
#define CLUSTERDETECTOR_HPP

#include "ObjectDetector.hpp"

#include "common/notification/PlaneNotification.hpp"
#include "common/notification/SaliencyNotification.hpp"
#include "imgproc/PlaneProcessor.hpp"
#include "point_clouds/ExtractedPlane.hpp"
#include "point_clouds/PCLFunctions.hpp"
#include "stm/MemoryObject.hpp"

struct IndexedValueType {
  float value;
  unsigned long int index;

  IndexedValueType(float value_ = 0.0, unsigned long int index_ = 0) : value(value_), index(index_) {
  };
};

enum ClusterDetectorReturnCodes {
  CLUSTERDETECTOR_OK = 0,
  CLUSTERDETECTOR_EMPTY_SALMAP = 1,
  CLUSTERDETECTOR_TIME_EXPIRED = 2,

};

bool IndexedValueTypeSortFunction(IndexedValueType i, IndexedValueType j);

class ClusterDetector : public ObjectDetector {
public:
  typedef boost::shared_ptr<ClusterDetector> Ptr;
  typedef boost::shared_ptr<const ClusterDetector> ConstPtr;

  ClusterDetector(const long long& processorId, const int imgWidth, const int imgHeight);
  ~ClusterDetector();

  //! override because this detector never wants capture notifications (uses plane notifications)
  virtual void registerForCaptureNotification();
  
  //! override because this detector never wants capture notifications (uses plane notifications)
  virtual void unregisterForCaptureNotification();


protected:
  virtual void handleSaliencyNotification(SaliencyNotification::ConstPtr notification);
  virtual void handlePlaneNotification(PlaneNotification::ConstPtr notification);

private:
  void haveNewTablePlane(PlaneNotification::ConstPtr planeNotification);
  void haveNewSaliencyMap(SaliencyNotification::ConstPtr saliencyNotification);
  void sortPointcloud();
  ade::stm::MemoryObject::VecPtr createMemoryObjects(const std::vector<pcl::PointIndices>& clusters,
        CaptureData::ConstPtr captureData);
  void display(ade::stm::MemoryObject::VecPtr newClusterObjects);

  ExtractedPlane::ConstPtr plane;

  double object_cluster_tolerance;
  double object_cluster_min_size;
  cv::Mat salmap;
  std::vector<IndexedValueType> sorted_indices;

  std::vector<cv::Point3f> objectPoints;
  std::vector<cv::Point2f> imagePoints;
};

#endif  //CLUSTERDETECTOR_HPP
