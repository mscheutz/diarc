/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

// class to make object clusters detection
// author: ep

#ifndef CLUSTERDETECTOR2D_HPP
#define CLUSTERDETECTOR2D_HPP

#include "ObjectDetector.hpp"
#include "imgproc/saliency/SaliencyProcessor.hpp"

struct IndexedValueType2D {
  float value;
  int index_r;
  int index_c;
  bool valid;

  IndexedValueType2D(float value_ = 0.0, int index_r_ = 0, int index_c_ = 0, bool valid_ = true) :
  value(value_), index_r(index_r_), index_c(index_c_), valid(valid_) {
  };
};

bool IndexedValueTypeSortFunction2D(IndexedValueType2D i, IndexedValueType2D j);

class ClusterDetector2D : public ObjectDetector {
public:
  typedef boost::shared_ptr<ClusterDetector2D> Ptr;
  typedef boost::shared_ptr<const ClusterDetector2D> ConstPtr;

  ClusterDetector2D(const long long& processorId, const int imgWidth, const int imgHeight);
  ~ClusterDetector2D();

protected:
  virtual void handleCaptureNotification(CaptureNotification::ConstPtr notification);
  virtual void handleSaliencyNotification(SaliencyNotification::ConstPtr notification);

private:
  void haveNewSaliencyMap(SaliencyNotification::ConstPtr saliency);
  void haveNewImage(CaptureNotification::ConstPtr capture);
  void setSortedSaliencyMap(const cv::Mat& salmap);
  void setUniformSaliencyMap();
  //void cluster(const cv::Mat& image, std::vector<std::vector<int> >& clusters,
  //        int min_pts_per_cluster = 20, int max_pts_per_cluster = 100000);
  bool nextCluster(const cv::Mat &image, std::vector<int>& cluster,
          int min_pts_per_cluster = 20, int max_pts_per_cluster = 100000);
  bool isBackgroundPixel(const cv::Mat& image, const int x, const int y);

  bool haveNewSalmap;
  cv::Mat saliencyMap;
  std::vector<IndexedValueType2D> sorted_indices;
  int currSortedIndex;
  std::vector<bool> processed_indices;
  float background_dist;

  bool writeFramesToFile;
};

#endif  //CLUSTERDETECTOR_HPP
