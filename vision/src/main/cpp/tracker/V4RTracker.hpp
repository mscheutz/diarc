/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef V4RTRACKER_HPP
#define V4RTRACKER_HPP

#include "ObjectTracker.hpp"
#include <v4r/tracking/ObjectTrackerMono.h>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <v4r/common/impl/DataMatrix2D.hpp>
#include <v4r/keypoints/ArticulatedObject.h>
#include <v4r/features/FeatureDetector.h>
#include <v4r/common/ZAdaptiveNormals.h>
#include <opencv2/core/core.hpp>
#include <boost/filesystem.hpp>


/* 
 * V4R object tracker available here: https://github.com/strands-project/v4r
 */

class V4RTracker : public ObjectTracker {
public:

  typedef std::tr1::unordered_map<long long, v4r::ObjectTrackerMono::Ptr> ObjectTrackerMonoPtrMap;

  V4RTracker(const long long &processorId, const int imgWidth, const int imgHeight);

  ~V4RTracker();

  void loadConfig(const std::string &config);

protected:
  virtual void haveNewImage(CaptureNotification::ConstPtr notification);


private:
  virtual void startTracking(const ade::stm::MemoryObject::Ptr &newMemObj);

  virtual void stopTracking(const ade::stm::MemoryObject::Ptr &existingMemObj);

  bool createTrackingModelFromRecognitionModel(const std::string &model_dir,
                                               const std::string &model_name,
                                               v4r::ArticulatedObject::Ptr &model);

  /** readRecognitionStructure **/
  void readRecognitionStructure(const std::string &dir, v4r::ArticulatedObject::Ptr &model,
                                const cv::Mat_<double> &intrinsic = cv::Mat_<double>(),
                                const cv::Mat_<double> &dist_coeffs = cv::Mat_<double>());


  bool
  addObjectView(const v4r::DataMatrix2D <Eigen::Vector3f> &cloud, const v4r::DataMatrix2D <Eigen::Vector3f> &normals,
                const cv::Mat_<unsigned char> &im, const cv::Mat_<unsigned char> &mask,
                const Eigen::Matrix4f &pose, v4r::ArticulatedObject &model);

/**
 * @brief V4RTracker::loadObjectIndices
 * @param _filename
 * @param _mask
 * @param _size
 * @return
 */
  bool loadObjectIndices(const std::string &_filename, cv::Mat_<unsigned char> &_mask, const cv::Size &_size);

  std::string modelDir_;
  std::unordered_map<std::string, std::string> nameMap_;
  ObjectTrackerMonoPtrMap trackers;
  v4r::ObjectTrackerMono::Parameter param_;

  v4r::FeatureDetector::Ptr keyDet;
  v4r::FeatureDetector::Ptr keyDesc;

  cv::Mat_<double> dist_coeffs;// = cv::Mat::zeros(4, 1, CV_64F);
  cv::Mat_<double> intrinsic;
  boost::mutex tracker_mutex;
};

#endif  //V4RTRACKER_HPP
