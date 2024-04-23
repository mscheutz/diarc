/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef SIFTFEATUREEXTRACTOR_HPP
#define SIFTFEATUREEXTRACTOR_HPP

//EAK: do we need all these?
#include <opencv2/opencv.hpp>
extern "C" {
#include "third_party/vlfeat/vl/sift.h"
} //for SIFT - from VLFeat library

#include <common/VisionConstants.hpp>
#include <vector>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>

struct SiftFeature {
typedef boost::shared_ptr<SiftFeature> Ptr;
typedef boost::shared_ptr<const SiftFeature> ConstPtr;

  SiftFeature()
  : descriptor(new vl_sift_pix[128]) {
  }

  float x, y; //subpixel keypoint location
  float sigma; //scale
  float ori; //orientation
  boost::shared_array<vl_sift_pix> descriptor; //Vector of descriptor values

  int getLocation() {
    return static_cast<int> ((y * VisionConstants::imgWidth) + x);
  }
};
typedef std::vector<SiftFeature::Ptr> SiftFeatureVect;
typedef boost::shared_ptr<std::vector<SiftFeature::Ptr> > SiftFeatureVectPtr;

struct SiftFeatures {
  typedef boost::shared_ptr<SiftFeatures> Ptr;
  typedef boost::shared_ptr<const SiftFeatures> ConstPtr;

  SiftFeatures(const unsigned long frameNumber)
  : frameNum(frameNumber),
  vlfeat_keys(new SiftFeatureVect()) {
  }

  ~SiftFeatures() {
  }

  unsigned long frameNum;
  SiftFeatureVectPtr vlfeat_keys;
};

class SiftFeatureExtractor {
public:
  SiftFeatureExtractor(int imgWidth, int imgHeight);
  ~SiftFeatureExtractor();

  SiftFeatures::Ptr calcSiftDescriptors(unsigned long frameNum, const cv::Mat imgSrc, cv::Mat imgDst); //calculates keypoints and descriptors and draws descriptors on imgDst
  SiftFeatures::Ptr calcSiftDescriptors(unsigned long frameNum, const cv::Mat imgSrc); //calculates keypoints and descriptors

  //    SiftFeatureVectPtr getSiftFeatures() const {return imgSiftFeatures; } //returns SiftFeatures for the last img processed by calcSiftDescriptors(), null if no img has beeen processed
  //const std::vector<const VlSiftKeypoint*>* getSiftDescriptors() const {return &imgDescriptors; //returns descriptors for the last img processed by calcSiftDescriptors(), null if no img has beeen processed
  //const std::vector<const vl_sift_pix*>* getKepoints() const { return &imgKeypoints; } //returns keypoints for the last img processed by calcSiftDescriptors(), null if no img has beeen processed

  static SiftFeature::Ptr CheckForMatch(SiftFeature::Ptr key, SiftFeatureVectPtr klist);
  static float GetBestDistance(SiftFeature::Ptr key, SiftFeatureVectPtr klist);
  static void RemoveMatch(SiftFeature::Ptr key, SiftFeatureVectPtr klist);
  static vl_sift_pix DistSquared(SiftFeature::Ptr k1, SiftFeature::Ptr k2);
private:
  void init();
  void cleanup();
  void processCurrentOctave();

  int width;
  int height;
  int numPixels; //width * heightVlSiftFilt* siftFilter;
  unsigned long currFrameNum;

  VlSiftFilt* siftFilter;
  vl_sift_pix* siftImg;
  std::vector<const VlSiftKeypoint*> imgKeypoints;

  cv::Mat grayScaleImg;
  cv::Mat grayScaleImgBlur;
  int m_gauss_kernel; //for image blur
  float m_gauss_dev; //for image blur

  SiftFeatures::Ptr imgSiftFeatures; //data to be (re)filled by calcSiftDescriptors

};

#endif
