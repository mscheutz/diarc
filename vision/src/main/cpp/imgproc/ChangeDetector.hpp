/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * The ChangeDetector processor uses background subtraction to estimate when a
 * scene is undergoing change.
 *
 * @author Michael Zillich (MZ)
 * @date Nov 2012
 */

#ifndef CHANGEDETECTOR_HPP
#define CHANGEDETECTOR_HPP

#include <opencv2/video/background_segm.hpp>
#include "common/notification/CaptureNotification.hpp"
#include "ImageProcessor.hpp"

/**
 * The ChangeDetector processor uses background subtraction to estimate when a
 * scene is undergoing change. It will emit a SceneChangeNotification when
 * change starts and stops (i.e. when the scene is static again).
 *
 * @author Michael Zillich (MZ)
 * @date Nov 2012
 */
class ChangeDetector : public ImageProcessor {
public:
  typedef boost::shared_ptr<ChangeDetector> Ptr;
  typedef boost::shared_ptr<const ChangeDetector> ConstPtr;

  ChangeDetector(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight, const bool isStereo);

protected:
  virtual void handleCaptureNotification(CaptureNotification::ConstPtr notification);
  
private:
  // Number of changed pixels after which we consider the scene to have changed.
  // (and if you are going to have a threshold, it might just as well be diabolic)
  static const int NUM_PIXELS_CHANGED_THR = 666;

  cv::Ptr<cv::BackgroundSubtractorMOG2> bg_model;
  cv::Mat fgMask;
  bool sceneIsChanging;

  void notifySceneChange(CaptureData::ConstPtr captureData);
  void notifySceneStatic(CaptureData::ConstPtr captureData);
};

#endif
