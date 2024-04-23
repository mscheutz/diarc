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

#include "ChangeDetector.hpp"
#include "common/notification/SceneChangeNotification.hpp"
#include "display/Display.hpp"

ChangeDetector::ChangeDetector(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight, const bool isStereo)
: ImageProcessor(processorId, imgWidth, imgHeight, isStereo),
sceneIsChanging(false) {
  visionProcessName = "ChangeDetector";
  logger = log4cxx::Logger::getLogger("ade.imgproc.ChangeDetector");
  bg_model = cv::createBackgroundSubtractorMOG2();
}

void ChangeDetector::notifySceneChange(CaptureData::ConstPtr captureData) {
  Notification::Ptr n(new SceneChangeNotification(shared_from_this(), captureData->frameNumber, captureData, true));
  sendNotifications(n);
}

void ChangeDetector::notifySceneStatic(CaptureData::ConstPtr captureData) {
  Notification::Ptr n(new SceneChangeNotification(shared_from_this(), captureData->frameNumber, captureData, false));
  sendNotifications(n);
}

void ChangeDetector::handleCaptureNotification(CaptureNotification::ConstPtr notification) {  
  const cv::Mat curFrame = notification->captureData->frame;

  if (fgMask.empty())
    fgMask.create(curFrame.size(), curFrame.type());

  // Note: I have not studied the BackgroundSubtractorMOG2 class in sufficient detail,
  // so not sure about the optimal parameter settings. But default parameters with a
  // learning rate of 0.1 during the update stage seem to do what we want: suppress
  // noise while maintaining only a very short-lived background model (a few frames).
  bg_model->apply(curFrame, fgMask, 0.1);

  // if there is significant change in the scene
  if (cv::countNonZero(fgMask) > NUM_PIXELS_CHANGED_THR) {
    if (!sceneIsChanging) {
      notifySceneChange(notification->captureData);
      sceneIsChanging = true;
    }
    // else: scene just keeps changing
  } else {
    if (sceneIsChanging) {
      notifySceneStatic(notification->captureData);
      sceneIsChanging = false;
    }
    // else: scene just keeps static
  }

  if (getDisplayFlag()) {
    ade::Display::displayFrame(fgMask, getDisplayName());
  }

  /* HACK: For the stereo case we assume that if there is motion in the left image,
   * then there is motion in the right image as well, i.e. assume a typical small
   * baseline setup.
  if (is_stereo)
  {
    ...
  }*/
}