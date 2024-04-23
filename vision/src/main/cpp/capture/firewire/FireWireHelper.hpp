/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef FIREWIREHELPER_HPP
#define FIREWIREHELPER_HPP

#include <opencv2/opencv.hpp>
#include <dc1394/dc1394.h>
#include <vector>
#include <iostream>
#include <fstream>

//TODO: this class needs to be cleaned up and the camTypes should be removed and
//replaced with additional cammode options.

class FireWireHelper {
public:
  //EAK: WTF is up with these seemingly random numbers???  and why not use enum? 
  static const int INFRARED_CAM = -6;
  static const int BUMBLEBEE_CAM = -7;
  static const int UNIBRAIN_CAM = -8;
  static const int NO_CAM_TYPE = -9;
  typedef int CAM_TYPE;

  FireWireHelper();
  ~FireWireHelper();
  void resetBus();
  void clearCameras();
  bool initializeFWSingle(int option, CAM_TYPE currentCam);
  bool initializeFWMultiple(int n);

  void getStereoSyncFWFrames(cv::Mat sync_f1, cv::Mat sync_f2, cv::Mat disparity_src);
  void getSingleFWFrame(int index, CAM_TYPE currentCam, cv::Mat frameToFill);

  cv::Size getDimensions(int index = 0);

  //currently should only be used for firefly cameras.
  bool initializeFW(int numCams);
  bool getFWFrame(int index, cv::Mat frameToFill);

private:
  void cleanup(dc1394camera_t** cameras, int num);

  //EAK: WTF is up with these seemingly random numbers???  and why not use enum? 
  const static int MAX_CAMERAS = 8;
  const static int MONO_CAM = -5;
  const static int STEREO_SYNC_CAM = -6;

  const static long long BUMBLEBEE_SN = 7040846;
  const static long long UNIBRAIN_SN1 = 40052924;
  const static long long UNIBRAIN_SN2 = 40052630;

  int numCameras;
  dc1394camera_t *cameras [MAX_CAMERAS];
  dc1394video_frame_t *frames [MAX_CAMERAS];
  dc1394_t *d;
  dc1394camera_list_t *list;
  dc1394error_t err;

  dc1394video_mode_t modes[MAX_CAMERAS];
  dc1394color_coding_t codings[MAX_CAMERAS];

  cv::Mat conversionImg;
};

#endif
