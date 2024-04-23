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

class FireWireHelper {
  
public:

  //EAK: WTF is up with these seemingly random numbers??? and why not use enum?
  static const int INFRARED_CAM = -6;
  static const int BUMBLEBEE_CAM = -7;
  static const int UNIBRAIN_CAM = -8;
  typedef int CAM_TYPE;

  
  FireWireHelper ();
  void clearSingle ();
  void clearMultiple ();
  bool initializeFWSingle (int option, CAM_TYPE currentCam);
  bool initializeFWMultiple (int n);
  
  void getStereoSyncFWFrames (IplImage **sync_f1, IplImage **sync_f2, IplImage **disparity_src);
  IplImage* getSingleFWFrame (int index, CAM_TYPE currentCam);
  IplImage* getSingleRotatedFWFrame (double angle, int index, CAM_TYPE currentCam);
  
  CvSize getDimensions ();
  CvSize getDimensions (int index);

private:

  //EAK: WTF is up with these seemingly random numbers???  and why not use enum? 
  const static int MAX_CAMERAS = 8;
  const static int MONO_CAM = -5;
  const static int STEREO_SYNC_CAM = -6;

  dc1394camera_t *camera;
  dc1394camera_t *cameras [MAX_CAMERAS];
  dc1394video_frame_t *frame;
  dc1394video_frame_t *frames [MAX_CAMERAS];
  dc1394_t *d;
  dc1394camera_list_t *list;
  dc1394error_t err;
  int numCameras;
  
};

#endif
