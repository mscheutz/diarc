/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   CaptureEnums.hpp
 * Author: evan
 *
 * Created on December 10, 2013, 12:56 PM
 */

#ifndef CAPTUREENUMS_HPP
#define	CAPTUREENUMS_HPP

namespace ade {
  namespace capture {

    // IMPORTANT: any additions to this enum list must be accompanied by additions to the stringToCAM_MODE method
    // int CaptureUtilities.hpp/cpp
    enum CAM_MODE {
      MONO_FW,
      STEREO_FW,
      STEREO_SYNC,
      MONO_V4L2,
      STEREO_V4L2,
      MONO_AVI,
      MONO_FILE,
      MULTI_FILE_LOG,
      OPENNI, //kinect or xtion using openni
      OPENNI_ONI, //from oni file
      OPENNI2,//kinect or xtion using openni2
      SIM_PC1, //simulate kinect version 1 (rgb and disparity separately)
      SIM_PC2, //simulate kinect version w (pcd files)
      KINECT_PLUS, //kinect plus another rgb camera (mainly for calibration of robot system with multiple sensors)
      DRONE, //ar drone
      ROS,
      NAO,
      GSTREAMER, // H264-encoded stream via GStreamer
      MULTISENSE,
      REALSENSE, // Intel RealSense
      NUM_MODES,
    };

    enum COLOR_SPACE {
      RGB,
      YCrCb,
      Luv,
      BGR,
      NUM_COLOR_SPACES
    };

  } //namespace capture
} //namespace ade

#endif	/* CAPTUREENUMS_HPP */

