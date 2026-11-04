/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef OPENNIDEVICE_HPP
#define OPENNIDEVICE_HPP

/**
 * Print various kinect parameters.
 * Michael Zillich, September 2011
 * Ekaterina Potapova, October 2011
 *
 * compile with:
 *   g++ getparameters.cpp -o getparameters `pkg-config --cflags --libs openni-dev`
 */

#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <string>
#include <fstream>
#include <log4cxx/logger.h>
#include <XnOpenNI.h>
#include <XnCppWrapper.h>
#include <opencv2/opencv.hpp>

class OpenniDevice
{
public:
  OpenniDevice(bool liveCapture, const std::string& oniPlaybackFilename = "");
  ~OpenniDevice();
  void CaptureOpenni(cv::Mat &colorImg, cv::Mat &depthImg);
  double getDepthFocalLength() const;
  double getImageFocalLength() const;
  void record(const std::string& oniFilename);
  void stopRecord();
  
  /**
   * A way to share the same context with OpenNI PeoplDetector. This is
   * necessary when running vision from an oni file so that capture and people
   * detection are synchronized, and not using two contexts that run their
   * own oni file independently.
   * @return 
   */
  static xn::Context getContext() {
    return context;
  }
private:
  XnStatus status;
  static xn::Context context;
  xn::DepthGenerator depth;
  xn::ImageGenerator image;
  XnFieldOfView FOV;
  XnUInt64 depth_focal_length_SXGA;
  XnUInt64 rgb_focal_length_SXGA;
  XnDouble pixel_size;
  XnMapOutputMode output_mode;
  int output_x_resolution;
  double scale;
  double depth_fl;
  double image_fl;
  bool isDepthRegistered;

  //for playing back from file
  xn::Player* player;
  
  //for recording to file
  xn::Recorder* recorder;
  bool shouldRecord;
  
  std::string oniFilename;
  
  log4cxx::LoggerPtr logger;
  
  //helper methods
  void initLiveCapture();
  void initFileCapture();
  void getDepthParameters();
  void initRecord();
  void CheckStatus(const std::string &msg, XnStatus status);
};

#endif //OPENNIDEVICE_HPP
