/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "OpenniDevice.hpp"

#include "boost/format.hpp"
xn::Context OpenniDevice::context;

OpenniDevice::~OpenniDevice() {
  // Clean up
  // context.Release();
  context.SetHandle(NULL); //equivalent to Release, but exists in older OpenNI versions
}

OpenniDevice::OpenniDevice(bool liveCapture, const std::string& oniPlaybackFilename)
: player(NULL),
recorder(NULL),
shouldRecord(false),
oniFilename(oniPlaybackFilename) {
  logger = log4cxx::Logger::getLogger("ade.capture.openni.OpenniDevice");

  if (liveCapture) {
    initLiveCapture();
  } else {
    initFileCapture();
  }

  // get depth parameters from openni and set locally
  getDepthParameters();
}

void OpenniDevice::CaptureOpenni(cv::Mat &colorImg, cv::Mat &depthImg) {
  context.WaitAndUpdateAll();

  if (shouldRecord && recorder == NULL) {
    initRecord();
  } else if (!shouldRecord && recorder != NULL) {
    recorder->Release();
    delete recorder;
    recorder = NULL;
  }

  xn::DepthMetaData depth_meta;
  depth.GetMetaData(depth_meta);
  xn::ImageMetaData image_meta;
  image.GetMetaData(image_meta);

  cv::Mat temp;
  temp = cv::Mat(image_meta.YRes(), image_meta.XRes(), CV_8UC3, (char*) image_meta.Data());
  cv::cvtColor(temp, colorImg, cv::COLOR_BGR2RGB);

  depthImg = cv::Mat_<float>::zeros(depth_meta.YRes(), depth_meta.XRes());
  for (int i = 0; i < depthImg.rows; ++i) {
    for (int j = 0; j < depthImg.cols; ++j) {
      depthImg.at<float>(i, j) = depth_meta[j + i * depthImg.cols] * 0.001;
    }
  }
}

double OpenniDevice::getDepthFocalLength() const {
  return depth_fl;
}

double OpenniDevice::getImageFocalLength() const {
  return image_fl;
}

void OpenniDevice::initLiveCapture() {
  // from:
  // ros/diamondback/stacks/openni_kinect/openni_camera/src/openni_device.cpp, line 628
  // or:
  // http://www.ros.org/doc/api/openni_camera/html/classopenni__wrapper_1_1OpenNIDevice.html
  rgb_focal_length_SXGA = 1050; //magic value taken from a calibration routine

  status = XN_STATUS_OK;

  // Initialize context object
  status = context.Init();
  CheckStatus("init context", status);

  // Create a DepthGenerator node
  status = depth.Create(context);
  CheckStatus("create depth generator", status);

  // Create a RGB ImageGenerator node
  status = image.Create(context);
  CheckStatus("create image generator", status);

  // Make it start generating data
  status = context.StartGeneratingAll();
  CheckStatus("context start generating all", status);

  // align the depth image with the color image
  //status = depth.GetAlternativeViewPointCap().SetViewPoint(image);
  //CheckStatus("change depth view point", status);
}

void OpenniDevice::initFileCapture() {
  XnStatus nRetVal = XN_STATUS_OK;

  // Initialize context object
  nRetVal = context.Init();
  CheckStatus("init context", nRetVal);

  //init player
  player = new xn::Player;
  context.OpenFileRecording(oniFilename.c_str(), *player);

  nRetVal = player->SetRepeat(TRUE);
  CheckStatus("Turn repeat on", nRetVal);

  // Get depth node from recording
  nRetVal = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depth);
  CheckStatus("Find depth generator", nRetVal);

  // Get rgb image node from recording
  nRetVal = context.FindExistingNode(XN_NODE_TYPE_IMAGE, image);
  CheckStatus("Find image generator", nRetVal);

  // Make it start generating data
  status = context.StartGeneratingAll();
  CheckStatus("context start generating all", status);
}

void OpenniDevice::getDepthParameters() {
  depth.GetFieldOfView(FOV);
  LOG4CXX_INFO(logger, boost::format("depth FOV [degree]: %lf %lf.")
          % (FOV.fHFOV * 180. / M_PI) % (M_PI, FOV.fVFOV * 180. / M_PI));

  status = depth.GetIntProperty("ZPD", depth_focal_length_SXGA);
  LOG4CXX_INFO(logger, boost::format("depth focal length in mm: %lu.")
          % (long unsigned) depth_focal_length_SXGA);

  status = depth.GetRealProperty("ZPPS", pixel_size);
  LOG4CXX_INFO(logger, boost::format("depth focal length in pixels for SXGA (1280x1024): %lf.")
          % ((double) depth_focal_length_SXGA / pixel_size));

  status = depth.GetMapOutputMode(output_mode);
  output_x_resolution = output_mode.nXRes;

  scale = scale = output_x_resolution / (float) XN_SXGA_X_RES;
  if (depth.GetAlternativeViewPointCap().IsViewPointAs(image)) {
    LOG4CXX_INFO(logger, "depth viewpoint is from image.");
    depth_fl = scale * rgb_focal_length_SXGA;
  } else {
    LOG4CXX_INFO(logger, "depth viewpoint is NOT from image.");
    depth_fl = scale * (double) depth_focal_length_SXGA / pixel_size;
  }
  LOG4CXX_INFO(logger, boost::format("depth focal length in pixels for actual resolution of %dx%d: %lf.")
          % output_mode.nXRes % output_mode.nYRes % depth_fl);

  status = image.GetMapOutputMode(output_mode);
  CheckStatus("get image map output mode", status);
  output_x_resolution = output_mode.nXRes;
  scale = (double) output_x_resolution / (float) XN_SXGA_X_RES;
  image_fl = scale * (double) rgb_focal_length_SXGA;
  LOG4CXX_INFO(logger, boost::format("image focal length in pixels for actual resolution of %dx%d: %lf.")
          % output_mode.nXRes % output_mode.nYRes % image_fl);
}

void OpenniDevice::initRecord() {

  XnStatus nRetVal = XN_STATUS_OK;
  recorder = new xn::Recorder;

  nRetVal = recorder->Create(context);
  CheckStatus("Create recorder", nRetVal);

  nRetVal = recorder->SetDestination(XN_RECORD_MEDIUM_FILE, oniFilename.c_str());
  CheckStatus("Set recorder destination file", nRetVal);

  nRetVal = recorder->AddNodeToRecording(depth);
  CheckStatus("Add depth node to recording", nRetVal);

  nRetVal = recorder->AddNodeToRecording(image);
  CheckStatus("Add rgb node to recording", nRetVal);

  //recording is automatically started and context.WaitAndUpdateAll()
  //takes care of calling "record" automatically
}

void OpenniDevice::record(const std::string& newOniFilename) {
  oniFilename = newOniFilename;
  shouldRecord = true;
}

void OpenniDevice::stopRecord() {
  shouldRecord = false;
}

void OpenniDevice::CheckStatus(const std::string &msg, XnStatus status) {
  if (status != XN_STATUS_OK) {
    LOG4CXX_ERROR(logger, boost::format("Failed: %s -  Reason: %s.") % msg % xnGetStatusString(status));
    exit(EXIT_FAILURE);
  }
}
