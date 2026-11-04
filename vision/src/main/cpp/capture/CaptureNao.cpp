/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "CaptureNao.hpp"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include "util/CaptureUtilities.hpp"
#include <alvision/alvisiondefinitions.h>
#include <boost/format.hpp>
#include <iostream>

namespace diarc {
  namespace capture {
    using namespace AL;

    CaptureNao::CaptureNao(const std::string &configFile)
            : Capture(configFile) {

      videoClientName = "ADECapture";

      // parse xml for configuration params
      using boost::property_tree::ptree;
      ptree pt;
      read_xml(configFile, pt);
      std::string ip = pt.get<std::string>("capture.ip", "192.168.0.190");
      int port = pt.get<int>("capture.port", 9559);

      int resolution = getResolutionFromSize(imgWidth, imgHeight);
      if (!isResolutionValid(resolution)) {
        LOG4CXX_FATAL(logger, "Invalid image resolution!");
        exit(0);
      }

      int naoColorSpace = 13;
//      switch (colorSpace) {
//        case RGB:
//          naoColorSpace = 13;
//        case Luv:
//          naoColorSpace = 11; //TODO: Luv not supported by naoqi
//        case YCrCb:
//          naoColorSpace = 14;
//        case BGR:
//          naoColorSpace = 13;
//      }

      // Allocate our Image header.
      tmpFrame = cv::Mat(cv::Size(imgWidth, imgHeight), CV_8UC3);

      LOG4CXX_INFO(logger,
                   boost::format("img size: %d x %d. resolution: %d. color: %d.") % imgWidth % imgHeight % resolution %
                   naoColorSpace);

      cameraProxy = boost::shared_ptr<ALVideoDeviceProxy>(new ALVideoDeviceProxy(ip, port));
      videoClientName = cameraProxy->subscribe(videoClientName, resolution, naoColorSpace, 30);

    };

    CaptureNao::~CaptureNao() {
      if (cameraProxy) {
        cameraProxy->unsubscribe(videoClientName);
      }
    };

    bool CaptureNao::captureCurrentCameraFrame() {
      ALValue results = cameraProxy->getImageRemote(videoClientName);

      if (results.getType() != ALValue::TypeArray && results.getSize() != 7) {
        LOG4CXX_ERROR(logger, "Invalid image returned.");
        return false;
      }

      // Set the buffer we received to our cv::Mat header.
      tmpFrame.data = (uchar *) (results[6].GetBinary());
      tmpFrame.copyTo(frame);

      cameraProxy->releaseImage(videoClientName);

      return true;
    }

  } //namespace capture
} //namespace diarc
