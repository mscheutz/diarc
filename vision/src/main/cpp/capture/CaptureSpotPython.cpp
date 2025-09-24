/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "CaptureSpotPython.hpp"
#include "util/CaptureUtilities.hpp"
#include "point_clouds/PointCloudUtilities.hpp"
#include "pyvis/PythonUtils.hpp"
#include "pyvis/PythonHelper.hpp"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <pybind11/embed.h> // everything needed for embedding

namespace py = pybind11;

namespace diarc {
  namespace capture {
    CaptureSpotPython::CaptureSpotPython(const std::string& configFile)
      : Capture(configFile){

      // parse configuration params
      loadConfig(configFile);

      createSpotCameraCapture();

      depthFrame.create(imgHeight, imgWidth, CV_32FC1);
    };

    CaptureSpotPython::~CaptureSpotPython() {
      PythonHelper* pyInstance = PythonHelper::getInstance();
      pyInstance->acquireGIL();

      SpotCameraCapture = py::object();
      spotCameraInstance = py::object();

      pyInstance->releaseGIL();
    };

    void CaptureSpotPython::createSpotCameraCapture() {
      LOG4CXX_DEBUG(logger, "[createSpotCameraCapture]");

      PythonHelper* pyInstance = PythonHelper::getInstance();
      try {
        pyInstance->acquireGIL();

        // Equivalent to "from diarc.spot.spot_camera_capture import SpotCameraCapture"
        py::module_ module = pyInstance->importModule("diarc.spot.spot_camera_capture");
        SpotCameraCapture = module.attr("SpotCameraCapture");

        // Construct a Python object of class SpotCameraCapture
        spotCameraInstance = SpotCameraCapture(
          py::arg("hostname") = hostname,
          py::arg("username") = username,
          py::arg("password") = password
        );

        // get camera parameters
        py::object rgb_params = spotCameraInstance.attr("get_rgb_camera_params")();
        setRGBCameraParameters(rgb_params.cast<py::list>());

        py::object depth_params = spotCameraInstance.attr("get_depth_camera_params")();
        setDepthCameraParameters(depth_params.cast<py::list>());

      } catch (py::error_already_set& e) {
        LOG4CXX_ERROR(logger, boost::format("[createSpotCameraCapture] %s") % e.what());
      }

      pyInstance->releaseGIL();
    }

    void CaptureSpotPython::loadConfig(const std::string& configFile) {
      // get directory of config file
      unsigned found = configFile.find_last_of("/\\");
      std::string dir = configFile.substr(0, found + 1);

      // extract configuration from xml
      using boost::property_tree::ptree;
      ptree pt;
      read_xml(configFile, pt);

      hostname = pt.get<std::string>("capture.hostname");
      username = pt.get<std::string>("capture.username");
      password = pt.get<std::string>("capture.password");
    }

    bool CaptureSpotPython::captureCurrentCameraFrame() {
      // acquire GIL
      PythonHelper* pyInstance = PythonHelper::getInstance();
      pyInstance->acquireGIL();

      // get images from python
      py::object py_rgb_img = spotCameraInstance.attr("get_rgb_image")();
      cv::Mat cv_rgb_img = PythonUtils::convertNumpyToMat(py_rgb_img.cast<py::array_t<uchar>>());

      py::object py_depth_img = spotCameraInstance.attr("get_depth_aligned_image")();
      cv::Mat cv_depth_img = PythonUtils::convertNumpyToMatDepth(py_depth_img.cast<py::array_t<float>>());

      // release GIL
      pyInstance->releaseGIL();

      // resize and populate diarc images
      cv::resize(cv_rgb_img, frame, frame.size(), 0, 0, cv::INTER_NEAREST);
      #if (CV_MAJOR_VERSION >= 4)
        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB); // opencv 4
      #else
        cv::cvtColor(frame, frame, CV_BGR2RGB); //  opencv 3
      #endif

      cv::resize(cv_depth_img, depthFrame, depthFrame.size(), 0, 0, cv::INTER_NEAREST);

      // create point cloud from color and depth frames
      pc::util::depthAndColorToPointCloud(depthFrame, frame, cloudRGB, cloud);

      return true;
    }


    void CaptureSpotPython::setRGBCameraParameters(const py::list& params) const {
      if (cameras->getCameraParameters(1)->intrinsicSet) {
        return;
      }

      //set rgb frame parameters
      int width = params[0].cast<int>();
      int height = params[1].cast<int>();
      double f_x = params[2].cast<double>();
      double f_y = params[3].cast<double>();
      double c_x = params[4].cast<double>();
      double c_y = params[5].cast<double>();

      LOG4CXX_DEBUG(
        logger,
        boost::format("[setRGBCameraParameters] cam params: %d %d %d %d %d %d")
        % width % height % f_x % f_y % c_x % c_y);

      CameraParameters::Ptr camParams(new CameraParameters(VisionConstants::imgWidth, VisionConstants::imgHeight));
      camParams->M.at<float>(0, 0) = f_x * (static_cast<float>(imgWidth) / width); // f_x
      camParams->M.at<float>(1, 1) = f_y * (static_cast<float>(imgHeight) / height); // f_y
      camParams->M.at<float>(0, 2) = c_x * (static_cast<float>(imgWidth) / width); // c_x
      camParams->M.at<float>(1, 2) = c_y * (static_cast<float>(imgHeight) / height); // c_y
      camParams->intrinsicSet = true;
      cameras->setCameraParameters(1, camParams);
    }

    void CaptureSpotPython::setDepthCameraParameters(const py::list& params) const {
      if (cameras->getCameraParameters(0)->intrinsicSet) {
        return;
      }

      // set depth image parameters
      int width = params[0].cast<int>();
      int height = params[1].cast<int>();
      double f_x = params[2].cast<double>();
      double f_y = params[3].cast<double>();
      double c_x = params[4].cast<double>();
      double c_y = params[5].cast<double>();

      LOG4CXX_DEBUG(
        logger,
        boost::format("[setDepthCameraParameters] cam params: %d %d %d %d %d %d")
        % width % height % f_x % f_y % c_x % c_y);

      CameraParameters::Ptr camParams(new CameraParameters(VisionConstants::imgWidth, VisionConstants::imgHeight));
      camParams->M.at<float>(0, 0) = f_x * (static_cast<float>(imgWidth) / width); // f_x
      camParams->M.at<float>(1, 1) = f_y * (static_cast<float>(imgHeight) / height); // f_y
      camParams->M.at<float>(0, 2) = c_x * (static_cast<float>(imgWidth) / width); // c_x
      camParams->M.at<float>(1, 2) = c_y * (static_cast<float>(imgHeight) / height); // c_y
      camParams->intrinsicSet = true;
      cameras->setCameraParameters(0, camParams);
    }
  } //namespace capture
} //namespace diarc
