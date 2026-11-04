/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   CaptureSpot.hpp
 * Author: evan
 *
 * Created on May 18, 2015, 12:01 PM
 */

#ifndef CAPTURESPOTPYTHON_HPP
#define  CAPTURESPOTPYTHON_HPP

#include "Capture.hpp"

#include <pybind11/pytypes.h>

namespace diarc {
  namespace capture {
    class CaptureSpotPython : public Capture {
    public:
      CaptureSpotPython(const std::string& configFile);

      virtual ~CaptureSpotPython();

      virtual bool captureCurrentCameraFrame();

    private:
      void loadConfig(const std::string& configFile);
      void setRGBCameraParameters(const pybind11::list& params) const;
      void setDepthCameraParameters(const pybind11::list& params) const;

      std::string hostname;
      std::string username;
      std::string password;

      void createSpotCameraCapture();
      void destroySpotCameraCapture();

      bool spot_capture_initialized;
      pybind11::object SpotCameraCapture;
      pybind11::object spotCameraInstance;
    };
  } //namespace capture
} //namespace diarc

#endif  /* CAPTURESPOTPYTHON_HPP */
