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

namespace diarc {
  namespace capture {
    class CaptureSpot : public Capture {
    public:
      CaptureSpot(const std::string& configFile);

      virtual ~CaptureSpot();

      virtual bool captureCurrentCameraFrame();

    private:
      void loadConfig(const std::string& configFile);
      //void setRGBCameraParameters(const bosdyn::api::ImageResponse& image_response) const;
      //void setDepthCameraParameters(const bosdyn::api::ImageResponse& image_response) const;

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
