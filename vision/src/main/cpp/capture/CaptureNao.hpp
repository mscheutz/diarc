/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   CaptureNao.hpp
 * Author: evan
 *
 * Created on May 18, 2015, 12:01 PM
 */

#ifndef CAPTURENAO_HPP
#define  CAPTURENAO_HPP

#include "Capture.hpp"
#include <alproxies/alvideodeviceproxy.h>

namespace diarc {
  namespace capture {

    class CaptureNao : public Capture {
    public:

      CaptureNao(const std::string &configFile);

      virtual ~CaptureNao();

      virtual bool captureCurrentCameraFrame();

    private:
      std::string videoClientName;
      boost::shared_ptr<AL::ALVideoDeviceProxy> cameraProxy;
      cv::Mat tmpFrame;
    };

  } //namespace capture
} //namespace diarc

#endif  /* CAPTURENAO_HPP */

