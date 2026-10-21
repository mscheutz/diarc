/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   CaptureSpot.hpp
 * Author: evan
 *
 * Created on May 18, 2015, 12:01 PM
 */

#ifndef CAPTURESPOT_HPP
#define  CAPTURESPOT_HPP

#include "Capture.hpp"

#include <bosdyn/client/image/image_client.h>
#include <bosdyn/client/sdk/client_sdk.h>
#include <bosdyn/client/util/cli_util.h>

namespace diarc {
  namespace capture {
    class CaptureSpot : public Capture {
    public:
      CaptureSpot(const std::string& configFile);

      virtual ~CaptureSpot();

      virtual bool captureCurrentCameraFrame();

    private:
      void loadConfig(const std::string& configFile);
      void setRGBCameraParameters(const bosdyn::api::ImageResponse& image_response) const;
      void setDepthCameraParameters(const bosdyn::api::ImageResponse& image_response) const;

      std::string hostname;
      std::string username;
      std::string password;
      std::unique_ptr<::bosdyn::client::ClientSdk> client_sdk;
      std::unique_ptr<::bosdyn::client::Robot> robot;
      ::bosdyn::client::Result<::bosdyn::client::ImageClient*> image_client_result;
    };
  } //namespace capture
} //namespace diarc

#endif  /* CAPTURESPOT_HPP */
