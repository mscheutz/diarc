/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
#ifndef CAPTUREGSTREAMER_H
#define CAPTUREGSTREAMER_H

#include "Capture.hpp"
#include <gst/gst.h>
#include <gst/app/gstappsink.h>

namespace ade {
  namespace capture {
    class CaptureGStreamer : public Capture {
    public:

      CaptureGStreamer(const std::string &configFile);

      ~CaptureGStreamer();

      bool captureCurrentCameraFrame();

    private:
      GstElement *pipeline;
    };

  } //namespace capture
} //namespace ade

#endif //CAPTUREGSTREAMER_H
