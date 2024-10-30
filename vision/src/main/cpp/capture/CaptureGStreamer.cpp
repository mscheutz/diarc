/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
#include "CaptureGStreamer.hpp"
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>

/**
* GStreamer capture module for the Vision Component.
*
* NOTE: all pipelines (specified via xml) must end with the following:
* videoconvert ! capsfilter caps="video/x-raw, format=BGR" ! appsink name=sink sync=false
*/
namespace diarc {
  namespace capture {
    CaptureGStreamer::CaptureGStreamer(const std::string &configFile)
            : Capture(configFile) {

      // parse xml for configuration params
      using boost::property_tree::ptree;
      ptree pt;
      read_xml(configFile, pt);
      std::string pipelineStr = pt.get<std::string>("capture.pipeline");

      // Init GStreamer
      gst_init(NULL, NULL);

      // Launch pipeline
      GError *gError = NULL;
      pipeline = gst_parse_launch(pipelineStr.c_str(), &gError);

      // Check if pipeline launched
      if (gError == NULL) {
        // Get sink
        GstElement *sink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");

        // Set buffer size to 1 frame, instruct sink to drop old frames when new arrive.
        gst_app_sink_set_max_buffers((GstAppSink *) sink, 1);
        gst_app_sink_set_drop((GstAppSink *) sink, true);

        // Set pipeline to playing...
        gst_element_set_state(pipeline, GST_STATE_PLAYING);
      } else {
        g_error("Could not construct GStreamer pipeline: %s\n", gError->message);
        g_error_free(gError);
      }
    };

    CaptureGStreamer::~CaptureGStreamer() {
      // Release pipeline
      gst_element_set_state(pipeline, GST_STATE_NULL);
      gst_object_unref(GST_OBJECT(pipeline));
    };

    bool CaptureGStreamer::captureCurrentCameraFrame() {
      GstElement *sink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
      GstSample *sample = gst_app_sink_pull_sample((GstAppSink *) sink);
      GstBuffer *buffer = gst_sample_get_buffer(sample);

      GstMapInfo map;
      gst_buffer_map(buffer, &map, GST_MAP_READ);

      // Get frame size
      gint w, h;
      const GstStructure *str;
      str = gst_caps_get_structure(gst_sample_get_caps(sample), 0);
      if (!gst_structure_get_int(str, "width", &w) || !gst_structure_get_int(str, "height", &h)) {
        g_print("No width/height available! \n");
        return false;
      }

      cv::Mat f(cv::Size(w, h), CV_8UC3, (char *) map.data, cv::Mat::AUTO_STEP);
      cv::resize(f, frame, cv::Size(imgWidth, imgHeight), 0, 0, cv::INTER_LINEAR);

      gst_buffer_unmap(buffer, &map);
      gst_sample_unref(sample);
      return true;
    }
  } //namespace capture
} //namespace diarc