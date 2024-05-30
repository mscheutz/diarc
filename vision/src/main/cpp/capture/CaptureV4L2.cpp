/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "Capture.hpp"

#include "util/CaptureUtilities.hpp"
#include "v4l2/V4L2Helper.hpp"                        /* V4L2 functions */
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace diarc {
  namespace capture {

    class CaptureV4L2 : public Capture {
    public:

      CaptureV4L2(const std::string &configFile)
              : Capture(configFile),
                v4l2helper(0),
                v4l2helper2(0) {

        // parse xml for configuration params
        using boost::property_tree::ptree;
        ptree pt;
        read_xml(configFile, pt);
        int deviceNum = pt.get<int>("capture.deviceNum", 0);

        if (stereoFlag) {
          initStereoV4L2(deviceNum);
        } else {
          initMonoV4L2(deviceNum);
        }
      };

      ~CaptureV4L2() {
        cleanupV4L2();
      };

      bool captureCurrentCameraFrame() {
        if (stereoFlag) {
          captureHelperStereoV4L2();
        } else {
          captureHelperMonoV4L2();
        }

        return true;
      }

    private:

      void initMonoV4L2(const int deviceNum) {
        v4l2helper = new V4L2Helper(deviceNum, imgWidth, imgHeight);
        while (!v4l2helper->init_V4L2Device()) {
          sleep(2);
        }
      }

      void initStereoV4L2(const int deviceNum) {
        v4l2helper = new V4L2Helper(deviceNum, imgWidth, imgHeight);
        v4l2helper2 = new V4L2Helper(deviceNum + 1, imgWidth, imgHeight);
        while (!v4l2helper->init_V4L2Device() || !v4l2helper2->init_V4L2Device()) {
          sleep(2);
        }
      }

      void captureHelperMonoV4L2() {
        //fill frame without re-allocating imgData
        v4l2helper->get_frame(frame);
        util::convertColorsIfNecessary(frame, YCrCb, RGB); // seems like it comes in YCrCb. (?)

        //un-distort if requested and able
        undistortFrame(0);
      }

      void captureHelperStereoV4L2() {
        //fill frame and frame2 without re-allocating imgData
        v4l2helper->get_frame(frame);
        v4l2helper2->get_frame(frame2);
        util::convertColorsIfNecessary(frame, YCrCb, RGB);
        util::convertColorsIfNecessary(frame2, YCrCb, RGB);

        //un-distort if requested and able
        undistortFrame(0);
        undistortFrame(1);

        //generate depth maps (via stereo) if stereo calibration has been done
        //NOTE: generally a good idea to un-distort first, but not strictly necessary
        reconstructStereoFrames();
      }

      void cleanupV4L2() {
        delete v4l2helper;
        delete v4l2helper2;
      }

      V4L2Helper *v4l2helper;
      V4L2Helper *v4l2helper2;


      cv::Mat tmpFrame;
    };

  } //namespace capture
} //namespace diarc