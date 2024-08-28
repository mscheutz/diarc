/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "Capture.hpp"

#include "util/CaptureUtilities.hpp"
#include "openni/OpenniDevice.hpp"
#include "v4l2/V4L2Helper.hpp"                        /* V4L2 functions */
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>


//TODO: this is kind of a hacky way to allow for a Kinect and separate RGB cam to run together.
//Capturing should be made more modular to allow for combining existing Capture classes.

namespace diarc {
  namespace capture {

    class CaptureOpenniPlus : public Capture {
    public:

      CaptureOpenniPlus(const std::string &configFile)
              : Capture(configFile),
                openniDevice(true),
                v4l2helper(0) {

        // parse xml for configuration params
        using boost::property_tree::ptree;
        ptree pt;
        read_xml(configFile, pt);
        int deviceNum = pt.get<int>("capture.deviceNum", 0);

        //init depth frame
        depthFrame.create(imgHeight, imgWidth, CV_32FC1);

        //init rgb camera
        v4l2helper = new V4L2Helper(deviceNum, imgWidth, imgHeight);
        while (!v4l2helper->init_V4L2Device()) {
          sleep(2);
        }
      };

      ~CaptureOpenniPlus() {
        delete v4l2helper;
      };

      bool captureCurrentCameraFrame() {
        //capture kinect data
        openniDevice.CaptureOpenni(colorImg, depthImg);
        cv::resize(colorImg, frame, frame.size(), 0, 0, cv::INTER_LINEAR);
        cv::resize(depthImg, depthFrame, depthFrame.size(), 0, 0, cv::INTER_NEAREST);

        //capture (non-kinect) rgb camera data
        v4l2helper->get_frame(frame2);
        util::convertColorsIfNecessary(frame2, YCrCb, RGB);

        return true;
      }

    private:
      OpenniDevice openniDevice;
      cv::Mat colorImg, depthImg;

      V4L2Helper *v4l2helper;
    };

  } //namespace capture
} //namespace diarc