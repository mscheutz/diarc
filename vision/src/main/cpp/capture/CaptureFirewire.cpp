/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "Capture.hpp"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include "util/CaptureUtilities.hpp"
#include "firewire/FireWireHelper.hpp"                /* Firewire functions */

namespace ade {
  namespace capture {

    class CaptureFirewire : public Capture {
    public:

      CaptureFirewire(const std::string &configFile)
              : Capture(configFile),
                fwhelper(0),
                camType() {

        // parse xml for configuration params
        using boost::property_tree::ptree;
        ptree pt;
        read_xml(configFile, pt);
        camType = pt.get<std::string>("capture.camType"); // TODO: convert string to enum

        switch (camMode) {
          case MONO_FW:
            initMonoFW();
            break;
          case STEREO_FW:
            initStereoFW();
            break;
          case STEREO_SYNC:
            initStereoSync();
            break;
        }
      };

      ~CaptureFirewire() {
        cleanupFwHelper();
      };

      bool captureCurrentCameraFrame() {
        switch (camMode) {
          case MONO_FW:
            return captureHelperMonoFW();
            break;
          case STEREO_FW:
            return captureHelperStereoFW();
            break;
          case STEREO_SYNC:
            return captureHelperStereoSync();
            break;
        }
        return false;
      }

    private:

      void initMonoFW() {
        fwhelper = new FireWireHelper();
        while (!fwhelper->initializeFWSingle(MONO_CAM, camType)) {
          sleep(2);
        }
        //        while (!fwhelper->initializeFW(1)) {
        //            sleep(2);
        //        }
      }

      void initStereoFW() {
        fwhelper = new FireWireHelper();
        while (!fwhelper->initializeFWMultiple(2)) {
          sleep(2);
        }
      }

      void initStereoSync() {
        fwhelper = new FireWireHelper();
        while (!fwhelper->initializeFWSingle(STEREO_SYNC_CAM, camType)) {
          sleep(2);
        }
      }

      //void initBumblebeePlusSaveUnibrain() {
      //    fwhelper = new FireWireHelper();
      //    while (!fwhelper->initializeFWMultiple(2)) {
      //        sleep(2);
      //    }
      //}

      bool captureHelperMonoFW() {
        //fill frame without re-allocating imgData
        IplImage iplFrame = frame;
        fwhelper->getSingleFWFrame(MONO_CAM, camType, &iplFrame);
        util::convertColorsIfNecessary(&iplFrame, YCrCb, BGR); // comes in as YcrCb

        //un-distort if requested and able
        undistortFrame(0);

        return true;
      }

      bool captureHelperStereoFW() {
        //fill frame and frame2 without re-allocating imgData
        IplImage iplFrame = frame;
        IplImage iplFrame2 = frame2;
        fwhelper->getSingleFWFrame(0, camType, &iplFrame);
        fwhelper->getSingleFWFrame(1, camType, &iplFrame2);
        util::convertColorsIfNecessary(&iplFrame, YCrCb, RGB);
        util::convertColorsIfNecessary(&iplFrame2, YCrCb, RGB);

        //un-distort if requested and able
        undistortFrame(0);
        undistortFrame(1);

        //generate depth maps (via stereo) if stereo calibration has been done
        //NOTE: generally a good idea to un-distort first, but not strictly necessary
        reconstructStereoFrames();

        return true;
      }

      bool captureHelperStereoSync() {
        IplImage *sync_f1, *sync_f2, *disparity_src;

        fwhelper->getStereoSyncFWFrames(&sync_f1, &sync_f2, &disparity_src);

        IplImage iplFrame = frame;
        IplImage iplFrame2 = frame2;
        cvResize(sync_f1, &iplFrame, CV_INTER_AREA);
        cvResize(sync_f2, &iplFrame2, CV_INTER_AREA);
        cvReleaseImage(&sync_f1);
        cvReleaseImage(&sync_f2);
        cvReleaseImage(&disparity_src);

        return true;
      }

      bool captureHelperBumblebeePlusSaveUnibrain() {
        //fill frame without re-allocating imgData
        IplImage iplFrame = frame;
        fwhelper->getSingleFWFrame(1, camType, &iplFrame);
        cvSaveImage("tmp/frame_prev.jpg", &iplFrame);
        captureHelperStereoSync();

        return true;
      }

      void cleanupFwHelper() {
        delete fwhelper;
      }

      FireWireHelper *fwhelper;
      CAM_TYPE camType;
    };

  } //namespace capture
} //namespace ade  