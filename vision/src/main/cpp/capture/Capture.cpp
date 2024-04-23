/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "Capture.hpp"

#include <sys/types.h>
#include <sys/stat.h>
#include <string>
#include <ios>
#include <fstream>
#include <boost/thread/locks.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>

//for capture(width, height, etc..) which should be moved?
#include "capture/calibration/Cameras.hpp"
#include "display/Display.hpp"

//for factory method
#include "CaptureFile.cpp"
#include "CaptureV4L2.cpp"

#include <pcl/io/pcd_io.h>
#include "CaptureSimulateKinect.cpp"
#include "CapturePCDFile.cpp"
#include "common/notification/CaptureNotification.hpp"

#ifdef FREENECT
#include "CaptureFreenect.cpp"
#endif
#ifdef USE_OPENNI

#include "CaptureOpenni.cpp"
#include "CaptureOpenniPlus.cpp"

#endif
#ifdef USE_OPENNI2
#include "CaptureOpenni2.cpp"
#endif

#ifdef USE_REALSENSE
#include "CaptureRealSense.hpp"
#endif

#ifdef USE_ROS_VISION
#include "CaptureROS.hpp"
#include "CaptureMultisense.hpp"
#endif

#ifdef USE_NAO
#include "CaptureNao.hpp"
#endif

#ifdef USE_GSTREAMER
#include "CaptureGStreamer.hpp"
#endif

namespace ade {
  namespace capture {
    ///////////// START: static member initialization ////////////////

    log4cxx::LoggerPtr Capture::logger(log4cxx::Logger::getLogger("ade.capture.Capture"));

    CaptureNotification::Ptr Capture::captureNotification;

    boost::mutex Capture::captureMutex;

    std::vector<VisionProcess::Ptr> Capture::registeredProcessors;

    boost::mutex Capture::registeredProcsMutex;
    ///////////// END: static member initialization ////////////////

    Capture::Ptr Capture::createCaptureDevice(const std::string &configFile) {

      // get camera mode from config file
      using boost::property_tree::ptree;
      ptree pt;
      read_xml(configFile, pt);
      CAM_MODE camMode = util::stringToCAM_MODE(pt.get<std::string>("capture.camMode"));

      Capture::Ptr captureDevice;
      LOG4CXX_INFO(logger, boost::format("Capture Factory cammode: %d.") % camMode);

      switch (camMode) {
        case MONO_FW:
        case STEREO_FW:
        case STEREO_SYNC: LOG4CXX_ERROR(logger, "FireWire modes not available. Capture classes need to be rewritten!!");
          //captureDevice = Capture::Ptr(new CaptureFirewire(configFile));
          break;
        case MONO_V4L2:
          captureDevice = Capture::Ptr(new CaptureV4L2(configFile));
          break;
        case STEREO_V4L2:
          captureDevice = Capture::Ptr(new CaptureV4L2(configFile));
          break;
        case MONO_FILE:
          captureDevice = Capture::Ptr(new CaptureFile(configFile));
          break;
        case MULTI_FILE_LOG:
          captureDevice = Capture::Ptr(new CaptureFile(configFile));
          break;
        case MONO_AVI:
          captureDevice = Capture::Ptr(new CaptureFile(configFile));
          break;
        case OPENNI:
#ifdef USE_OPENNI
          captureDevice = Capture::Ptr(new CaptureOpenni(configFile));
#else
          LOG4CXX_ERROR(logger, "OPENNI is not available.");
#endif
          break;
        case OPENNI_ONI:
#ifdef USE_OPENNI
          captureDevice = Capture::Ptr(new CaptureOpenni(configFile));
#else
          LOG4CXX_ERROR(logger, "OPENNI is not available.");
#endif
          break;
        case OPENNI2:
#ifdef USE_OPENNI2
          captureDevice = Capture::Ptr(new CaptureOpenni2(configFile));
#else
          LOG4CXX_ERROR(logger, "OPENNI2 is not available.");
#endif
          break;
        case REALSENSE:
#ifdef USE_REALSENSE
          captureDevice = Capture::Ptr(new CaptureRealSense(configFile));
#else
        LOG4CXX_ERROR(logger, "RealSense driver is not available.");
#endif
          break;
          //==================== Start PR2 Options ===========================
        case ROS:
#ifdef USE_ROS_VISION
          captureDevice = Capture::Ptr(new CaptureROS(configFile));
#else
        LOG4CXX_ERROR(logger, "ROS is not available.");
#endif
          break;
          //====================== End PR2 Options ===========================

        case MULTISENSE:
#ifdef USE_ROS_VISION
          captureDevice = Capture::Ptr(new CaptureMultisense(configFile));
#else
        LOG4CXX_ERROR(logger, "ROS is not available.");
#endif
          break;

        case SIM_PC1:
        case SIM_PC2:
          captureDevice = Capture::Ptr(new CaptureSimulateKinect(configFile));
          break;
        case KINECT_PLUS:
#ifdef USE_OPENNI
          captureDevice = Capture::Ptr(new CaptureOpenniPlus(configFile));
#else
          LOG4CXX_ERROR(logger, "OpenNI is not available.");
#endif
          break;
        case NAO:
#ifdef USE_NAO
          captureDevice = Capture::Ptr(new CaptureNao(configFile));
#else
        LOG4CXX_ERROR(logger, "NAOqi SDK is not available.");
#endif
          break;
        case GSTREAMER:
#ifdef USE_GSTREAMER
          captureDevice = Capture::Ptr(new CaptureGStreamer(configFile));
#else
          LOG4CXX_ERROR(logger, "GStreamer not available");
#endif
          break;
        default: LOG4CXX_ERROR(logger, "Invalid camera mode!");
        case NUM_MODES:
          break;
      }

      return captureDevice;
    }

    void Capture::registerForNotification(VisionProcess::Ptr notifyTarget) {
      boost::lock_guard<boost::mutex> lock(registeredProcsMutex);
      //check to see if processor is already registered
      std::vector<VisionProcess::Ptr>::const_iterator itr;
      for (itr = registeredProcessors.begin(); itr != registeredProcessors.end(); ++itr) {
        if (itr->get() == notifyTarget.get()) {
          //already registered
          return;
        }
      }
      registeredProcessors.push_back(notifyTarget);
      LOG4CXX_DEBUG(logger,
                    boost::format("[registerForNotification] registered: '%s'") % notifyTarget->visionProcessName);
    }

    void Capture::unregisterForNotification(VisionProcess::Ptr notifyTarget) {
      boost::lock_guard<boost::mutex> lock(registeredProcsMutex);
      //check to see if processor is registered
      std::vector<VisionProcess::Ptr>::iterator itr;
      for (itr = registeredProcessors.begin(); itr != registeredProcessors.end(); ++itr) {
        if ((*itr).get() == notifyTarget.get()) {
          //found it, remove it
          registeredProcessors.erase(itr);
          return;
        }
      }
      LOG4CXX_DEBUG(logger,
                    boost::format("[unregisterForNotification] unregistered: '%s'") % notifyTarget->visionProcessName);
    }

    void Capture::sendNotifications(Notification::Ptr n) {
      boost::lock_guard<boost::mutex> lock(registeredProcsMutex);
      for (std::vector<VisionProcess::Ptr>::const_iterator itr = registeredProcessors.begin();
           itr != registeredProcessors.end(); ++itr) {
        (*itr)->notify(n);

        LOG4CXX_TRACE(logger, boost::format("[sendNotifications] notifying: '%s'") % (*itr)->visionProcessName);
      }
    }

    CaptureNotification::ConstPtr Capture::getLastCaptureNotification() {
      LOG4CXX_TRACE(logger, "[getLastCaptureNotification] method called.");
      boost::lock_guard<boost::mutex> lock(captureMutex);
      return captureNotification;
    }

    Capture::Capture(const std::string &configFile)
            : frame(), frame2(),
              depthFrame(), depthFrame2(), depthFrameIntensity(),
              cloud(new pcl::PointCloud<pcl::PointXYZ>()),
              cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>()),
              imgWidth(0), imgHeight(0),
              stereoFlag(false), depthFlag(false),
              lasttime(0), currenttime(0), timediff(0), tv(),
              undistortFlag(false),
              cameras(),
              frameCount(0),
              transformMat(4, 4, CV_64F),
              videoWriter(), videoWriter2(),
              saveToAVIstarted(false),
              displayImagesFlag(false),
              displayDepthFlag(false),
              tmpUndistortFrame()
#ifdef USE_ROS_VISION
    , publishCloudToRos(false)
#endif // USE_ROS_VISION
    {

      // get directory of config file
      unsigned found = configFile.find_last_of("/\\");
      std::string dir = configFile.substr(0, found + 1);

      // extract configuration from xml
      using boost::property_tree::ptree;
      ptree pt;
      read_xml(configFile, pt);

      camMode = util::stringToCAM_MODE(pt.get<std::string>("capture.camMode"));
      imgWidth = pt.get<int>("capture.imageWidth");
      imgHeight = pt.get<int>("capture.imageHeight");
      stereoFlag = pt.get<bool>("capture.hasStereo");
      depthFlag = pt.get<bool>("capture.hasDepth");
      cameraInfoFile = pt.get<std::string>("capture.cameraInfo", "");
#ifdef USE_ROS_VISION
      publishCloudToRos = pt.get<bool>("capture.publishCloudToRos", false);
#endif // USE_ROS_VISION

      // set global parameters -- TODO: are these still needed?
      VisionConstants::imgWidth = (unsigned int) imgWidth;
      VisionConstants::imgHeight = (unsigned int) imgHeight;
      VisionConstants::isStereo = stereoFlag;

      // do one-time initialize of camera calibration parameters
      cameras = Cameras::getNewInstance(0, imgWidth, imgHeight, imgWidth, imgHeight);

      //alloc local image data for use during remainder of object's existence
      frame.create(imgHeight, imgWidth, CV_8UC3);
      if (stereoFlag) {
        frame2.create(imgHeight, imgWidth, CV_8UC3);
      }

      if (!cameraInfoFile.empty()) {
        cameraInfoFile = dir + cameraInfoFile;
        std::string calib = "calib";
        std::string calib2 = "calib2";
        if (0 == cameraInfoFile.compare(cameraInfoFile.length() - calib.length(), calib.length(), calib)) {
          cameras->calibrationSingleCamLoad(0, cameraInfoFile.c_str());
        } else if (0 == cameraInfoFile.compare(cameraInfoFile.length() - calib2.length(), calib2.length(), calib2)) {
          cameras->calibrationStereoLoad(cameraInfoFile.c_str());
        } else {
          LOG4CXX_ERROR(logger, "Invalid camera calibration file extension detected : " + cameraInfoFile);
        }
      }

#ifdef USE_ROS_VISION
      if (publishCloudToRos) {
        // initialize ROS
        LOG4CXX_INFO(logger, "Initializing ROS.");
        std::vector<std::pair<std::string, std::string> > remapping;
        ros::init(remapping, "ade_vision_cap");

        // create node handle
        LOG4CXX_INFO(logger, "Initializing ROS node.");
        nh = new ros::NodeHandle();

        // ROS publisher
        pub = nh->advertise<pcl::PointCloud<pcl::PointXYZ> >("/head_mount_kinect/depth_registered/points", 1);
      }
#endif //USE_ROS_VISION
    }

    Capture::~Capture() {
      //cleanup avi writer
      if (saveToAVIstarted) {
        stopAviWrite();
      }
    }

    bool Capture::capture(const double transform[]) {
      //unique scoped lock
      boost::lock_guard<boost::mutex> lock(captureMutex);

      //capture new frame(s) from derived class
      bool success = captureCurrentCameraFrame();

#ifdef USE_ROS_VISION
      if (publishCloudToRos && cloud) {
        cloud->header.frame_id = "head_mount_kinect_rgb_optical_frame";
        cloud->header.stamp = ros::Time::now().toNSec() / 1e3;
        pub.publish(cloud);
        ros::spinOnce();
      }
#endif //USE_ROS_VISION

      //write frame to video file
      if (success && saveToAVIstarted) {
        writeAvi();
      }

      //base to vision transform
      memcpy(transformMat.data, transform, 16 * sizeof(double));

      //create and send notifications to all registered processors
      captureNotification = CaptureNotification::Ptr(new CaptureNotification(++frameCount,
                                                                             transformMat,
                                                                             frame,
                                                                             frame2,
                                                                             depthFrame,
                                                                             depthFrame2,
                                                                             depthFrameIntensity,
                                                                             cloud,
                                                                             cloudRGB));

      // send capture notifications
      sendNotifications(captureNotification);

      // send frame completion notification
      FrameCompletionNotification::Ptr frameCompletionNotification =
              FrameCompletionNotification::Ptr(
                      new FrameCompletionNotification(VisionProcess::Ptr(), -1L, captureNotification->frameNumber));
      sendNotifications(frameCompletionNotification);

      //display captured frames
      if (displayImagesFlag) {
        displayImages();
      }
      if (displayDepthFlag) {
        displayDepth();
      }

      return success;
    }

    void Capture::displayImages() {
      //display captured frame(s)
      if (!frame.empty()) {
        ade::Display::displayFrame(frame, "capture 1");
      }
      if (!frame2.empty()) {
        ade::Display::displayFrame(frame2, "capture 2");
      }
    }

    void Capture::displayDepth() {
      if (!depthFrame.empty()) {
        ade::Display::displayFrame(depthFrame, "depth 1");
      }
      if (!depthFrame2.empty()) {
        ade::Display::displayFrame(depthFrame2, "depth 2");
      }
      if (!depthFrameIntensity.empty()) {
        ade::Display::displayFrame(depthFrameIntensity, "depth intensity");
      }

      //display point cloud
      std::string windowName = "depth 1";
      if (cloudRGB && !cloudRGB->empty()) {
        ade::Display::displayPointCloud(cloudRGB, windowName, windowName);
      } else if (!depthFrame.empty() && !frame.empty()) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        ade::pc::util::depthAndColorToPointCloud(depthFrame, frame, tmpCloud);
        ade::Display::displayPointCloud(tmpCloud, windowName, windowName);
      }
    }


    CAM_MODE Capture::getCameraMode() const {
      return camMode;
    }

    int Capture::getImageHeight() const {
      return imgHeight;
    }

    int Capture::getImageWidth() const {
      return imgWidth;
    }

    bool Capture::hasStereo() const {
      return stereoFlag;
    }

    bool Capture::hasDepth() const {
      return depthFlag;
    }

    void Capture::setImageDisplayFlag(bool flag) {
      boost::lock_guard<boost::mutex> lock(captureMutex);

      if (displayImagesFlag == flag) {
        return;
      }

      if (flag) {
        ade::Display::createWindowIfDoesNotExist("capture 1");
        ade::Display::createWindowIfDoesNotExist("capture 2");
      } else {
        ade::Display::destroyWindowIfItExists("capture 1");
        ade::Display::destroyWindowIfItExists("capture 2");
      }

      displayImagesFlag = flag;
    }

    void Capture::setDepthDisplayFlag(bool flag) {
      boost::lock_guard<boost::mutex> lock(captureMutex);

      if (displayDepthFlag == flag) {
        return;
      }

      if (flag) {
        ade::Display::createWindowIfDoesNotExist("depth 1");
        ade::Display::createWindowIfDoesNotExist("depth 2");
        ade::Display::createWindowIfDoesNotExist("depth intensity");
      } else {
        ade::Display::destroyWindowIfItExists("depth 1");
        ade::Display::destroyWindowIfItExists("depth 2");
        ade::Display::destroyWindowIfItExists("depth intensity");
      }

      displayDepthFlag = flag;
    }

    void Capture::setUndistortFlag(bool flag) {
      boost::lock_guard<boost::mutex> lock(captureMutex);
      undistortFlag = flag;
    }


    bool Capture::getUndistortFlag() const {
      return undistortFlag;
    }

    void Capture::startAviWrite(const std::string &aviFilename) {
      //unique scoped lock
      boost::lock_guard<boost::mutex> lock(captureMutex);

      if (!saveToAVIstarted) {
        if (videoWriter.open(aviFilename.c_str(), cv::VideoWriter::fourcc('I', 'Y', 'U', 'V')
                , 30, cv::Size(imgWidth, imgHeight), true)) {
          //videoWriter = cvCreateVideoWriter(aviFile.c_str(), CV_FOURCC('D', 'I', 'V', 'X'), 30, cvSize(imgWidth, imgHeight), 1);
          saveToAVIstarted = true;
        }
      }
    }

    void Capture::stopAviWrite() {
      //unique scoped lock
      boost::lock_guard<boost::mutex> lock(captureMutex);

      if (saveToAVIstarted) {
        //        videoWriter.release();
        saveToAVIstarted = false;
      }
    }

    void Capture::writeAvi() {
      if (videoWriter.isOpened() && saveToAVIstarted) {
        videoWriter.write(frame);
      }
    }

    void Capture::writeFrame(const std::string &rgbFilename) {
      //unique scoped lock
      boost::lock_guard<boost::mutex> lock(captureMutex);

      cv::imwrite(rgbFilename, frame);
    }

    void Capture::writePointCloud(const std::string &pcdFilename) {
      //unique scoped lock
      boost::lock_guard<boost::mutex> lock(captureMutex);

      if (cloudRGB && !cloudRGB->empty()) {
        pcl::io::savePCDFile(pcdFilename, *cloudRGB);
      } else if (!depthFrame.empty() && !frame.empty()) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        ade::pc::util::depthAndColorToPointCloud(depthFrame, frame, tmpCloud);
        pcl::io::savePCDFile(pcdFilename, *tmpCloud);
      } else {
        LOG4CXX_ERROR(logger, "[writePointCloud] can't write to file, not capturing RGB-D info.");
      }
    }

    void Capture::startOniWrite(const std::string &oniFilename) {
      boost::lock_guard<boost::mutex> lock(captureMutex);
      startOniImpl(oniFilename);
    }

    void Capture::stopOniWrite() {
      boost::lock_guard<boost::mutex> lock(captureMutex);
      stopOniImpl();
    }

    void Capture::startOniImpl(const std::string &oniFilename) {
      LOG4CXX_WARN(logger, "[startOniImpl] not implemented in base class.");
    }

    void Capture::stopOniImpl() {
      LOG4CXX_WARN(logger, "[stopOniImpl] not implemented in base class.");
    }

    void Capture::interruptWait() {
    }

    bool Capture::undistortFrame(const int cameraIndex) {
      bool success = false;

      //un-distort if requested and able
      if (undistortFlag) {
        CameraParameters::ConstPtr camParams = cameras->getCameraParameters(cameraIndex);

        if (camParams->intrinsicSet) {
          //get frame to un-distort
          cv::Mat frameToUndistort;
          if (cameraIndex == 0) {
            frameToUndistort = frame;
          } else if (cameraIndex == 1) {
            frameToUndistort = frame2;
          }

          //un-distort frame
          frameToUndistort.copyTo(tmpUndistortFrame);
          util::undistortFrame(tmpUndistortFrame, frameToUndistort, camParams);
          success = true;
        }
      }

      return success;
    }

    bool Capture::reconstructStereoFrames() {
      bool success = false;

      CameraParameters::ConstPtr camParams1 = cameras->getCameraParameters(0);
      CameraParameters::ConstPtr camParams2 = cameras->getCameraParameters(1);
      StereoCameraParameters::ConstPtr stereoParams1 = cameras->getStereoParameters(0);
      StereoCameraParameters::ConstPtr stereoParams2 = cameras->getStereoParameters(1);
      if (stereoParams1->set && stereoParams2->set) {
        if (depthFrame.empty()) depthFrame.create(frame.rows, frame.cols, CV_32FC1);
        if (depthFrame2.empty()) depthFrame2.create(frame.rows, frame.cols, CV_32FC1);
        util::generateDepthMaps(frame, frame2,
                                depthFrame, depthFrame2,
                                stereoParams1, stereoParams2,
                                camParams1, camParams2);

        success = true;
      }

      return success;
    }

  } //namespace capture
} //namespace ade
