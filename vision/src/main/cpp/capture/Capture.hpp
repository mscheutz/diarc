/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef CAPTURE_HPP
#define CAPTURE_HPP

#include <opencv2/opencv.hpp>

#include "common/VisionConstants.hpp"
#include "calibration/Cameras.hpp"
#include "CaptureEnums.hpp"
#include "capture/firewire/FireWireHelper.hpp"      //for CAM_TYPE - is this necessary?
#include "visionproc/VisionProcess.hpp"
#include "common/notification/CaptureNotification.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <fstream>
#include <string>
#include <assert.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#ifdef USE_ROS_VISION
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#endif //USE_ROS_VISION

// include log4cxx header files.
#include <log4cxx/logger.h>

/** @brief Base class for all vision related capturing.  
 *
 * All capture devices need to
 * inherit from this class, overwrite captureCurrentCameraFrame, and make
 * the necessary modifications to Capture.cpp and cmake files.
 */

namespace diarc {
  namespace capture {

    class Capture {
    public:
      typedef boost::shared_ptr<Capture> Ptr;
      typedef boost::shared_ptr<const Capture> ConstPtr;

      //! Factory Method
      static Capture::Ptr createCaptureDevice(const std::string& configFile);

      //! register to be notified of capture notifications
      static void registerForNotification(VisionProcess::Ptr notifyTarget);
      
      //! unregister from capture notifications
      static void unregisterForNotification(VisionProcess::Ptr notifyTarget);
      
      //! for clients that don't require regular notification updates (i.e., just need one-time capture data)
      static CaptureNotification::ConstPtr getLastCaptureNotification();

      bool capture(const double transform[]);

      diarc::capture::CAM_MODE getCameraMode() const;
      int getImageHeight() const;
      int getImageWidth() const;
      bool hasStereo() const;
      bool hasDepth() const;

      //! display frames
      void setImageDisplayFlag(bool flag);
      void setDepthDisplayFlag(bool flag);

      //! should un-distort frames
      void setUndistortFlag(bool flag);
      bool getUndistortFlag() const;

      //! for saving RGB data to file (supported by all capture modes)
      void writeFrame(const std::string& rgbFilename);
      void startAviWrite(const std::string& aviFilename);
      void stopAviWrite();

      //! for saving RGB-D data to file (not supported by all capture modes)
      void writePointCloud(const std::string& pcdFilename);
      void startOniWrite(const std::string& oniFilename);
      void stopOniWrite();

      //! optional method to wake up waiting capture threads on shutdown 
      //! from notifications that might never come
      virtual void interruptWait();

      static log4cxx::LoggerPtr logger;

    protected:
      Capture(const std::string &configFile);
      virtual ~Capture();

      virtual bool captureCurrentCameraFrame() = 0;

      /** Helper methods for un-distort. Not guaranteed to perform un-distort,
       * depends on un-distort flag and camera parameters being set, as well as
       * captured frame existing.
       * @param cameraIndex 0 --> frame, 1 --> frame2
       * @return if un-distort was successful
       */
      bool undistortFrame(const int cameraIndex);

      /** Helper methods for stereo reconstruction. Not guaranteed to perform 
       * reconstruction, depends on camera parameters being set and both
       * frames 1 and 2 existing.
       * @return if un-distort was successful
       */
      bool reconstructStereoFrames();

      //! display RGB or RGB-D frames
      void displayImages();
      void displayDepth();

      //! if derived class can write RGB-D (oni file) it should override these methods
      //! this is currently only true for OpenNI capture mode
      virtual void startOniImpl(const std::string& oniFilename);
      virtual void stopOniImpl();
      
      //! total frame count so far - might be converted to timestamp in the future
      unsigned long long frameCount;

      // camera mode (i.e., capture type)
      CAM_MODE camMode;

      //! frames to be filled in by derived classes 
      cv::Mat frame, frame2;
      cv::Mat depthFrame, depthFrame2, depthFrameIntensity;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB;

      int imgWidth, imgHeight;
      bool stereoFlag;
      bool depthFlag;
      std::string cameraInfoFile;

      //! for regulating fps, only used by some derived classes
      unsigned long long lasttime;
      unsigned long long currenttime;
      unsigned long long timediff;
      struct timeval tv;

      //! un-distort flag
      bool undistortFlag;

      //! to get calibration parameters
      Cameras* cameras;

    private:
      //! doesn't make sense to copy this class - so disallow copy and assignment
      Capture(const Capture&);
      Capture& operator=(const Capture&);

      //! send capture notifications
      void sendNotifications(Notification::Ptr n);

      //! saving to video or "snapshot"
      void writeAvi();

      //! base frame to vision frame transform
      cv::Mat transformMat;

      //! for writing captured frames to AVIs
      cv::VideoWriter videoWriter, videoWriter2;
      bool saveToAVIstarted;

      //! for displaying in main capture method
      bool displayImagesFlag;
      bool displayDepthFlag;

      //! used as tmp storage for un-distorting frames
      cv::Mat tmpUndistortFrame;
      
      //! notification that gets filled and sent out every capture iteration
      //! static so that getLastCaptureNotification can return this notification
      static CaptureNotification::Ptr captureNotification;

      //! to lock when updating or using/getting capture data
      static boost::mutex captureMutex;

      //! all registered vision processors that should be notified of new capture data
      static std::vector<VisionProcess::Ptr> registeredProcessors;
      
      //! lock when adding/removing/notifying registeredProcessors
      static boost::mutex registeredProcsMutex;
      
#ifdef USE_ROS_VISION
      // ROS pub info
      bool publishCloudToRos;
      ros::NodeHandle* nh;
      ros::Publisher pub;
#endif //USE_ROS_VISION
    };

  } //namespace diarc
} //namespace capture

#endif
