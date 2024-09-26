/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "Capture.hpp"
#include "calibration/Cameras.hpp"
#include "point_clouds/PointCloudUtilities.hpp"

#include <boost/format.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <pcl/io/oni_grabber.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/openni2_grabber.h>
#include <functional> // for boost:bind --> update to lambda at some point

namespace diarc {
  namespace capture {

    class CaptureOpenni2 : public Capture {
    public:

      CaptureOpenni2(const std::string &configFile)
              : Capture(configFile),
                grabber(),
                colorImg(imgHeight, imgWidth, CV_8UC3),
                depthImg(imgHeight, imgWidth, CV_32FC1),
                new_data(false),
                data_mutex(),
                condition() {

          initOpenni2Grabber();

        // set callbacks
#if PCL_VERSION_COMPARE(>=, 1, 12, 0)
        std::function<void(
                const boost::shared_ptr<pcl::io::openni2::Image> &,
                const boost::shared_ptr<pcl::io::openni2::DepthImage> &, float constant)> f =
                std::bind(&CaptureOpenni2::imageDepthImageCallback, this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
#else
        boost::function<void(
                const boost::shared_ptr<pcl::io::openni2::Image> &,
                const boost::shared_ptr<pcl::io::openni2::DepthImage> &, float constant)> f =
                boost::bind(&CaptureOpenni2::imageDepthImageCallback, this, _1, _2, _3);
#endif

        grabber->registerCallback(f);

        // start the grabber
        grabber->start();

      };

      ~CaptureOpenni2() {
        delete grabber;
      };

      void initOpenni2Grabber() {
        pcl::io::OpenNI2Grabber *openni_grabber = new pcl::io::OpenNI2Grabber();

#if PCL_VERSION_COMPARE(>=, 1, 12, 0)
        std::shared_ptr<pcl::io::openni2::OpenNI2Device> device_ptr = openni_grabber->getDevice();
#else
        boost::shared_ptr<pcl::io::openni2::OpenNI2Device> device_ptr = openni_grabber->getDevice();
#endif
        grabber = openni_grabber;

        //set depth image parameters
        CameraParameters::Ptr camParams(new CameraParameters(VisionConstants::imgWidth, VisionConstants::imgHeight));

        // scale to current vision width and height
        float fx = device_ptr->getDepthFocalLength() * (imgWidth / 640.0);
        camParams->M.at<float>(0, 0) = fx;
        float fy = fx;
        camParams->M.at<float>(1, 1) =fy;
        float cx = (imgWidth - 1.0) / 2.0;
        camParams->M.at<float>(0, 2) =cx;
        float cy = ( imgHeight - 1.0) / 2.0;
        camParams->M.at<float>(1, 2) =cy;
        camParams->intrinsicSet = true;
        cameras->setCameraParameters(0, camParams);
        LOG4CXX_INFO(logger,
                     boost::format("[initOpenniGrabber] depth camera params: %f, %f, %f, %f") % fx % fy % cx % cy);

        //set rgb frame parameters
        camParams = CameraParameters::Ptr(new CameraParameters(VisionConstants::imgWidth, VisionConstants::imgHeight));

        float fx2 = device_ptr->getColorFocalLength() * (imgWidth / 640.0);
        camParams->M.at<float>(0, 0) = fx2;
        float fy2 = fx2;
        camParams->M.at<float>(1, 1) = fy2;
        float cx2 = (imgWidth - 1.0) / 2.0;
        camParams->M.at<float>(0, 2) = cx2;
        float cy2 = (imgHeight - 1.0) / 2.0;
        camParams->M.at<float>(1, 2) = cy2;
        camParams->intrinsicSet = true;
        cameras->setCameraParameters(1, camParams);
        LOG4CXX_INFO(logger,
                     boost::format("[initOpenniGrabber] rgb camera params: %f, %f, %f, %f") % fx2 % fy2 % cx2 % cy2);
      }

      void imageDepthImageCallback(const boost::shared_ptr<pcl::io::openni2::Image> &image,
                                   const boost::shared_ptr<pcl::io::openni2::DepthImage> &depth_image, float constant) {
        boost::mutex::scoped_lock lock(data_mutex);

        // copy rgb image data
        image->fillRGB(colorImg.cols, colorImg.rows, colorImg.data, colorImg.step);

        // copy depth image data
        depth_image->fillDepthImage(depthImg.cols, depthImg.rows, (float *) depthImg.data, depthImg.step);

        // notify waiting threads of new data (ie., captureCurrentCameraFrame)
        new_data = true;
        condition.notify_one();
      }

      bool captureCurrentCameraFrame() {
        boost::mutex::scoped_lock lock(data_mutex);

        // wait for new data
        while (!new_data) {
          condition.wait(lock);
        }
        new_data = false;

        //        colorImg.copyTo(frame);
        cv::cvtColor(colorImg, frame, cv::COLOR_BGR2RGB);
        depthImg.copyTo(depthFrame);

        diarc::pc::util::depthAndColorToPointCloud(depthFrame, frame, cloudRGB, cloud);

        return true;
      }

      virtual void startOniImpl(const std::string &oniFilename) {
        LOG4CXX_WARN(logger, "[startOniImpl] temporarily unavailable.");
        //        LOG4CXX_DEBUG(logger, "[startOniImpl] called.");
      }

      virtual void stopOniImpl() {
        LOG4CXX_WARN(logger, "[stopOniImpl] temporarily unavailable.");
        //        LOG4CXX_DEBUG(logger, "[stopOniImpl] called.");
      }

    private:
      pcl::Grabber *grabber;
      cv::Mat colorImg, depthImg;
      bool new_data;
      boost::mutex data_mutex;
      boost::condition_variable condition;
    };

  } //namespace capture
} //namespace diarc
