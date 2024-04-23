/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "Capture.hpp"
#include "common/VisionConstants.hpp"
#include "point_clouds/PointCloudUtilities.hpp"

#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>

namespace ade {
  namespace capture {

    class CaptureSimulateKinect : public Capture {
    public:

      CaptureSimulateKinect(const std::string &configFile)
              : Capture(configFile) {

        // parse xml for configuration params
        using boost::property_tree::ptree;
        ptree pt;
        read_xml(configFile, pt);

        //init depth frame
        depthFrame.create(imgHeight, imgWidth, CV_32FC1);

        switch (camMode) {
          case SIM_PC1: {
            // read
            std::string dirName = pt.get<std::string>("capture.dirNamCV_e");

            disparityDirectory = dirName + "/disparity/";
            colorDirectory = dirName + "/color/";

            if (ReadFiles(disparityDirectory, disparityNames) ||
                ReadFiles(colorDirectory, colorNames)
                ) {
              LOG4CXX_ERROR(logger, "[CaptureSimulateKinect] Check path to the input files for simulation mode..");
              disparityNames.clear();
              colorNames.clear();
            }
            maxIndex = std::max(disparityNames.size(), colorNames.size());
            currentIndex = 0;
            break;
          }
          case SIM_PC2: {
            tmpColorCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

            pcdFilename = pt.get<std::string>("capture.pcdFilename");
            rgbFilename = pt.get<std::string>("capture.rgbFilename", "");
            break;
          }
        }

        Cameras *cameras = Cameras::getInstance();

        //set depth image parameters
        CameraParameters::Ptr camParams(new CameraParameters(VisionConstants::imgWidth, VisionConstants::imgHeight));
        double depth_fl = 575.0;
        float fx = depth_fl * (VisionConstants::imgWidth / 640.0);
        camParams->M.at<float>(0, 0) = fx;
        float fy = depth_fl * (VisionConstants::imgHeight / 480.0);
        camParams->M.at<float>(1, 1) = fy;
        float cx = VisionConstants::imgWidth / 2.0;
        camParams->M.at<float>(0, 2) = cx;
        float cy = VisionConstants::imgHeight / 2.0;
        camParams->M.at<float>(1, 2) = cy;
        camParams->intrinsicSet = true;
        cameras->setCameraParameters(0, camParams);

        //set rgb frame parameters
        camParams = CameraParameters::Ptr(new CameraParameters(VisionConstants::imgWidth, VisionConstants::imgHeight));
        double image_fl = 525.0;
        float fx2 = image_fl * (VisionConstants::imgWidth / 640.0);
        camParams->M.at<float>(0, 0) = fx2;
        float fy2 = image_fl * (VisionConstants::imgHeight / 480.0);
        camParams->M.at<float>(1, 1) = fy2;
        float cx2 = VisionConstants::imgWidth / 2.0;
        camParams->M.at<float>(0, 2) = cx2;
        float cy2 = VisionConstants::imgHeight / 2.0;
        camParams->M.at<float>(1, 2) = cy2;
        camParams->intrinsicSet = true;
        cameras->setCameraParameters(1, camParams);

      };

      ~CaptureSimulateKinect() {
      };

      bool captureCurrentCameraFrame() {
        switch (camMode) {
          case SIM_PC1:
            return captureSimPC1();
          case SIM_PC2:
            return captureSimPC2();
        }
        return false;
      }

    private:
      //SIM_PC1
      cv::Mat colorImg, disparityImg, depthImg;
      std::string disparityDirectory, colorDirectory;
      std::vector<std::string> disparityNames, colorNames;
      int maxIndex, currentIndex;

      //SIM_PC2
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpColorCloud;
      cv::Mat tmpColor, tmpDepth;
      std::string pcdFilename;
      std::string rgbFilename;

      bool captureSimPC1() {
        if (!maxIndex)
          return (false);

        disparityImg = cv::imread(disparityNames.at(currentIndex), -1);
        colorImg = cv::imread(colorNames.at(currentIndex), -1);
        currentIndex = (currentIndex + 1) % maxIndex;
        Disparity2Depth();

        cv::Mat tempMat1 = frame;
        cv::resize(colorImg, tempMat1, tempMat1.size(), 0, 0, cv::INTER_LINEAR);
        cv::Mat tempMat2 = depthFrame;
        cv::resize(depthImg, tempMat2, tempMat2.size(), 0, 0, cv::INTER_NEAREST);
        // sleep
        boost::this_thread::sleep(boost::posix_time::milliseconds(25));

        return (true);
      }

      bool captureSimPC2() {
        //no need to load the same file over and over
        if (tmpColorCloud->empty()) {
          if (pcl::io::loadPCDFile(pcdFilename, *tmpColorCloud) == -1) {
            LOG4CXX_ERROR(logger, boost::format("[captureSimPC2] Couldn't read file: %s.") % pcdFilename);
            return false;
          }
        }

        //resize pointcloud (if necessary) and fill rgb and depth frames
        if (tmpColorCloud->width != imgWidth || tmpColorCloud->height != imgHeight) {
          //allocate tmp images
          if (tmpColor.empty() || tmpDepth.empty()) {
            tmpColor.create(tmpColorCloud->height, tmpColorCloud->width, CV_8UC3);
            tmpDepth.create(tmpColorCloud->height, tmpColorCloud->width, CV_32FC1);
          }
          //fill tmp color and depth from point cloud and resize
          ade::pc::util::pointCloudToDepthAndColor(tmpDepth, tmpColor, tmpColorCloud);
          cv::resize(tmpColor, frame, frame.size(), 0, 0, cv::INTER_LINEAR);
          cv::resize(tmpDepth, depthFrame, depthFrame.size(), 0, 0, cv::INTER_NEAREST);

          //fill frame containers with resized data
          ade::pc::util::depthAndColorToPointCloud(depthFrame, frame, cloudRGB, cloud);
        } else {
          //fill frame containers
          cloudRGB = tmpColorCloud;
          ade::pc::util::pointCloudToDepthAndColor(depthFrame, frame, cloudRGB);
          pcl::copyPointCloud(*cloudRGB, *cloud);
        }

        // if rgbFilename has been specified, overwrite rgb image with one loaded from file
        // this is inefficient to do it here and reload from file every time, but this class is just for testing so it should be fine
        if (!rgbFilename.empty()) {
          tmpColor = cv::imread(rgbFilename);
          cv::resize(tmpColor, frame, frame.size(), 0, 0, cv::INTER_LINEAR);
          if (tmpColor.empty()) {
            LOG4CXX_ERROR(logger, boost::format("[captureSimPC2] Couldn't read file: %s.") % rgbFilename);
            return false;
          }
        }

        // sleep
        boost::this_thread::sleep(boost::posix_time::milliseconds(5));

        return true;
      }

      void Disparity2Depth() {
        float f = 525;
        float b = 0.075;
        depthImg = cv::Mat_<float>::zeros(disparityImg.rows, disparityImg.cols);
        for (int i = 0; i < depthImg.rows; ++i) {
          for (int j = 0; j < depthImg.cols; ++j) {
            uchar disp = disparityImg.at<uchar>(i, j);
            if (disp == 255) {
              continue;
            }
            float z = b * f / ((float) disp + 28);
            depthImg.at<float>(i, j) = z;
          }
        }
      }

      int ReadFiles(const std::string &directory_name, std::vector<std::string> &names) {
        names.clear();
        boost::filesystem::path directory(directory_name);
        if (exists(directory)) {
          if (is_regular_file(directory)) {
            //names.push_back(directory_name);
            return (3);
          }
          if (is_directory(directory)) {
            for (boost::filesystem::directory_iterator end, it(directory); it != end; ++it) {
              if (is_regular_file(*it)) {
                names.push_back(it->path().directory_string());
              }
            }
            std::sort(names.begin(), names.end());
            return (0);
          }
          return (1);
        } else
          return (2);
      }
    };

  } //namespace capture
} //namespace ade
