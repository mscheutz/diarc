/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * Capture from PCD files (point cloud library data files).
 * @author Michael Zillich
 * @date March 2013
 */

#include <iostream>
#include "Capture.hpp"
#include "common/VisionConstants.hpp"
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/thread/thread.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace diarc {
  namespace capture {

    class CapturePCDFile : public Capture {
    public:

      CapturePCDFile(const std::string &configFile)
              : Capture(configFile) {

        // parse xml for configuration params
        using boost::property_tree::ptree;
        ptree pt;
        read_xml(configFile, pt);
        std::string directory = pt.get<std::string>("capture.directory");

        //init depth frame
        depthFrame.create(imgHeight, imgWidth, CV_32FC1);
        cloudRGB.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

        if (ConstructFilenames(directory, fileNames)) {
          std::cerr << "[ERROR] Check path to the input files for reading PCD files." << std::endl;
          fileNames.clear();
        }
        maxIndex = fileNames.size();
        currentIndex = 0;
      };

      bool captureCurrentCameraFrame() {
        if (maxIndex == 0)
          return (false);

        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(fileNames[currentIndex], *cloudRGB) == -1) {
          std::cerr << "CapturePCDFile: Failed to read file '" << fileNames[currentIndex] << "'\n";
          return (false);
        }
        if (!cloudRGB->isOrganized()) {
          std::cerr << "CapturePCDFile: we require organized point clouds, and the one in '"
                    << fileNames[currentIndex] << "' is not.\n";
          return (false);
        }

        currentIndex = (currentIndex + 1) % maxIndex;

        cv::Mat tmpFrame(cloudRGB->height, cloudRGB->width, CV_8UC3);
        cv::Mat tmpDepth(cloudRGB->height, cloudRGB->width, CV_32FC1);
        diarc::pc::util::pointCloudToDepthAndColor(tmpDepth, tmpFrame, cloudRGB);
        cv::resize(tmpFrame, frame, frame.size(), 0, 0, cv::INTER_LINEAR);
        cv::resize(tmpDepth, depthFrame, depthFrame.size(), 0, 0, cv::INTER_NEAREST);

        // sleep
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));

        return (true);
      }

    private:
      std::vector<std::string> fileNames;
      int maxIndex, currentIndex;

      int ConstructFilenames(const std::string &directoryName, std::vector<std::string> &fileNames) {
        fileNames.clear();
        boost::filesystem::path directory(directoryName);
        if (exists(directory)) {
          if (is_regular_file(directory)) {
            return (3);
          }
          if (is_directory(directory)) {
            for (boost::filesystem::directory_iterator end, it(directory); it != end; ++it) {
              if (is_regular_file(*it)) {
                fileNames.push_back(it->path().directory_string());
              }
            }
            std::sort(fileNames.begin(), fileNames.end());
            return (0);
          }
          return (1);
        } else {
          return (2);
        }
      }
    };

  } //namespace capture
} //namespace diarc
