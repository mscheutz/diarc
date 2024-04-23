/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "PeopleDetector.hpp"


#include <pcl/gpu/containers/initialization.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

PeopleDetector::PeopleDetector(const long long& moTypeId, const long long& processorId, const int imgWidth, const int imgHeight)
: ObjectDetector(moTypeId, processorId, imgWidth, imgHeight),
peopleDetector(new pcl::gpu::people::PeopleDetector()),
vis(new pcl::visualization::PCLVisualizer("people")) {
  // selecting GPU and prining info
  int device = 0;
  pcl::gpu::setDevice(device);
  pcl::gpu::printShortCudaDeviceInfo(device);
}

PeopleDetector::~PeopleDetector() {
}

void PeopleDetector::loadConfig(const std::string& config) {
  //selecting tree files
  //TODO:  pass in config file
  std::vector<std::string> tree_files;
  tree_files.push_back("Data/forest1/tree_20.txt");
  tree_files.push_back("Data/forest2/tree_20.txt");
  tree_files.push_back("Data/forest3/tree_20.txt");

  int num_trees = (int) tree_files.size();

  tree_files.resize(num_trees);
  if (num_trees == 0 || num_trees > 4) {
    std::cout << "Invalid number of trees" << std::endl;
    return;
  }

  try {
    // loading trees
    rdf = pcl::gpu::people::RDFBodyPartsDetector::Ptr(new pcl::gpu::people::RDFBodyPartsDetector(tree_files));
    peopleDetector->rdf_detector_ = rdf;

  } catch (const pcl::PCLException& e) {
    std::cout << "PCLException: " << e.detailedMessage() << std::endl;
  } catch (const std::runtime_error& e) {
    std::cout << e.what() << std::endl;
  } catch (const std::bad_alloc& /*e*/) {
    std::cout << "Bad alloc" << std::endl;
  } catch (const std::exception& /*e*/) {
    std::cout << "Exception" << std::endl;
  }
}

void PeopleDetector::handleCaptureNotification(CaptureNotification::ConstPtr notification) {
    const cv::Mat currFrame = notification->captureData->frame;
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudRaw = notification->captureData->cloud;

    //build xyzrgb cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloudRaw(new pcl::PointCloud<pcl::PointXYZRGB > ());
    colorCloudRaw->resize(cloudRaw->size());
    uint8_t* colorData = (uint8_t*) currFrame.data;
    for (int i = 0; i < cloudRaw->size(); ++i) {
      colorCloudRaw->points[i].x = cloudRaw->points[i].x;
      colorCloudRaw->points[i].y = cloudRaw->points[i].y;
      colorCloudRaw->points[i].z = cloudRaw->points[i].z;
      colorCloudRaw->points[i].r = colorData[i * 3];
      colorCloudRaw->points[i].g = colorData[i * 3 + 1];
      colorCloudRaw->points[i].b = colorData[i * 3 + 2];
    }

    //do the detection
    int processReturn = peopleDetector->process(colorCloudRaw);

    //display joint positions
    if (getDisplayFlag()) {
      static bool initialized = false;
      static bool joints_added[pcl::gpu::people::NUM_LABELS];
      if (!initialized) {
        for (int i = 0; i < pcl::gpu::people::NUM_LABELS; ++i) {
          joints_added[i] = false;
        }
        initialized = true;
      }
      const pcl::gpu::people::RDFBodyPartsDetector::BlobMatrix& blobMatrix = peopleDetector->rdf_detector_->getBlobMatrix();
      for (int i = 0; i < blobMatrix.size(); ++i) {
        int size = blobMatrix[i].size();
        std::stringstream ss;
        ss << i;
        std::string s(ss.str());
        if (size > 0) {
          cout << "blob " << i << " size: " << size << endl;
          //for (int j = 0; j < size; ++j) {
          //cout << blobMatrix[i][j] << endl;
          pcl::PointXYZ point(blobMatrix[i][0].mean(0), blobMatrix[i][0].mean(1), blobMatrix[i][0].mean(2));
          if (!joints_added[i]) {
            vis->addSphere(point, 0.025, s);
            joints_added[i] = true;
          } else {
            vis->updateSphere(point, 0.025, 255, 255, 255, s);
          }
          //}
        } else {
          if (joints_added[i]) {
            vis->removeShape(s);
            joints_added[i] = false;
          }
        }

        static bool cloud_added = false;
        if (!cloud_added) {
          vis->addPointCloud(colorCloudRaw);
          cloud_added = true;
        } else {
          vis->updatePointCloud(colorCloudRaw);
        }
        vis->spinOnce(1, true);
      }
    }
  }
