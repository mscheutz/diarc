/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "DepthSensorCalibration.hpp"
#include "display/Display.hpp"

DepthSensorCalibration::DepthSensorCalibration(const int numCornersX, const int numCornersY,
                                               const enum CALIB_MODE calibmode)
        : cornersX(numCornersX), cornersY(numCornersY), calibrationMode(calibmode) {
  pointsPixel.resize(cornersX * cornersY);
}

bool DepthSensorCalibration::takeSnapshot() {
  //get images
  CaptureNotification::ConstPtr captureNotification = ade::capture::Capture::getLastCaptureNotification();
  if (!captureNotification) {
    // nothing captured yet
    return false;
  }

  const cv::Mat tempDepth = captureNotification->captureData->depthFrame;

  tempDepth.copyTo(img3d);

  if (calibrationMode == DepthSensorCalibration::KINECT) {
    cv::cvtColor(captureNotification->captureData->frame, imgIntensity, cv::COLOR_BGR2GRAY);
  } else if (calibrationMode == DepthSensorCalibration::SR) {
    captureNotification->captureData->depthFrameIntensity.copyTo(imgIntensity);
  }

  //checkerboard detection
  if (findCheckerboardPoints()) {
    //check that corresponding 3d data is not nan!
    //TODO: this should probably be generalized to check ALL 3d grid corners
    bool result = true;
    double x, y, z = 0;
    for (int i = 0; i < 4; ++i) {
      getDatum(i, x, y, z);
      printf("[DepthSensorCalibration] checking for NaN: %f %f %f.\n", x, y, z);

      if (x != x || y != y || z != z) {
        printf("[DepthSensorCalibration] NaN encountered.\n");
        result = false;
        break;
      }
    }
    printf("\n");
    return result;
  } else {
    return false;
  }
}

void DepthSensorCalibration::getDatum(int index, double &x, double &y, double &z) {
  int indexTo3dPoint = -1;
  switch (index) {
    case 0:
      //top left
      indexTo3dPoint = 0;
      break;
    case 1:
      //top right
      indexTo3dPoint = cornersX - 1;
      break;
    case 2:
      //bottom left
      indexTo3dPoint = cornersX * (cornersY - 1);
      break;
    case 3:
      //bottom right
      indexTo3dPoint = (cornersX * cornersY) - 1;
      break;
  }

  //printf("indexTo3dPoint: %d point3d size: %d\n", indexTo3dPoint, points3d.size());
  if (indexTo3dPoint + 1 <= points3d.size()) {
    x = points3d[indexTo3dPoint].x;
    y = points3d[indexTo3dPoint].y;
    z = points3d[indexTo3dPoint].z;
  } else {
    x = y = z = 0;
  }
}

bool DepthSensorCalibration::findCheckerboardPoints() {
  int cornersDetected = 0;
  int numCorners = cornersX * cornersY;

  //clear old data
  //pointsPixel.clear();
  points3d.clear();

  //FIND CHESSBOARDS AND CORNERS:
  int result = cv::findChessboardCorners(
          imgIntensity, cv::Size(cornersX, cornersY),
          pointsPixel,
          cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
  );

  if (result && cornersDetected == numCorners) {
    //Calibration will suffer without sub-pixel interpolation
    cv::find4QuadCornerSubpix(
            imgIntensity, pointsPixel,
            cv::Size(3, 3)
    );

    //fill 3d points based on 2d pixel locations
    points3d.reserve(numCorners);
    float *img3d_data = (float *) img3d.data;
    int cornerIndex = -1;
    for (int i = 0; i < numCorners; ++i) {
      cornerIndex = static_cast<int> (pointsPixel[i].x) + static_cast<int> (pointsPixel[i].y) * imgIntensity.cols;
      cv::Point3d point3d;
      point3d.x = img3d_data[cornerIndex * 3 + 0];
      point3d.y = img3d_data[cornerIndex * 3 + 1];
      point3d.z = img3d_data[cornerIndex * 3 + 2];
      points3d.push_back(point3d);
    }

    //display 
    display3d();
    display2d();

    return true;
  } else {
    printf("DepthSensorCalibration failed to detect corners.\n");
    return false;
  }
}

void DepthSensorCalibration::display2d() {
  std::string windowName = "DepthSensorCalibration";
  ade::Display::createWindowIfDoesNotExist(windowName);
  imgIntensity.copyTo(frameToDraw);
  for (int i = 0; i < pointsPixel.size(); ++i) {
    cv::circle(frameToDraw, cv::Point(pointsPixel[i].x, pointsPixel[i].y),
             2, CV_RGB(255, 255, 255), 1, 8, 0);
  }
  ade::Display::displayFrame(frameToDraw, windowName);
}

void DepthSensorCalibration::display3d() {
  //visualization
  //TODO: this is temporary - need to implement a more permanent display mechanism!!
  if (boost::shared_ptr<pcl::visualization::PCLVisualizer>() == viewer) {
    viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(
            new pcl::visualization::PCLVisualizer("Checkerboard Detector"));
    viewer->initCameraParameters();
    viewer->addCoordinateSystem(0.1, "calibration", 0);
  }

  //entire point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>());

  float *img3d_data = (float *) img3d.data;
  const int imgWidth = img3d.cols;
  const int imgHeight = img3d.rows;

  point_cloud->width = imgWidth;
  point_cloud->height = imgHeight;
  point_cloud->is_dense = false;
  point_cloud->points.resize(point_cloud->width * point_cloud->height);

  for (int xx = 0; xx < imgWidth; xx++) {
    for (int yy = 0; yy < imgHeight; yy++) {
      point_cloud->points[xx + yy * imgWidth].x = img3d_data[(xx + yy * imgWidth) * 3 + 0];
      point_cloud->points[xx + yy * imgWidth].y = img3d_data[(xx + yy * imgWidth) * 3 + 1];
      point_cloud->points[xx + yy * imgWidth].z = img3d_data[(xx + yy * imgWidth) * 3 + 2];
    }
  }
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_point_cloud(point_cloud, 255, 255, 255);
  viewer->addPointCloud(point_cloud, handler_point_cloud, "capture");


  // all checkerboard corners ///////////////////////////////////////////////
  pcl::PointCloud<pcl::PointXYZ>::Ptr all_corner_points(new pcl::PointCloud<pcl::PointXYZ>());

  const int numPoints = points3d.size();

  //    corner_points->width = imgWidth;
  //    corner_points->height = imgHeight;
  //    corner_points->is_dense = false;
  all_corner_points->points.resize(numPoints);

  for (int i = 0; i < numPoints; i++) {
    all_corner_points->points[i].x = points3d[i].x;
    all_corner_points->points[i].y = points3d[i].y;
    all_corner_points->points[i].z = points3d[i].z;
  }
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_all_corner_points(all_corner_points, 0, 255,
                                                                                            0);

  viewer->addPointCloud(all_corner_points, handler_all_corner_points, "all_corners");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "all_corners");

  //4 checkerboard corners of interest ///////////////////////////////////////
  pcl::PointCloud<pcl::PointXYZ>::Ptr corner_points(new pcl::PointCloud<pcl::PointXYZ>());

  //    corner_points->width = imgWidth;
  //    corner_points->height = imgHeight;
  //    corner_points->is_dense = false;
  corner_points->points.resize(4);

  //top left
  int i = 0;
  corner_points->points[0].x = points3d[i].x;
  corner_points->points[0].y = points3d[i].y;
  corner_points->points[0].z = points3d[i].z;

  //top right
  i = cornersX - 1;
  corner_points->points[1].x = points3d[i].x;
  corner_points->points[1].y = points3d[i].y;
  corner_points->points[1].z = points3d[i].z;

  //bottom left
  i = cornersX * (cornersY - 1);
  corner_points->points[2].x = points3d[i].x;
  corner_points->points[2].y = points3d[i].y;
  corner_points->points[2].z = points3d[i].z;

  //bottom right
  i = (cornersX * cornersY) - 1;
  corner_points->points[3].x = points3d[i].x;
  corner_points->points[3].y = points3d[i].y;
  corner_points->points[3].z = points3d[i].z;

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_corner_points(corner_points, 255, 0, 0);
  viewer->addPointCloud(corner_points, handler_corner_points, "corners");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "corners");

  //    if (!viewer->wasStopped()) {
  //        //allow one opportinity to orient pcl viewer
  //        while (!viewer->wasStopped()) {
  //            viewer->spinOnce();
  //        }
  //    } else {
  //        viewer->spinOnce();
  //    }

  viewer->spinOnce();

  viewer->removePointCloud("capture");
  viewer->removePointCloud("all_corners");
  viewer->removePointCloud("corners");
  //viewer->resetStoppedFlag();


  //viewer.reset();
}
