/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include <point_clouds/PointCloudUtilities.hpp>
#include <stdint.h>
#include <opencv2/calib3d/calib3d.hpp>
#include "capture/util/CaptureUtilities.hpp"
#include "PCLFunctions.hpp"
#include <pcl/features/moment_of_inertia_estimation.h>

namespace ade {
  namespace pc {
    namespace util {

      log4cxx::LoggerPtr logger(log4cxx::Logger::getLogger("ade.pc.util.PointCloudUtilities"));

      void xyzImageToPointCloud(const cv::Mat xyzImage, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
              pcl::PointIndices::Ptr indices) {

        const int imgWidth = xyzImage.cols;
        const int imgHeight = xyzImage.rows;
        const int channels = xyzImage.channels();
        if (channels != 3) {
          LOG4CXX_WARN(logger, "[xyzImageToPointCloud]: incorrect xyzData format! Returning.");
          return;
        }

        cloud->width = imgWidth;
        cloud->height = imgHeight;
        cloud->is_dense = false;
        cloud->points.resize(cloud->width * cloud->height);

        float* xyzData = (float*) xyzImage.data;
        for (int xx = 0; xx < imgWidth; xx++) {
          for (int yy = 0; yy < imgHeight; yy++) {
            float x_wc = xyzData[(xx + imgWidth * yy)*3];
            float y_wc = xyzData[(xx + imgWidth * yy)*3 + 1];
            float z_wc = xyzData[(xx + imgWidth * yy)*3 + 2];
            //if (x_wc > 0 || y_wc > 0 || z_wc > 0)
#if defined FREENECT || defined USE_OPENNI
            if (z_wc >= 0.4)
#else
            if (z_wc > 0)
#endif
            {
              cloud->points[xx + yy * imgWidth].x = x_wc;
              cloud->points[xx + yy * imgWidth].y = y_wc;
              cloud->points[xx + yy * imgWidth].z = z_wc;
            } else {
              //printf("Cloud has NaN!\n");
              cloud->points[xx + yy * imgWidth].x = std::numeric_limits<float>::quiet_NaN();
              cloud->points[xx + yy * imgWidth].y = std::numeric_limits<float>::quiet_NaN();
              cloud->points[xx + yy * imgWidth].z = std::numeric_limits<float>::quiet_NaN();
            }
          }
        }

        // create useful indices
        FilterPointCloud<pcl::PointXYZ>(cloud, indices);
      }

      void depthToPointCloud(const cv::Mat depth, CameraParameters::ConstPtr irParams,
              pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr indices) {

        if (3 == depth.channels()) {
          xyzImageToPointCloud(depth, cloud, indices);
          return;
        }

        if (!irParams->intrinsicSet) {
          LOG4CXX_WARN(logger, "[depthToPointCloud]: camera params not set! Returning.");
          return;
        }
        const float fx = irParams->M.at<float>(0, 0);
        const float fy = irParams->M.at<float>(1, 1);
        const float cx = irParams->M.at<float>(0, 2);
        const float cy = irParams->M.at<float>(1, 2);
        const int imgWidth = depth.cols;
        const int imgHeight = depth.rows;

        float* depthData = (float*) depth.data;

        cloud->width = imgWidth;
        cloud->height = imgHeight;
        cloud->is_dense = false;
        cloud->points.resize(cloud->width * cloud->height);

        for (int xx = 0; xx < imgWidth; xx++) {
          for (int yy = 0; yy < imgHeight; yy++) {
            float z_ir = depthData[xx + imgWidth * yy];
            float x_ir, y_ir;
#if defined FREENECT || defined USE_OPENNI
            if (z_ir >= 0.4)
#else
            if (z_ir > 0)
#endif
            {
              x_ir = ((xx - cx) / fx) * z_ir;
              y_ir = ((yy - cy) / fy) * z_ir;
            } else {
              x_ir = std::numeric_limits<float>::quiet_NaN();
              y_ir = std::numeric_limits<float>::quiet_NaN();
              z_ir = std::numeric_limits<float>::quiet_NaN();
            }
            cloud->points[xx + yy * imgWidth].x = x_ir;
            cloud->points[xx + yy * imgWidth].y = y_ir;
            cloud->points[xx + yy * imgWidth].z = z_ir;
          }
        }

        // create useful indices
        FilterPointCloud<pcl::PointXYZ>(cloud, indices);
      }

      void xyzAndColorToPointCloud(const cv::Mat xyzImage, const cv::Mat color,
              pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud,
              pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        const int imgWidth = xyzImage.cols;
        const int imgHeight = xyzImage.rows;
        const int channels = xyzImage.channels();
        if (channels != 3) {
          LOG4CXX_WARN(logger, "[xyzAndColorToPointCloud]: incorrect xyzData format! Returning.");
          return;
        }

        //    Cameras* cameras = Cameras::getInstance();
        //    CameraParameters::ConstPtr camParams1 = cameras->getCameraParameters(0);
        //    CameraParameters::ConstPtr camParams2 = cameras->getCameraParameters(1);
        //    StereoCameraParameters::ConstPtr stereoParams1 = cameras->getStereoParameters(0);
        //    StereoCameraParameters::ConstPtr stereoParams2 = cameras->getStereoParameters(1);
        //
        //    //project 3D point onto color image to get pixel location
        //    //calc rotation
        //    cv::Mat rVec = (cv::Mat_<double>(1,3) << 0, 0, 0);
        //    cv::Mat rMat(3, 3, CV_64F);
        //    cv::Mat rMat1(3, 3, CV_64F);
        //    cv::Mat rMat2(3, 3, CV_64F);
        //    cv::Rodrigues(camParams1->rVec, rMat1);
        //    cv::Rodrigues(camParams2->rVec, rMat2);
        //    rMat =  rMat1.inv() * rMat2;   //rMat2.t() * rMat1;
        //    cv::Rodrigues(rMat, rVec);
        //    printf("Stereo R: \n%f %f %f\n%f %f %f\n %f %f %f\n", rMaT.at<float>(0, 0), rMaT.at<float>(0, 1), rMaT.at<float>(0, 2),
        //            rMaT.at<float>(1, 0), rMaT.at<float>(1, 1), rMaT.at<float>(1, 2),
        //            rMaT.at<float>(2, 0), rMaT.at<float>(2, 1), rMaT.at<float>(2, 2));
        //    //calc translation
        //    cv::Mat tVec = cv::Mat(camParams1->tVec) - cv::Mat(camParams2->tVec);
        //    printf("Stereo T: \n%f %f %f\n", tVec.at<double>(0, 0), tVec.at<double>(0, 1), tVec.at<double>(0, 2));
        //    cv::Mat objectPoints(xyzImage, true);
        //    
        //    //fill 3D object points to project onto image plane
        //    //objectPoints.reshape(0, objectPoints.rows*objectPoints.cols);
        //    int newRows = objectPoints.rows*objectPoints.cols;
        //    objectPoints.rows = newRows;
        //    objectPoints.cols = 1;
        //    //printf("cont: %s depth: %d chan: %d rows: %d cols: %d\n", (objectPoints.isContinuous()?"true":"false"), objectPoints.depth(), objectPoints.channels(), objectPoints.rows, objectPoints.cols);
        //    std::vector<cv::Point2f> imgPoints;
        //    cv::projectPoints(objectPoints, rVec, tVec, camParams1->M, camParams1->D, imgPoints);

        //fill 3D object points to project onto image plane
        std::vector<cv::Point2f> imgPoints;
        cv::Mat objectPoints = xyzImage;
        //objectPoints.reshape(0, objectPoints.rows*objectPoints.cols);
        int newRows = objectPoints.rows * objectPoints.cols;
        objectPoints.rows = newRows;
        objectPoints.cols = 1;
        //printf("cont: %s depth: %d chan: %d rows: %d cols: %d\n", (objectPoints.isContinuous()?"true":"false"), objectPoints.depth(), objectPoints.channels(), objectPoints.rows, objectPoints.cols);

        ade::capture::util::projectPoints(objectPoints, imgPoints, 0);


        //fill in point cloud
        color_cloud->width = imgWidth;
        color_cloud->height = imgHeight;
        color_cloud->is_dense = false;
        color_cloud->points.resize(color_cloud->width * color_cloud->height);

        if (cloud) {
          cloud->width = imgWidth;
          cloud->height = imgHeight;
          cloud->is_dense = false;
          cloud->points.resize(cloud->width * cloud->height);
        }

        float* xyzData = (float*) xyzImage.data;
        uint8_t* colorData = (uint8_t*) color.data;
        for (int i = 0; i < imgWidth * imgHeight; ++i) {
          float x_wc = xyzData[i * 3];
          float y_wc = xyzData[i * 3 + 1];
          float z_wc = xyzData[i * 3 + 2];
          //if (x_wc > 0 || y_wc > 0 || z_wc > 0)
#if defined FREENECT || defined USE_OPENNI
          if (z_wc >= 0.4)
#else
          if (z_wc > 0)
#endif
          {
            x_wc = xyzData[i * 3];
            y_wc = xyzData[i * 3 + 1];
          } else {
            //printf("Cloud has NaN!\n");
            x_wc = std::numeric_limits<float>::quiet_NaN();
            y_wc = std::numeric_limits<float>::quiet_NaN();
            z_wc = std::numeric_limits<float>::quiet_NaN();
          }

          color_cloud->points[i].x = x_wc;
          color_cloud->points[i].y = y_wc;
          color_cloud->points[i].z = z_wc;

          if (cloud) {
            cloud->points[i].x = x_wc;
            cloud->points[i].y = y_wc;
            cloud->points[i].z = z_wc;
          }


          //add color from image, if possible
          uint8_t b = 0;
          uint8_t g = 255;
          uint8_t r = 0;
          int colorWidth = color.cols;
          int colorHeight = color.rows;
          int x = imgPoints[i].x;
          int y = imgPoints[i].y;
          if (x >= 0 && x < colorWidth && y >= 0 && y < colorHeight) {
            //printf("color pixel %d loc: %d %d\n", i, x, y);
            b = colorData[(x + y * colorWidth) * 3];
            g = colorData[(x + y * colorWidth) * 3 + 1];
            r = colorData[(x + y * colorWidth) * 3 + 2];
          }
          uint32_t rgb = ((uint32_t) r << 16 | (uint32_t) g << 8 | (uint32_t) b);
          color_cloud->points[i].rgb = *reinterpret_cast<float*> (&rgb); // pack r/g/b into rgb
        }

      }

      void depthAndColorToPointCloud(const cv::Mat depth, const cv::Mat color,
              pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud,
              pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        if (3 == depth.channels()) {
          xyzAndColorToPointCloud(depth, color, color_cloud, cloud);
          return;
        }

        //using "default" kinect params
        const int imgWidth = depth.cols;
        const int imgHeight = depth.rows;

        // get depth frame camera matrix
        Cameras* cameras = Cameras::getInstance();
        CameraParameters::ConstPtr camParams = cameras->getCameraParameters(0);
        float fx, fy, cx, cy = 0.0;
        if (camParams->intrinsicSet) {
          fx = camParams->M.at<float>(0, 0);
          fy = camParams->M.at<float>(1, 1);
          cx = camParams->M.at<float>(0, 2);
          cy = camParams->M.at<float>(1, 2);
        } else {
          //using "default" kinect params
          fx = 580.0 * (imgWidth / 640.0);
          fy = 580.0 * (imgHeight / 480.0);
          cx = imgWidth / 2.0;
          cy = imgHeight / 2.0;
        }

        float* depthData = (float*) depth.data;
        uint8_t* colorData = (uint8_t*) color.data;

        color_cloud->width = imgWidth;
        color_cloud->height = imgHeight;
        color_cloud->is_dense = false;
        color_cloud->points.resize(color_cloud->width * color_cloud->height);

        if (cloud) {
          cloud->width = imgWidth;
          cloud->height = imgHeight;
          cloud->is_dense = false;
          cloud->points.resize(cloud->width * cloud->height);
        }

        for (int xx = 0; xx < imgWidth; xx++) {
          for (int yy = 0; yy < imgHeight; yy++) {
            float z_ir = depthData[xx + imgWidth * yy];
            float x_ir, y_ir;
#ifdef FREENECT
            if (z_ir >= 0.4)
#else
            if (z_ir > 0)
#endif
            {
              x_ir = ((xx - cx) / fx) * z_ir;
              y_ir = ((yy - cy) / fy) * z_ir;
            } else {
              x_ir = std::numeric_limits<float>::quiet_NaN();
              y_ir = std::numeric_limits<float>::quiet_NaN();
              z_ir = std::numeric_limits<float>::quiet_NaN();
            }
            color_cloud->points[xx + yy * imgWidth].x = x_ir;
            color_cloud->points[xx + yy * imgWidth].y = y_ir;
            color_cloud->points[xx + yy * imgWidth].z = z_ir;

            if (cloud) {
              cloud->points[xx + yy * imgWidth].x = x_ir;
              cloud->points[xx + yy * imgWidth].y = y_ir;
              cloud->points[xx + yy * imgWidth].z = z_ir;
            }

            uint8_t b = colorData[(xx + yy * imgWidth)*3];
            uint8_t g = colorData[(xx + yy * imgWidth)*3 + 1];
            uint8_t r = colorData[(xx + yy * imgWidth)*3 + 2];
            uint32_t rgb = ((uint32_t) r << 16 | (uint32_t) g << 8 | (uint32_t) b);
            color_cloud->points[xx + yy * imgWidth].rgb = *reinterpret_cast<float*> (&rgb); // pack r/g/b into rgb
          }
        }
      }

      void pointCloudToDepthAndColor(cv::Mat depth, cv::Mat color, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {

        //image size check
        if (depth.cols != cloud->width || depth.rows != cloud->height
                || color.cols != cloud->width || color.rows != cloud->height) {
          LOG4CXX_WARN(logger, "[pointCloudToDepthAndColor]: inconsistent sizes! Returning.");
          return;
        }

        //using "default" kinect params
        const int imgWidth = cloud->width;
        const int imgHeight = cloud->height;

        //TODO: use CameraParameters
        //        const float fx = 580.0 * (imgWidth / 640.0);
        //        const float fy = 580.0 * (imgHeight / 480.0);
        //        const float cx = imgWidth / 2.0;
        //        const float cy = imgHeight / 2.0;

        cv::Mat colorImage = cv::Mat(imgHeight,imgWidth, CV_8UC3);
        cv::Mat depthImage = cv::Mat(imgHeight, imgWidth, CV_32FC1);
        float* depthData = (float*) depthImage.data;
        uint8_t* colorData = (uint8_t*) colorImage.data;
        float x_ir, y_ir, z_ir;
        uint8_t b, g, r;

        for (int xx = 0; xx < imgWidth; xx++) {
          for (int yy = 0; yy < imgHeight; yy++) {

            x_ir = cloud->points[xx + yy * imgWidth].x;
            y_ir = cloud->points[xx + yy * imgWidth].y;
            z_ir = cloud->points[xx + yy * imgWidth].z;

            if (std::isnan(x_ir) || std::isnan(y_ir) || std::isnan(z_ir)) {
              depthData[xx + imgWidth * yy] = std::numeric_limits<float>::quiet_NaN();
              colorData[(xx + yy * imgWidth)*3] = 0;
              colorData[(xx + yy * imgWidth)*3 + 1] = 0;
              colorData[(xx + yy * imgWidth)*3 + 2] = 0;
            } else {
              //assumes an organized point cloud
              depthData[xx + imgWidth * yy] = z_ir;

              r = cloud->points[xx + yy * imgWidth].r;
              g = cloud->points[xx + yy * imgWidth].g;
              b = cloud->points[xx + yy * imgWidth].b;
              colorData[(xx + yy * imgWidth)*3] = b;
              colorData[(xx + yy * imgWidth)*3 + 1] = g;
              colorData[(xx + yy * imgWidth)*3 + 2] = r;
            }
          }
        }

        cv::resize(depthImage, depth, depth.size(), 0,0,cv::INTER_NEAREST);
        cv::resize(colorImage, color, color.size(), 0,0,cv::INTER_LINEAR);
      }

      //      //fill rgb frame and depth frame from point cloud
      //
      //      void pointCloudToDepthAndColor(IplImage* depth, IplImage* color, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
      //        //image size check
      //        if (depth->width != cloud->width || depth->height != cloud->height
      //                || color->width != cloud->width || color->height != cloud->height) {
      //          LOG4CXX_WARN(logger, "[pointCloudToDepthAndColor]: inconsistent sizes! Returning.");
      //          return;
      //        }
      //
      //        //using "default" kinect params
      //        const int imgWidth = cloud->width;
      //        const int imgHeight = cloud->height;
      //
      //        //TODO: use CameraParameters
      //        const float fx = 580.0 * (imgWidth / 640.0);
      //        const float fy = 580.0 * (imgHeight / 480.0);
      //        const float cx = imgWidth / 2.0;
      //        const float cy = imgHeight / 2.0;
      //
      //        float* depthData = (float*) depth->imageData;
      //        uint8_t* colorData = (uint8_t*) color->imageData;
      //        float x_ir, y_ir, z_ir;
      //        uint8_t b, g, r;
      //
      //        for (int xx = 0; xx < imgWidth; xx++) {
      //          for (int yy = 0; yy < imgHeight; yy++) {
      //
      //            x_ir = cloud->points[xx + yy * imgWidth].x;
      //            y_ir = cloud->points[xx + yy * imgWidth].y;
      //            z_ir = cloud->points[xx + yy * imgWidth].z;
      //
      //            if (std::isnan(x_ir) || std::isnan(y_ir) || std::isnan(z_ir)) {
      //              depthData[xx + imgWidth * yy] = std::numeric_limits<float>::quiet_NaN();
      //            } else {
      //              //assumes an organized point cloud
      //              depthData[xx + imgWidth * yy] = z_ir;
      //
      //              colorData[(xx + yy * imgWidth)*3] = 0;
      //              colorData[(xx + yy * imgWidth)*3 + 1] = 0;
      //              colorData[(xx + yy * imgWidth)*3 + 2] = 0;
      //            }
      //          }
      //        }
      //      }

      void calculateBoundingBox(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                pcl::PointXYZ& min_point,
                                pcl::PointXYZ& max_point,
                                Eigen::Vector3f& mass_center) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < cloud->size(); ++i) {
          pcl::PointXYZ point = cloud->at(i);
          if (point.x == point.x && point.y == point.y && point.z == point.z) {
            // not nan
            cloud_filtered->push_back(point);
          }
        }

        pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
        feature_extractor.setInputCloud(cloud_filtered);
        feature_extractor.compute();

        std::vector <float> moment_of_inertia;
        std::vector <float> eccentricity;
        pcl::PointXYZ min_point_AABB;
        pcl::PointXYZ max_point_AABB;
        pcl::PointXYZ min_point_OBB;
        pcl::PointXYZ max_point_OBB;
        pcl::PointXYZ position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;
        float major_value, middle_value, minor_value;
        Eigen::Vector3f major_vector, middle_vector, minor_vector;

        bool result;
        result = feature_extractor.getMomentOfInertia (moment_of_inertia);
        if (!result) LOG4CXX_WARN(logger, "feature extractor false 0");
        result = feature_extractor.getEccentricity (eccentricity);
        if (!result) LOG4CXX_WARN(logger, "feature extractor false 1");
        result = feature_extractor.getAABB (min_point_AABB, max_point_AABB);
        if (!result) LOG4CXX_WARN(logger, "feature extractor false 2");
        result = feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
        if (!result) LOG4CXX_WARN(logger, "feature extractor false 3");
        result = feature_extractor.getEigenValues (major_value, middle_value, minor_value);
        if (!result) LOG4CXX_WARN(logger, "feature extractor false 4");
        result = feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
        if (!result) LOG4CXX_WARN(logger, "feature extractor false 5");
        result = feature_extractor.getMassCenter (mass_center);
        if (!result) LOG4CXX_WARN(logger, "feature extractor false 6");

        min_point = min_point_AABB;
        max_point = max_point_AABB;

        LOG4CXX_DEBUG(logger, boost::format("mass center: %f %f %f.") % mass_center(0) % mass_center(1) % mass_center(2));
        LOG4CXX_DEBUG(logger, boost::format("min_point: %f %f %f.") % min_point.x % min_point.y % min_point.z);
        LOG4CXX_DEBUG(logger, boost::format("max_point: %f %f %f.") % max_point.x % max_point.y % max_point.z);
//        LOG4CXX_DEBUG(logger, boost::format("position: %f %f %f.") % position_OBB.x % position_OBB.y % position_OBB.z);

        ///////////////// TEMPORARY DEBUGGING VIZ //////////////////////////
//        pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D BB Viewer"));
//        viewer->setBackgroundColor (0, 0, 0);
//        viewer->addCoordinateSystem (1.0);
//        viewer->initCameraParameters ();
//        viewer->addPointCloud<pcl::PointXYZ> (cloud, "cluster cloud");
////        viewer->addPointCloud<pcl::PointXYZ> (scene_cloud, "scene cloud");
//        viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
//        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");
//
//        Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
//        Eigen::Quaternionf quat (rotational_matrix_OBB);
//        viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
//        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");
//
//        pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
//        pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
//        pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
//        pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
//        viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
//        viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
//        viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");
//
//        viewer->setPosition(0, 0);
//        viewer->setSize(640, 480);
//
//        while (!viewer->wasStopped()) {
//          viewer->spinOnce(100);
//          boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//        }
//
//        viewer->close();
      }

    } //namespace util
  } //namespace pc
} //namespace ade
