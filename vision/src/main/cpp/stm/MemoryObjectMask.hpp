/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   MemoryObjectMask.hpp
 * Author: Evan Krause <evan.krause@tufts.edu>
 *
 * Created on March 7, 2016, 2:59 PM
 */

#ifndef MEMORYOBJECTMASK_HPP
#define	MEMORYOBJECTMASK_HPP

#include "common/CaptureData.hpp"

#include <boost/thread/mutex.hpp>
#include <log4cxx/logger.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <utility>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace ade {
  namespace stm {

    class MemoryObjectMask {
    public:
      typedef boost::shared_ptr<MemoryObjectMask> Ptr;
      typedef boost::shared_ptr<const MemoryObjectMask> ConstPtr;

      MemoryObjectMask(const CaptureData::ConstPtr& capture, const cv::Mat& imageMask);
      MemoryObjectMask(const CaptureData::ConstPtr& capture, const cv::Rect& boundingBoxMask);
      MemoryObjectMask(const CaptureData::ConstPtr& capture, const std::vector<int>& imageIndicesMask);
      MemoryObjectMask(const CaptureData::ConstPtr& capture, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloudMask);
      MemoryObjectMask(const CaptureData::ConstPtr& capture, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloudMask);

      MemoryObjectMask(const MemoryObjectMask&);

      virtual ~MemoryObjectMask();

      //! get frame number
      unsigned long long getFrameNumber() const;

      //! get capture data
      const CaptureData::ConstPtr& getCaptureData() const;

      //! get transform from capture data
      const cv::Mat& getTransform() const;

      //! get image probability mask (values between [0 1]) defining segmented object
      const cv::Mat_<float>& getImageMask() const;

      //! get indices (into image and cloud) mask defining segmented object
      const std::vector<int>& getIndicesMask() const;

      //! get segmented object image (all non-object pixels are black (0,0,0))
      const cv::Mat& getObjectImage() const;

      //! get segmented unorganized object point cloud. built either from (1) passed in cloud during
      //! construction or (2) from captureData and indices. NOTE: it's possible that
      //! in case (1) the returned cloud does not exactly match captureData cloud
      //! plus indices
      pcl::PointCloud<pcl::PointXYZ>::ConstPtr getUnorganizedObjectPointCloud() const;

      //! get segmented object point cloud. built either from (1) passed in cloud during
      //! construction or (2) from captureData and indices. NOTE: it's possible that
      //! in case (1) the returned cloud does not exactly match captureData cloud
      //! plus indices
      pcl::PointCloud<pcl::PointXYZ>::ConstPtr getObjectPointCloud() const;

      //! get segmented object color point cloud. built either from (1) passed in cloud during
      //! construction or (2) from captureData and indices. NOTE: it's possible that
      //! in case (1) the returned cloud does not exactly match captureData cloud
      //! plus indices 
      pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr getObjectPointCloudRGB() const;

      //! get single point of point cloud
      pcl::PointXYZ getObjectPoint(int index) const;

      //! get single colored point of point cloud
      pcl::PointXYZRGB getObjectPointRGB(int index) const;

      //! get (x,y,z) location in camera coordinate frame (meters)
      const cv::Point3d& getLocation() const;

      //! get direction in camera coordinate frame (unit vector)
      const cv::Vec3d& getDirection() const;

      //! get bounding box in image coordinate frame (in pixels)
      const cv::Rect& getBoundingBox() const;

      //! get dimensions of object based on initial detection (meters)
      const cv::Point3d& getDimensions() const;

    protected:

      //! set (x,y,z) location in camera coordinate frame (meters). also sets direction.
      void setLocation(const double& x, const double& y, const double& z);

      //! set (x,y,z) location in camera coordinate frame (meters). also sets direction.
      void setLocation(const cv::Point3d& loc);

      //! set (x,y,z) dimensions of object
      void setDimensions(const cv::Point3d& dims);

      //! set (x,y,z) dimensions of object
      void setDimensions(const double& x, const double& y, const double& z);

      //! set direction in camera coordinate frame (unit vector)
      void setDirection(const double& x, const double& y, const double& z);

      //! set direction in camera coordinate frame (unit vector)
      void setDirection(const cv::Vec3d& dir);

      //! capture data that was used to detect the object
      CaptureData::ConstPtr captureData_;

      /**
       * Probability image mask of captureData indicating segmented part of 
       * image representing this object. Has type CV_32F (single channel floats)
       * where each pixel is in [0,1].
       * TODO: change this to a cv::SparseMat_<float> ??
       */
      cv::Mat_<float> imageMask_;

      /**
       * Indices mask of captureData indicating segmented part of image
       * representing this object. Indices into captured frame and 
       * point cloud (if available). Stores the same info as imageMask but
       * in a different format.
       */
      std::vector<int> indicesMask_;

      //! bounding box in the image frame completely enclosing the imageMask
      cv::Rect boundingBox_;

      //! object dimensions based on point cloud from initial detection
      cv::Point3d dimensions_;

      //! object location (m) only available if depth info is available
      cv::Point3d location_;

      //! object direction from origin (unit vector)
      cv::Vec3d direction_;

      //! cached segmented object point cloud (only set during getObjectCloud call)
      mutable boost::mutex cloud_mutex_;
      mutable pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloudUnorganized_;
      mutable pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloud_;
      mutable pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectCloudRGB_;

      //! cached segmented object image (all non-object pixels are black (0,0,0))
      //! (only set during getObjectImage call)
      mutable boost::mutex image_mutex_;
      mutable cv::Mat objectImage_;

      static log4cxx::LoggerPtr logger;
    };

  } //namespace stm
} //namespace ade

#endif	/* MEMORYOBJECTMASK_HPP */

