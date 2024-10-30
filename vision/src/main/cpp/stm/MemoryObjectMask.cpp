
/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "MemoryObjectMask.hpp"

#include "capture/util/CaptureUtilities.hpp"
#include <boost/format.hpp>
#include <boost/thread/lock_guard.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>

using namespace diarc::stm;

//static object(s)
log4cxx::LoggerPtr MemoryObjectMask::logger(log4cxx::Logger::getLogger("edu.tufts.hrilab.stm.MemoryObjectMask"));

MemoryObjectMask::MemoryObjectMask(const CaptureData::ConstPtr &capture, const cv::Mat &mask)
        : captureData_(capture),
          imageMask_(),
          indicesMask_(),
          boundingBox_(),
          dimensions_(),
          location_(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN()),
          direction_(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::quiet_NaN()),
          objectCloud_(),
          objectCloudRGB_(),
          objectImage_(),
          cloud_mutex_(),
          image_mutex_() {

  LOG4CXX_DEBUG(logger, "[MemoryObjectMask] cvMat Mask constructor.");

  if (mask.size != captureData_->frame.size) {
    LOG4CXX_ERROR(logger, boost::format("Image mask of wrong size. Expected: (%d,%d). Received: (%d,%d).")
                          % mask.rows % mask.cols % captureData_->frame.rows % captureData_->frame.cols);
  } else if (mask.type() == CV_32F) {
    mask.copyTo(imageMask_);
  } else if (mask.type() == CV_8U) {
    mask.convertTo(imageMask_, CV_32F, (1.0/255.0));
  } else {
    LOG4CXX_ERROR(logger, "Mask has invalid type. Must be CV_8U or CV_32F");
  }

  // get image size
  int imgWidth = captureData_->frame.cols;
  int imgHeight = captureData_->frame.rows;

  // for bounding box
  int xmin = imgWidth;
  int xmax = -1;
  int ymin = imgHeight;
  int ymax = -1;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  // for location
  cv::Point3d loc(0, 0, 0);
  int locIndices = 0;

  //
  for (int x = 0; x < mask.cols; ++x) {
    for (int y = 0; y < mask.rows; ++y) {
      if (imageMask_.at<float>(y, x) > 0) {

        int index = x + y * imgWidth;
        indicesMask_.push_back(index);

        // create bounding box
        if (x > xmax)
          xmax = x;
        if (x < xmin)
          xmin = x;
        if (y > ymax)
          ymax = y;
        if (y < ymin)
          ymin = y;

        // if we have depth info
        if (captureData_->hasCloudData()
            && !std::isnan(captureData_->cloud->points[index].x)
            && !std::isnan(captureData_->cloud->points[index].y)
            && !std::isnan(captureData_->cloud->points[index].z)) {

          // create filtered cloud to use for finding dimensions
          pcl::PointXYZ point = captureData_->cloud->at(index);
          // Note: Added conditional necessary due to detector randomly picking up (0,0,0) points
          // TODO: Fix clustering of object points to eliminate noise then get rid of conditional
          if (!(point.x == 0.0 && point.y == 0.0 && point.z == 0.0))
            cloud_filtered->push_back(point);

          //add to loc sum
          loc.x += point.x;
          loc.y += point.y;
          loc.z += point.z;
          ++locIndices;
        }

      }
    }
  }

  // set dimensions
  if (captureData_->hasCloudData()) {
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud_filtered);
    feature_extractor.compute();
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    feature_extractor.getAABB (min_point_AABB, max_point_AABB);
    LOG4CXX_DEBUG(logger, boost::format("max_point: %f %f %f.") % max_point_AABB.x % max_point_AABB.y % max_point_AABB.z);
    LOG4CXX_DEBUG(logger, boost::format("min_point: %f %f %f.") % min_point_AABB.x % min_point_AABB.y % min_point_AABB.z);

    dimensions_.x = max_point_AABB.x - min_point_AABB.x;
    dimensions_.y = max_point_AABB.y - min_point_AABB.y;
    dimensions_.z = max_point_AABB.z - min_point_AABB.z;
  }

  //set direction and location
  if (captureData_->hasCloudData() && locIndices > 0) {
    loc.x /= locIndices;
    loc.y /= locIndices;
    loc.z /= locIndices;
    setLocation(loc); //also sets direction
  } else {
    direction_ = diarc::capture::util::calcDirection(0, xmin + (xmax - xmin) / 2.0, ymin + (ymax - ymin) / 2.0);
  }

  //set bounding box
  boundingBox_.x = xmin;
  boundingBox_.y = ymin;
  boundingBox_.width = xmax - xmin;
  boundingBox_.height = ymax - ymin;
}

MemoryObjectMask::MemoryObjectMask(const CaptureData::ConstPtr &capture, const cv::Rect &boundingBoxMask)
        : captureData_(capture),
          imageMask_(),
          indicesMask_(),
          boundingBox_(boundingBoxMask),
          dimensions_(),
          location_(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN()),
          direction_(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::quiet_NaN()),
          objectCloud_(),
          objectCloudRGB_(),
          objectImage_(),
          cloud_mutex_(),
          image_mutex_() {

  if (logger->isDebugEnabled()) {
    LOG4CXX_DEBUG(logger, boost::format("[MemoryObjectMask] rect constructor. Rect: (%d,%d,%d,%d)")
                          % boundingBoxMask.x % boundingBoxMask.y % boundingBoxMask.width % boundingBoxMask.height);
  }

  // initialize image mask to all zeros
  imageMask_ = cv::Mat_<float>::zeros(captureData_->frame.size());

  // get image size
  int imgWidth = captureData_->frame.cols;
  int imgHeight = captureData_->frame.rows;

  // for location
  cv::Point3d loc(0, 0, 0);
  int locIndices = 0;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  // bounds check and adjustment
  if (boundingBox_.x < 0) {
    LOG4CXX_DEBUG(logger, boost::format("[MemoryObjectMask] bounding box out of bounds: (%d,%d,%d,%d).")
                          % boundingBox_.x % boundingBox_.y % boundingBox_.width % boundingBox_.height);
    boundingBox_.width = boundingBox_.width + boundingBox_.x;
    boundingBox_.x = 0;
  }
  if (boundingBox_.y < 0) {
    LOG4CXX_DEBUG(logger, boost::format("[MemoryObjectMask] bounding box out of bounds: (%d,%d,%d,%d).")
                          % boundingBox_.x % boundingBox_.y % boundingBox_.width % boundingBox_.height);
    boundingBox_.height = boundingBox_.height + boundingBox_.y;
    boundingBox_.y = 0;
  }
  if (boundingBox_.br().x >= imgWidth) {
    LOG4CXX_DEBUG(logger, boost::format("[MemoryObjectMask] bounding box out of bounds: (%d,%d,%d,%d).")
                          % boundingBox_.x % boundingBox_.y % boundingBox_.width % boundingBox_.height);
    boundingBox_.width = imgWidth - boundingBox_.x - 1;
  }
  if (boundingBox_.br().y >= imgHeight) {
    LOG4CXX_DEBUG(logger, boost::format("[MemoryObjectMask] bounding box out of bounds: (%d,%d,%d,%d).")
                          % boundingBox_.x % boundingBox_.y % boundingBox_.width % boundingBox_.height);
    boundingBox_.height = imgHeight - boundingBox_.y - 1;
  }


  // calculate additional mask info based on bounding box
  for (int x = boundingBox_.x; x < (boundingBox_.x + boundingBox_.width); ++x) {
    for (int y = boundingBox_.y; y < (boundingBox_.y + boundingBox_.height); ++y) {

      imageMask_(y, x) = 1.0;

      int index = x + y * imgWidth;
      indicesMask_.push_back(index);

      // if we have depth info
      if (captureData_->hasCloudData()
          && !std::isnan(captureData_->cloud->points[index].x)
          && !std::isnan(captureData_->cloud->points[index].y)
          && !std::isnan(captureData_->cloud->points[index].z)) {

        // create filtered cloud to use for finding dimensions
        pcl::PointXYZ point = captureData_->cloud->at(index);
        // Note: Added conditional necessary due to detector randomly picking up (0,0,0) points
        // TODO: Fix clustering of object points to eliminate noise then get rid of conditional
        if (!(point.x == 0.0 && point.y == 0.0 && point.z == 0.0))
          cloud_filtered->push_back(point);

        //add to loc sum
        loc.x += point.x;
        loc.y += point.y;
        loc.z += point.z;
        ++locIndices;
      }
    }
  }

  // set dimensions
  if (captureData_->hasCloudData()) {
    // pcl::io::savePCDFileASCII ("/home/dev/Downloads/empty_bowl_test.pcd", *cloud_filtered);
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud_filtered);
    feature_extractor.compute();
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    feature_extractor.getAABB (min_point_AABB, max_point_AABB);
    LOG4CXX_DEBUG(logger, boost::format("max_point: %f %f %f.") % max_point_AABB.x % max_point_AABB.y % max_point_AABB.z);
    LOG4CXX_DEBUG(logger, boost::format("min_point: %f %f %f.") % min_point_AABB.x % min_point_AABB.y % min_point_AABB.z);

    dimensions_.x = max_point_AABB.x - min_point_AABB.x;
    dimensions_.y = max_point_AABB.y - min_point_AABB.y;
    dimensions_.z = max_point_AABB.z - min_point_AABB.z;
  }

  LOG4CXX_DEBUG(logger, boost::format("[MemoryObjectMask] num depth points: %d.") % locIndices);

  //set direction and location
  if (captureData_->hasCloudData() && locIndices > 0) {
    loc.x /= locIndices;
    loc.y /= locIndices;
    loc.z /= locIndices;
    setLocation(loc); //also sets direction
  } else {
    direction_ = diarc::capture::util::calcDirection(0, boundingBox_.x + (boundingBox_.width) / 2.0,
                                                   boundingBox_.y + (boundingBox_.height) / 2.0);
  }

}

MemoryObjectMask::MemoryObjectMask(const CaptureData::ConstPtr &capture, const std::vector<int> &imageIndicesMask)
        : captureData_(capture),
          imageMask_(),
          indicesMask_(imageIndicesMask),
          boundingBox_(),
          dimensions_(),
          location_(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN()),
          direction_(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::quiet_NaN()),
          objectCloud_(),
          objectCloudRGB_(),
          objectImage_(),
          cloud_mutex_(),
          image_mutex_() {

  LOG4CXX_DEBUG(logger, "[MemoryObjectMask] imageIndicesMask constructor.");
  // initialize image mask to all zeros
  imageMask_ = cv::Mat_<float>::zeros(captureData_->frame.size());

  // get image size
  int imgWidth = captureData_->frame.cols;
  int imgHeight = captureData_->frame.rows;

  // for bounding box
  int xmin = imgWidth;
  int xmax = -1;
  int ymin = imgHeight;
  int ymax = -1;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filteredRGB(new pcl::PointCloud<pcl::PointXYZRGB>);

  // for location
  cv::Point3d loc(0, 0, 0);
  int locIndices = 0;

  for (int i = 0; i < indicesMask_.size(); ++i) {
    // indices numbers in the cluster are numbers in the original point cloud
    int index = indicesMask_[i];
    int xx = index % imgWidth;
    int yy = index / imgWidth;

    // bounds check
    if (xx < 0 || xx >= imgWidth || yy < 0 || yy >= imgHeight) continue;

    imageMask_(yy, xx) = 1.0;

    // create bounding box
    if (xx > xmax)
      xmax = xx;
    if (xx < xmin)
      xmin = xx;
    if (yy > ymax)
      ymax = yy;
    if (yy < ymin)
      ymin = yy;

    // if we have depth info
    if (captureData_->hasCloudData()
        && !std::isnan(captureData_->cloud->points[index].x)
        && !std::isnan(captureData_->cloud->points[index].y)
        && !std::isnan(captureData_->cloud->points[index].z)) {

      // create filtered cloud to use for finding dimensions
      pcl::PointXYZ point = captureData_->cloud->at(index);
      // LOG4CXX_DEBUG(logger, boost::format("MEMOBJMASK point: %f %f %f.") % point.x % point.y % point.z);
      // pcl::PointXYZRGB pointRGB = captureData_->cloudRGB->at(index);
      // Note: Added conditional necessary due to detector randomly picking up (0,0,0) points
      // TODO: Fix clustering of object points to eliminate noise then get rid of conditional
      if (!(point.x == 0.0 && point.y == 0.0 && point.z == 0.0)) {
        cloud_filtered->push_back(point);
        // cloud_filteredRGB->push_back(pointRGB);
      }

      //add to loc sum
      // Experimental change - maybe since we know object clouds will not be full,
      //   i.e. from one perspective, we should not use avg of points
      // For sim case, as well as possibly more generally now with new tightened clustering 
      //   params + table mask, could be better to use middle of xyz min/max
      loc.x += point.x;
      loc.y += point.y;
      loc.z += point.z;
      ++locIndices;
    }
  }
  // pcl::io::savePCDFile ("/home/dev/test.pcd", *(cloud_filteredRGB));
  // pcl::io::savePCDFile ("/home/dev/test.pcd", *(cloud_filtered));

  // set dimensions
  if (captureData_->hasCloudData()) {
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud_filtered);
    feature_extractor.compute();
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    feature_extractor.getAABB (min_point_AABB, max_point_AABB);
    LOG4CXX_DEBUG(logger, boost::format("max_point: %f %f %f.") % max_point_AABB.x % max_point_AABB.y % max_point_AABB.z);
    LOG4CXX_DEBUG(logger, boost::format("min_point: %f %f %f.") % min_point_AABB.x % min_point_AABB.y % min_point_AABB.z);

    dimensions_.x = max_point_AABB.x - min_point_AABB.x;
    dimensions_.y = max_point_AABB.y - min_point_AABB.y;
    dimensions_.z = max_point_AABB.z - min_point_AABB.z;
    // loc.x = (max_point_AABB.x + min_point_AABB.x)/2.0;
    // loc.y = (max_point_AABB.y + min_point_AABB.y)/2.0;
    // loc.z = (max_point_AABB.z + min_point_AABB.z)/2.0;
    setLocation(loc); //also sets direction
  }

  //set direction and location
  if (locIndices > 0) {
    loc.x /= locIndices;
    loc.y /= locIndices;
    loc.z /= locIndices;
    setLocation(loc); //also sets direction
  } else {
    direction_ = diarc::capture::util::calcDirection(0, xmin + (xmax - xmin) / 2.0, ymin + (ymax - ymin) / 2.0);
  }

  //set bounding box
  boundingBox_.x = xmin;
  boundingBox_.y = ymin;
  boundingBox_.width = xmax - xmin;
  boundingBox_.height = ymax - ymin;

  LOG4CXX_DEBUG(logger, boost::format("[MemoryObjectMask]  2D bb. (x,width,y,height) = (%d,%d,%d,%d).")
                    % boundingBox_.x % boundingBox_.width % boundingBox_.y % boundingBox_.height);
  LOG4CXX_DEBUG(logger, boost::format("[MemoryObjectMask]  dims (x,y,z)) = (%d,%d,%d).")
                  % dimensions_.x % dimensions_.y % dimensions_.z);
}

MemoryObjectMask::MemoryObjectMask(const CaptureData::ConstPtr &capture,
                                   const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloudMask)
        : captureData_(capture),
          imageMask_(),
          indicesMask_(),
          boundingBox_(),
          dimensions_(),
          location_(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN()),
          direction_(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::quiet_NaN()),
          objectCloud_(),
          objectCloudRGB_(),
          objectImage_(),
          cloud_mutex_(),
          image_mutex_() {

  LOG4CXX_DEBUG(logger, "[MemoryObjectMask] point cloud constructor.");

  // get image size
  int imgWidth = captureData_->frame.cols;
  int imgHeight = captureData_->frame.rows;

  // make copy of cloud if organized - else create a new organized cloud to be
  // populated in loop below
  bool isOrganized = cloudMask->isOrganized();
  if (isOrganized) {
    objectCloud_ = cloudMask->makeShared();
  } else {
    pcl::PointXYZ pointNaN;
    pointNaN.x = pointNaN.y = pointNaN.z = std::numeric_limits<float>::quiet_NaN();
    objectCloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(
            new pcl::PointCloud<pcl::PointXYZ>(imgWidth, imgHeight, pointNaN));
    objectCloudUnorganized_ = cloudMask->makeShared();
  }

  // initialize image mask to all zeros
  imageMask_ = cv::Mat_<float>::zeros(captureData_->frame.size());

  // for location
  cv::Point3d loc(0, 0, 0);

  // for bounding box
  int xmin = imgWidth;
  int xmax = -1;
  int ymin = imgHeight;
  int ymax = -1;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  // iterate through point cloud
  pcl::PointXYZ point3D;
  int xx, yy, index;
  for (int i = 0; i < cloudMask->size(); ++i) {
    point3D = cloudMask->at(i);

    capture::util::projectPoint(point3D.x, point3D.y, point3D.z, xx, yy, 0);
    LOG4CXX_DEBUG(logger, boost::format("[MemoryObjectMask]  3D to 2D. (x,y,z) = (%d,%d,%d) to (x,y) = (%d,%d).")
                          % point3D.x % point3D.y % point3D.z % xx % yy);

    // bounds check
    if (xx < 0 || xx >= imgWidth || yy < 0 || yy >= imgHeight) continue;

    // fill indices
    index = xx + yy * imgWidth;
    indicesMask_.push_back(index);

    // fill cloud (if not already filled)
    if (!isOrganized) {
      objectCloud_->at(index).x = point3D.x;
      objectCloud_->at(index).y = point3D.y;
      objectCloud_->at(index).z = point3D.z;
    }

    // fill binary image mask
    imageMask_(yy, xx) = 1.0;

    // create bounding box
    if (xx > xmax)
      xmax = xx;
    if (xx < xmin)
      xmin = xx;
    if (yy > ymax)
      ymax = yy;
    if (yy < ymin)
      ymin = yy;


    // Note: Added conditional necessary due to detector randomly picking up (0,0,0) points
    // TODO: Fix clustering of object points to eliminate noise then get rid of conditional
    if (!(point3D.x == 0.0 && point3D.y == 0.0 && point3D.z == 0.0))
      cloud_filtered->push_back(point3D);

    loc.x += point3D.x;
    loc.y += point3D.y;
    loc.z += point3D.z;
  }

  // TODO: handle better
  if (cloudMask->size() > 1) {
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud_filtered);
    feature_extractor.compute();
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    feature_extractor.getAABB (min_point_AABB, max_point_AABB);
    LOG4CXX_DEBUG(logger, boost::format("max_point: %f %f %f.") % max_point_AABB.x % max_point_AABB.y % max_point_AABB.z);
    LOG4CXX_DEBUG(logger, boost::format("min_point: %f %f %f.") % min_point_AABB.x % min_point_AABB.y % min_point_AABB.z);

    dimensions_.x = max_point_AABB.x - min_point_AABB.x;
    dimensions_.y = max_point_AABB.y - min_point_AABB.y;
    dimensions_.z = max_point_AABB.z - min_point_AABB.z;
  }
  else {
    dimensions_.x = 0;
    dimensions_.y = 0;
    dimensions_.z = 0;
  }

  //set direction and location
  if (indicesMask_.size() > 0) {
    loc.x /= indicesMask_.size();
    loc.y /= indicesMask_.size();
    loc.z /= indicesMask_.size();
    setLocation(loc); //also sets direction
  } else {
    direction_ = diarc::capture::util::calcDirection(0, xmin + (xmax - xmin) / 2.0, ymin + (ymax - ymin) / 2.0);
  }

  //set bounding box
  boundingBox_.x = xmin;
  boundingBox_.y = ymin;
  boundingBox_.width = xmax - xmin;
  boundingBox_.height = ymax - ymin;

  LOG4CXX_DEBUG(logger, boost::format("[MemoryObjectMask]  2D bb. (x,width,y,height) = (%d,%d,%d,%d).")
                      % boundingBox_.x % boundingBox_.width % boundingBox_.y % boundingBox_.height);
  LOG4CXX_DEBUG(logger, boost::format("[MemoryObjectMask]  dims (x,y,z)) = (%d,%d,%d).")
                  % dimensions_.x % dimensions_.y % dimensions_.z);
}

MemoryObjectMask::MemoryObjectMask(const CaptureData::ConstPtr &capture,
                                   const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloudMask)
        : captureData_(capture),
          imageMask_(),
          indicesMask_(),
          boundingBox_(),
          dimensions_(),
          location_(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN()),
          direction_(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::quiet_NaN()),
          objectCloud_(),
          objectCloudRGB_(),
          objectImage_(),
          cloud_mutex_(),
          image_mutex_() {

  LOG4CXX_DEBUG(logger, "[MemoryObjectMask] RGB point cloud constructor.");

  // get image size
  int imgWidth = captureData_->frame.cols;
  int imgHeight = captureData_->frame.rows;

  // make copy of cloud if organized - else create a new organized cloud to be
  // populated in loop below
  bool isOrganized = cloudMask->isOrganized();
  if (isOrganized) {
    objectCloudRGB_ = cloudMask->makeShared();
  } else {
    pcl::PointXYZRGB pointNaN;
    pointNaN.x = pointNaN.y = pointNaN.z = std::numeric_limits<float>::quiet_NaN();
    objectCloudRGB_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
            new pcl::PointCloud<pcl::PointXYZRGB>(imgWidth, imgHeight, pointNaN));
  }

  // initialize image mask to all zeros
  imageMask_ = cv::Mat_<float>::zeros(captureData_->frame.size());

  // for location
  cv::Point3d loc(0, 0, 0);

  // for bounding box
  int xmin = imgWidth;
  int xmax = -1;
  int ymin = imgHeight;
  int ymax = -1;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  // iterate through point cloud
  pcl::PointXYZ point3D;
  pcl::PointXYZRGB point3DRGB;
  int xx, yy, index;
  for (int i = 0; i < cloudMask->size(); ++i) {
    point3DRGB = cloudMask->at(i);

    capture::util::projectPoint(point3DRGB.x, point3DRGB.y, point3DRGB.z, xx, yy, 0);

    // bounds check
    if (xx < 0 || xx >= imgWidth || yy < 0 || yy >= imgHeight) continue;

    // fill indices
    index = xx + yy * imgWidth;
    indicesMask_.push_back(index);

    // fill cloud (if not already filled)
    if (!isOrganized) {
      objectCloudRGB_->at(index).x = point3DRGB.x;
      objectCloudRGB_->at(index).y = point3DRGB.y;
      objectCloudRGB_->at(index).z = point3DRGB.z;
      objectCloudRGB_->at(index).rgba = point3DRGB.rgba;
    }

    // fill binary image mask
    imageMask_(yy, xx) = 1.0;

    // create bounding box
    if (xx > xmax)
      xmax = xx;
    if (xx < xmin)
      xmin = xx;
    if (yy > ymax)
      ymax = yy;
    if (yy < ymin)
      ymin = yy;

    // Note: Added conditional necessary due to detector randomly picking up (0,0,0) points
    // TODO: Fix clustering of object points to eliminate noise then get rid of conditional
    if (!(point3D.x == 0.0 && point3D.y == 0.0 && point3D.z == 0.0))
      cloud_filtered->push_back(point3D);

    loc.x += point3D.x;
    loc.y += point3D.y;
    loc.z += point3D.z;
  }

  if (cloudMask->size() > 1) {
    // set dimensions
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud_filtered);
    feature_extractor.compute();
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    feature_extractor.getAABB (min_point_AABB, max_point_AABB);
    LOG4CXX_DEBUG(logger, boost::format("max_point: %f %f %f.") % max_point_AABB.x % max_point_AABB.y % max_point_AABB.z);
    LOG4CXX_DEBUG(logger, boost::format("min_point: %f %f %f.") % min_point_AABB.x % min_point_AABB.y % min_point_AABB.z);

    dimensions_.x = max_point_AABB.x - min_point_AABB.x;
    dimensions_.y = max_point_AABB.y - min_point_AABB.y;
    dimensions_.z = max_point_AABB.z - min_point_AABB.z;
  }
  else {
    dimensions_.x = 0;
    dimensions_.y = 0;
    dimensions_.z = 0;
  }
  
  //set direction and location
  if (indicesMask_.size() > 0) {
    loc.x /= indicesMask_.size();
    loc.y /= indicesMask_.size();
    loc.z /= indicesMask_.size();
    setLocation(loc); //also sets direction
  } else {
    direction_ = diarc::capture::util::calcDirection(0, xmin + (xmax - xmin) / 2.0, ymin + (ymax - ymin) / 2.0);
  }

  //set bounding box
  boundingBox_.x = xmin;
  boundingBox_.y = ymin;
  boundingBox_.width = xmax - xmin;
  boundingBox_.height = ymax - ymin;
}


MemoryObjectMask::MemoryObjectMask(const MemoryObjectMask &other) {
  // copy everything except for mutexes
  this->captureData_ = other.captureData_;
  this->imageMask_ = other.imageMask_;
  this->indicesMask_ = other.indicesMask_;
  this->boundingBox_ = other.boundingBox_;
  this->dimensions_ = other.dimensions_;
  this->location_ = other.location_;
  this->direction_ = other.direction_;
  this->objectCloudUnorganized_ = other.objectCloudUnorganized_;
  this->objectCloud_ = other.objectCloud_;
  this->objectCloudRGB_ = other.objectCloudRGB_;
  this->objectImage_ = other.objectImage_;
}

MemoryObjectMask::~MemoryObjectMask() {
}

unsigned long long MemoryObjectMask::getFrameNumber() const {
  return captureData_->frameNumber;
}

const CaptureData::ConstPtr &MemoryObjectMask::getCaptureData() const {
  return captureData_;
}

const cv::Mat &MemoryObjectMask::getTransform() const {
  return captureData_->transform;
}

const cv::Mat_<float> &MemoryObjectMask::getImageMask() const {
  return imageMask_;
}

const std::vector<int> &MemoryObjectMask::getIndicesMask() const {
  return indicesMask_;
}

const cv::Mat &MemoryObjectMask::getObjectImage() const {
  boost::lock_guard<boost::mutex> lock(image_mutex_);

  if (objectImage_.size != captureData_->frame.size) {
    objectImage_ = cv::Mat::zeros(captureData_->frame.size(), CV_8UC3);
    for (int r = 0; r < captureData_->frame.rows; ++r) {
      for (int c = 0; c < captureData_->frame.cols; ++c) {
        if (imageMask_(r, c) > 0) {
          objectImage_.at<cv::Vec3b>(r, c, 0) = captureData_->frame.at<cv::Vec3b>(r, c);
        }
      }
    }
  }

  return objectImage_;
}

pcl::PointCloud<pcl::PointXYZ>::ConstPtr MemoryObjectMask::getObjectPointCloud() const {
  boost::lock_guard<boost::mutex> lock(cloud_mutex_);

  if (!captureData_->hasCloudData()) {
    return pcl::PointCloud<pcl::PointXYZ>::Ptr();
  }

  if (!objectCloud_) {
    if (!objectCloudRGB_) {
      // if no colored object cloud, use indices and capture data 
      objectCloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(captureData_->cloud);
      pcl::PointIndices::Ptr cloudIndices(new pcl::PointIndices());
      cloudIndices->indices = indicesMask_;
      extract.setIndices(cloudIndices);
      extract.setNegative(false);
      extract.setKeepOrganized(true);
      extract.filter(*objectCloud_);
    } else {
      // use colored object cloud (which is guaranteed to be organized if it exists)
      pcl::PointXYZ pointNaN;
      pointNaN.x = pointNaN.y = pointNaN.z = std::numeric_limits<float>::quiet_NaN();
      objectCloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(
              new pcl::PointCloud<pcl::PointXYZ>(objectCloudRGB_->width, objectCloudRGB_->height, pointNaN));

      for (int i = 0; i < objectCloudRGB_->size(); ++i) {
        // populate 3D point
        pcl::PointXYZ &point3D = objectCloud_->at(i);
        const pcl::PointXYZRGB &point3DRGB = objectCloudRGB_->at(i);
        point3D.x = point3DRGB.x;
        point3D.y = point3DRGB.y;
        point3D.z = point3DRGB.z;
      }
    }
  }

  return objectCloud_;
}

pcl::PointCloud<pcl::PointXYZ>::ConstPtr MemoryObjectMask::getUnorganizedObjectPointCloud() const {
  boost::lock_guard<boost::mutex> lock(cloud_mutex_);

  if (!captureData_->hasCloudData()) {
    return pcl::PointCloud<pcl::PointXYZ>::Ptr();
  }

  if (!objectCloudUnorganized_) {
    if (objectCloud_) {
      // use organized point cloud to populate unorganized cloud
      std::vector<int> indices;
      objectCloudUnorganized_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::removeNaNFromPointCloud(*objectCloud_, *objectCloudUnorganized_, indices);
    } else if (objectCloudRGB_) {
      // use colored object cloud (which is guaranteed to be organized if it exists)
      objectCloudUnorganized_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

      pcl::PointXYZ point3D;
      pcl::PointXYZRGB point3DRGB;
      for (int i = 0; i < objectCloudRGB_->size(); ++i) {
        // populate 3D point
        point3DRGB = objectCloudRGB_->at(i);
        point3D.x = point3DRGB.x;
        point3D.y = point3DRGB.y;
        point3D.z = point3DRGB.z;
        objectCloudUnorganized_->push_back(point3D);
      }
    } else {
      // if no populated organized clouds, use indices and capture data 
      objectCloudUnorganized_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(captureData_->cloud);
      pcl::PointIndices::Ptr cloudIndices(new pcl::PointIndices());
      cloudIndices->indices = indicesMask_;
      extract.setIndices(cloudIndices);
      extract.setNegative(false);
      extract.setKeepOrganized(false);
      extract.filter(*objectCloudUnorganized_);
    }
  }

  return objectCloudUnorganized_;
}

pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr MemoryObjectMask::getObjectPointCloudRGB() const {
  boost::lock_guard<boost::mutex> lock(cloud_mutex_);

  if (!captureData_->hasCloudRGBData()) {
    return pcl::PointCloud<pcl::PointXYZRGB>::Ptr();
  }

  if (!objectCloudRGB_) {
    if (true) {
      //LOG4CXX_WARN(logger, "[getObjectPointCloudRGB] RGB pointcloud constructor given existing object cloud is broken - creating from captureData");
      // if no non-colored object cloud, use indices and capture data 
      objectCloudRGB_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud(captureData_->cloudRGB);
      pcl::PointIndices::Ptr cloudIndices(new pcl::PointIndices());
      cloudIndices->indices = indicesMask_;
      extract.setIndices(cloudIndices);
      extract.setNegative(false);
      extract.setKeepOrganized(true);
      extract.filter(*objectCloudRGB_);
    } else {
      // use non-colored object cloud (which is guaranteed to be organized if it exists)
      pcl::PointXYZRGB pointNaN;
      pointNaN.x = pointNaN.y = pointNaN.z = std::numeric_limits<float>::quiet_NaN();
      objectCloudRGB_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
              new pcl::PointCloud<pcl::PointXYZRGB>(objectCloud_->width, objectCloud_->height, pointNaN));

      pcl::PointXYZ point3D;
      pcl::PointXYZRGB point3DRGB;
      uint8_t r, g, b;

      for (int i = 0; i < objectCloud_->size(); ++i) {
        // populate color 3D point
        point3DRGB = objectCloudRGB_->at(i);
        point3D = objectCloud_->at(i);
        point3DRGB.x = point3D.x;
        point3DRGB.y = point3D.y;
        point3DRGB.z = point3D.z;

        // get color of point and pack r/g/b into rgb
        r = captureData_->frame.at<uint8_t>(i * 3);
        g = captureData_->frame.at<uint8_t>(i * 3 + 1);
        b = captureData_->frame.at<uint8_t>(i * 3 + 2);
        point3DRGB.rgba = ((uint32_t) r << 16 | (uint32_t) g << 8 | (uint32_t) b);
      }
    }
  }

  return objectCloudRGB_;
}

pcl::PointXYZ MemoryObjectMask::getObjectPoint(int index) const {
  boost::lock_guard<boost::mutex> lock(cloud_mutex_);

  if (!objectCloud_ && !objectCloudRGB_) {
    // neither internal object cloud has been set -- use capture data
    return captureData_->cloud->at(index);

  } else if (!objectCloud_) {
    // don't have cloud, but must have colored cloud -- use that
    pcl::PointXYZRGB point3DRGB = objectCloudRGB_->at(index);
    pcl::PointXYZ point3D(point3DRGB.x, point3DRGB.y, point3DRGB.z);
    return point3D;
  } else {
    // have cloud
    return objectCloud_->at(index);
  }
}

pcl::PointXYZRGB MemoryObjectMask::getObjectPointRGB(int index) const {
  boost::lock_guard<boost::mutex> lock(cloud_mutex_);

  if (!captureData_->hasCloudRGBData()) {
    return pcl::PointXYZRGB();
  }

  if (!objectCloud_ && !objectCloudRGB_) {
    // neither internal object cloud has been set -- use capture data
    return captureData_->cloudRGB->at(index);
  } else if (!objectCloudRGB_) {
    // don't have cloud, but must have colored cloud -- use that
    pcl::PointXYZ point3D = objectCloud_->at(index);
    pcl::PointXYZRGB point3DRGB(point3D.x, point3D.y, point3D.z);

    // get color of point and pack r/g/b into rgb
    uint8_t r, g, b;
    r = captureData_->frame.at<uint8_t>(index * 3);
    g = captureData_->frame.at<uint8_t>(index * 3 + 1);
    b = captureData_->frame.at<uint8_t>(index * 3 + 2);
    point3DRGB.rgba = ((uint32_t) r << 16 | (uint32_t) g << 8 | (uint32_t) b);
    return point3DRGB;
  } else {
    // have cloud
    return objectCloudRGB_->at(index);
  }
}

const cv::Point3d &MemoryObjectMask::getLocation() const {
  return location_;
}

const cv::Vec3d &MemoryObjectMask::getDirection() const {
  return direction_;
}

const cv::Rect &MemoryObjectMask::getBoundingBox() const {
  return boundingBox_;
}

const cv::Point3d &MemoryObjectMask::getDimensions() const {
  return dimensions_;
}

void MemoryObjectMask::setLocation(const double &x, const double &y, const double &z) {
  LOG4CXX_DEBUG(logger, boost::format("[setLocation] (x,y,z) = (%d,%d,%d).") % x % y % z);

  location_.x = x;
  location_.y = y;
  location_.z = z;

  // also set corresponding direction vector
  setDirection(x, y, z);
}

void MemoryObjectMask::setLocation(const cv::Point3d &loc) {
  setLocation(loc.x, loc.y, loc.z);
}

void MemoryObjectMask::setDirection(const double &x, const double &y, const double &z) {

  LOG4CXX_DEBUG(logger, boost::format("[setDirection] (x,y,z) = (%d,%d,%d).") % x % y % z);

  direction_[0] = x;
  direction_[1] = y;
  direction_[2] = z;

  // make unit vector
  double normValue = 1.0 / cv::norm(direction_);
  direction_[0] *= normValue;
  direction_[1] *= normValue;
  direction_[2] *= normValue;
}

void MemoryObjectMask::setDirection(const cv::Vec3d &dir) {
  setDirection(dir[0], dir[1], dir[2]);
}

void MemoryObjectMask::setDimensions(const cv::Point3d &dims) {
  dimensions_.x = dims.x;
  dimensions_.y = dims.y;
  dimensions_.z = dims.z;
}

void MemoryObjectMask::setDimensions(const double& x, const double& y, const double& z) {
  dimensions_.x = x;
  dimensions_.y = y;
  dimensions_.z = z;
}
