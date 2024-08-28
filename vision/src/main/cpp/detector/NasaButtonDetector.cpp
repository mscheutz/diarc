/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "NasaButtonDetector.hpp"
#include "display/Display.hpp"
#include <opencv2/opencv.hpp>
#ifdef USE_ROS_VISION
#include <srcsim/Console.h>
#include <vector>
#endif //USE_ROS_VISION

NasaButtonDetector::NasaButtonDetector(const long long& processorId, const int imgWidth, const int imgHeight)
: ObjectDetector(processorId, imgWidth, imgHeight),
buttonDetectionReported(false) {
  visionProcessName = "NasaButtonDetector";
  logger = log4cxx::Logger::getLogger("diarc.detector.NasaButtonDetector");

  displayFrame = cv::Mat(imgHeight, imgWidth, CV_8UC3);

#ifdef USE_ROS_VISION
  //initialize ROS
  LOG4CXX_INFO(logger, "Initializing ROS.");
  std::vector<std::pair<std::string, std::string> > remapping;
  ros::init(remapping, "diarc_nasa_button_results");

  //create node handle
  LOG4CXX_INFO(logger, "Initializing ROS node.");
  n = new ros::NodeHandle();

  // results publisher
  pub = n->advertise<srcsim::Console>("/srcsim/qual1/light", 1);
#endif //USE_ROS_VISION

}

NasaButtonDetector::~NasaButtonDetector() {
}

void NasaButtonDetector::handleCaptureNotification(CaptureNotification::ConstPtr notification) {
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudRGB = notification->captureData->cloudRGB;
  const cv::Mat frame = notification->captureData->frame;

  // check for red button
  int pixelCount = 0;
  cv::inRange(frame, cv::Scalar(0, 0, 245), cv::Scalar(10, 10, 255), fgMask); // BGR color space
  pixelCount = cv::countNonZero(fgMask);
  bool redButtonDetected = (pixelCount > 0) ? true : false;

  // check for green button
  bool greenButtonDetected = false;
  if (!redButtonDetected) {
    cv::inRange(frame, cv::Scalar(0, 245, 0), cv::Scalar(10, 255, 10), fgMask); // BGR color space
    pixelCount = cv::countNonZero(fgMask);
    greenButtonDetected = (pixelCount > 0) ? true : false;
  } else {
    LOG4CXX_INFO(logger, boost::format("RED button detected with pixelCount: %d.") % pixelCount);
  }

  // check for blue button
  bool blueButtonDetected = false;
  if (!redButtonDetected && !greenButtonDetected) {
    cv::inRange(frame, cv::Scalar(245, 0, 0), cv::Scalar(255, 10, 10), fgMask); // BGR color space
    pixelCount = cv::countNonZero(fgMask);
    blueButtonDetected = (pixelCount > 0) ? true : false;
  } else {
    LOG4CXX_INFO(logger, boost::format("GREEN button detected with pixelCount: %d.") % pixelCount);
  }

  if (blueButtonDetected) {
    LOG4CXX_INFO(logger, boost::format("BLUE button detected with pixelCount: %d.") % pixelCount);
  }

  bool buttonDetected = (redButtonDetected || greenButtonDetected || blueButtonDetected) ? true : false;

  if (buttonDetected && !buttonDetectionReported) {

    // calculate location of button
    int pixelCount = 0;
    int minX = fgMask.cols, maxX = 0, minY = fgMask.rows, maxY = 0;
    int x_pix = 0, y_pix = 0;
    float x = 0.0f, y = 0.0f, z = 0.0f;
    for (int r = 0; r < fgMask.rows; ++r) {
      for (int c = 0; c < fgMask.cols; ++c) {
        if (fgMask.at<uchar>(r, c) > 0) {
          // calc bounding box
          minX = (c < minX) ? c : minX;
          maxX = (c > maxX) ? c : maxX;
          minY = (r < minY) ? r : minY;
          maxY = (r > maxY) ? r : maxY;

          int i = r * fgMask.cols + c;
          pcl::PointXYZRGB point = cloudRGB->points.at(i);
          LOG4CXX_DEBUG(logger, boost::format("pixel 3D location (x,y) -> (x,y,z) : (%f,%f) -> (%f,%f,%f)")
                  % r % c % point.x % point.y % point.z);
          // Check if the point is invalid
          if (!std::isfinite(point.x) ||
                  !std::isfinite(point.y) ||
                  !std::isfinite(point.z)) {
            continue;
          }
          ++pixelCount;
          x += point.x;
          y += point.y;
          z += point.z;
          x_pix += c;
          y_pix += r;
        }
      }
    }

    pcl::PointXYZ buttonLoc;
    buttonLoc.x = x / pixelCount;
    buttonLoc.y = y / pixelCount;
    buttonLoc.z = z / pixelCount;
    x_pix /= pixelCount;
    y_pix /= pixelCount;

    LOG4CXX_INFO(logger, boost::format("button bounding pixel count: %d.") % pixelCount);
    LOG4CXX_INFO(logger, boost::format("button bounding box at (min_x,min_y,max_x,max_y) = (%d,%d,%d,%d)") % minX % minY % maxX % maxY);
    LOG4CXX_INFO(logger, boost::format("button centroid at (x,y,z) = (%f,%f,%f)") % buttonLoc.x % buttonLoc.y % buttonLoc.z);

    // prepare display frame
    //    cv::Mat tmpFrame = cv::Mat(frame.rows, frame.cols, CV_32FC1);
    for (int r = 0; r < frame.rows; ++r) {
      for (int c = 0; c < frame.cols; ++c) {
        int i = r * fgMask.cols + c;
        pcl::PointXYZRGB point = cloudRGB->points.at(i);
        if (!std::isfinite(point.x) ||
                !std::isfinite(point.y) ||
                !std::isfinite(point.z)) {
          displayFrame.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 0, 0);
        } else {
          displayFrame.at<cv::Vec3b>(r, c) = frame.at<cv::Vec3b>(r, c);
        }
      }
    }
    //    double min;
    //    double max;
    //    cv::minMaxIdx(tmpFrame, &min, &max);
    //    cv::convertScaleAbs(tmpFrame, displayFrame, 255 / max);
    int lineThickness = 1;
    cv::Rect rect = cv::Rect(minX, minY, maxX - minX, maxY - minY);
    cv::rectangle(displayFrame, cv::Point(rect.x, rect.y),
            cv::Point(rect.x + rect.width, rect.y + rect.height),
            cv::Scalar(255, 255, 255),
            lineThickness, 8, 0);
    cv::rectangle(displayFrame, cv::Point(x_pix, y_pix),
            cv::Point(x_pix, y_pix),
            cv::Scalar(0, 255, 255),
            lineThickness, 8, 0);

#ifdef USE_ROS_VISION

    // transform location into head frame
    geometry_msgs::PointStamped left_camera_optical_frame_point;
    left_camera_optical_frame_point.header.frame_id = "left_camera_optical_frame";
    left_camera_optical_frame_point.header.stamp = ros::Time().fromNSec(notification->frameNum);
    left_camera_optical_frame_point.point.x = buttonLoc.x;
    left_camera_optical_frame_point.point.y = buttonLoc.y;
    left_camera_optical_frame_point.point.z = buttonLoc.z;
    geometry_msgs::PointStamped head_point;
    try {
      listener.transformPoint("head", left_camera_optical_frame_point, head_point);
    } catch (tf::TransformException &ex) {
      LOG4CXX_ERROR(logger, boost::format("Exception transforming point using ros tf: %s.") % std::string(ex.what()));
      return;
    }

    // publish results
    srcsim::Console results_msg;
    results_msg.x = head_point.point.x;
    results_msg.y = head_point.point.y;
    results_msg.z = head_point.point.z;
    results_msg.r = redButtonDetected ? 1.0 : 0.0;
    results_msg.g = greenButtonDetected ? 1.0 : 0.0;
    results_msg.b = blueButtonDetected ? 1.0 : 0.0;
    pub.publish(results_msg);
    ros::spinOnce();
#endif //USE_ROS_VISION

    buttonDetectionReported = true;
  } else if (!buttonDetected) {
    // reset flag
    buttonDetectionReported = false;
  }

  if (getDisplayFlag()) {
    diarc::Display::displayFrame(displayFrame, getDisplayName());
  }
}
