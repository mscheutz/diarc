/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "StereoCalibration.hpp"
#include "display/Display.hpp"
#include "capture/calibration/Cameras.hpp"
#include "capture/Capture.hpp"
#include "common/notification/CaptureNotification.hpp"

#include <pcl/common/intersections.h>
#include <pcl/ModelCoefficients.h>

StereoCalibration::StereoCalibration(const int numCornersX, const int numCornersY, const enum CALIB_MODE calibmode)
: cornersX(numCornersX), cornersY(numCornersY),
calibrationMode(calibmode) {

  pointsPixel[0].resize(cornersX * cornersY);
  pointsPixel[1].resize(cornersX * cornersY);

  //init frames
  CaptureNotification::ConstPtr captureNotification = diarc::capture::Capture::getLastCaptureNotification();

  int frame1_width, frame1_height;
  int frame2_width, frame2_height;
  if (StereoCalibration::DEPTH_RGB_L == calibrationMode) {
    frame1_width = captureNotification->captureData->depthFrameIntensity.cols;
    frame1_height = captureNotification->captureData->depthFrameIntensity.rows;
    frame2_width = captureNotification->captureData->frame.cols;
    frame2_height = captureNotification->captureData->frame.rows;
  } else if (StereoCalibration::DEPTH_RGB_R == calibrationMode) {
    frame1_width = captureNotification->captureData->depthFrameIntensity.cols;
    frame1_height = captureNotification->captureData->depthFrameIntensity.rows;
    frame2_width = captureNotification->captureData->frame2.cols;
    frame2_height = captureNotification->captureData->frame2.rows;
  } else if (StereoCalibration::RGB == calibrationMode) {
    frame1_width = captureNotification->captureData->frame.cols;
    frame1_height = captureNotification->captureData->frame.rows;
    frame2_width = captureNotification->captureData->frame2.cols;
    frame2_height = captureNotification->captureData->frame2.rows;
  }

  stereoPair = Cameras::getInstance(calibmode);
  //camParams[0] = CameraParameters::Ptr(new CameraParameters(frame1_width, frame1_height));
  //camParams[1] = CameraParameters::Ptr(new CameraParameters(frame2_width, frame2_height));
  //stereoParams = StereoCameraParameters::Ptr(new StereoCameraParameters(frame1_width, frame1_height));

  images[0] = cv::Mat(frame1_width, frame1_height, CV_8U);
  images[1] = cv::Mat(frame2_width, frame2_height, CV_8U);
  framesToDraw[0] = cv::Mat(frame1_width, frame1_height, CV_8U);
  framesToDraw[1] = cv::Mat(frame2_width, frame2_height, CV_8U);
}

StereoCalibration::~StereoCalibration() {
}

int StereoCalibration::getParametersSize() const {
  return 28;
}

//order of params:
//extrinsic: [r00 r01 r02 r10...r33][t0 t1 t2]
//intrinsic cam1: [fx fy cx cy][k1 k2 p1 p2]
//intrinsic cam2: [fx fy cx cy][k1 k2 p1 p2]

void StereoCalibration::getParameters(double* params) {
  int i = 0;
  //extrinsic Rotation
  params[i++] = stereoPair->stereoParams[0]->R.at<float>(0,0);
  params[i++] = stereoPair->stereoParams[0]->R.at<float>(0,1);
  params[i++] = stereoPair->stereoParams[0]->R.at<float>(0,2);
  params[i++] = stereoPair->stereoParams[0]->R.at<float>(1,0);
  params[i++] = stereoPair->stereoParams[0]->R.at<float>(1,1);
  params[i++] = stereoPair->stereoParams[0]->R.at<float>(1,2);
  params[i++] = stereoPair->stereoParams[0]->R.at<float>(2,0);
  params[i++] = stereoPair->stereoParams[0]->R.at<float>(2,1);
  params[i++] = stereoPair->stereoParams[0]->R.at<float>(2,2);

  //extrinsic Translation
  params[i++] = stereoPair->stereoParams[0]->T.at<float>(0);
  params[i++] = stereoPair->stereoParams[0]->T.at<float>(1);
  params[i++] = stereoPair->stereoParams[0]->T.at<float>(2);

  //intrinsic Camera 1
  params[i++] = stereoPair->camParams[0]->M.at<float>(0,0);
  params[i++] = stereoPair->camParams[0]->M.at<float>(1,1);
  params[i++] = stereoPair->camParams[0]->M.at<float>(0,2);
  params[i++] = stereoPair->camParams[0]->M.at<float>(1,2);
  params[i++] = stereoPair->camParams[0]->D.at<float>(0);
  params[i++] = stereoPair->camParams[0]->D.at<float>(1);
  params[i++] = stereoPair->camParams[0]->D.at<float>(2);
  params[i++] = stereoPair->camParams[0]->D.at<float>(3);

  //intrinsic Camera 2
  params[i++] = stereoPair->camParams[1]->M.at<float>(0,0);
  params[i++] = stereoPair->camParams[1]->M.at<float>(1,1);
  params[i++] = stereoPair->camParams[1]->M.at<float>(0,2);
  params[i++] = stereoPair->camParams[1]->M.at<float>(1,2);
  params[i++] = stereoPair->camParams[1]->D.at<float>(0);
  params[i++] = stereoPair->camParams[1]->D.at<float>(1);
  params[i++] = stereoPair->camParams[1]->D.at<float>(2);
  params[i++] = stereoPair->camParams[1]->D.at<float>(3);

  //    printf("native params =\n");
  //    for (int j = 0; j < i; ++j) {
  //        printf("%f ", params[j]);
  //    }
  //    printf("\n\n");

}

bool StereoCalibration::takeSnapshot() {
  //get images
  CaptureNotification::ConstPtr captureNotification = diarc::capture::Capture::getLastCaptureNotification();

  if (StereoCalibration::DEPTH_RGB_L == calibrationMode) {
    cv::cvtColor(captureNotification->captureData->depthFrameIntensity, images[0], cv::COLOR_BGR2GRAY);
    cv::cvtColor(captureNotification->captureData->frame, images[1], cv::COLOR_BGR2GRAY);
  } else if (StereoCalibration::DEPTH_RGB_R == calibrationMode) {
    cv::cvtColor(captureNotification->captureData->depthFrameIntensity, images[0], cv::COLOR_BGR2GRAY);
    cv::cvtColor(captureNotification->captureData->frame2, images[1], cv::COLOR_BGR2GRAY);
  } else if (StereoCalibration::RGB == calibrationMode) {
    cv::cvtColor(captureNotification->captureData->frame, images[0], cv::COLOR_BGR2GRAY);
    cv::cvtColor(captureNotification->captureData->frame2, images[1], cv::COLOR_BGR2GRAY);
  }

  //checkerboard detection
  bool frame1results = findCheckerboardPoints(0);
  bool frame2results = findCheckerboardPoints(1);

  return (frame1results && frame2results);
}

void StereoCalibration::getDatum(int index, double* datum) const {
  int indexToPoints = -1;
  switch (index) {
    case 0:
      //top left
      indexToPoints = 0;
      break;
    case 1:
      //top right
      indexToPoints = cornersX - 1;
      break;
    case 2:
      //bottom left
      indexToPoints = cornersX * (cornersY - 1);
      break;
    case 3:
      //bottom right
      indexToPoints = (cornersX * cornersY) - 1;
      break;
  }

  //printf("indexToPoints: %d points size: %d\n", indexToPoints, pointsPixel[0].size());
  if (indexToPoints + 1 <= pointsPixel[0].size() && indexToPoints + 1 <= pointsPixel[1].size()) {
    datum[0] = pointsPixel[0][indexToPoints].x;
    datum[1] = pointsPixel[0][indexToPoints].y;

    datum[2] = pointsPixel[1][indexToPoints].x;
    datum[3] = pointsPixel[1][indexToPoints].y;
    printf("[StereoCalibration::getDatum] [%f %f], [%f %f]\n", datum[0], datum[1], datum[2], datum[3]);
  } else {
    printf("[StereoCalibration::getDatum] Error\n");
  }
}

void StereoCalibration::getPoint(const double* params, const double* datum, double* point3d) {
  //set local camera parameters
  setParameters(params);

  //    printf("[StereoCalibration::getPoint] Rotation = \n[%f %f %f]\n[%f %f %f]\n[%f %f %f]\n", stereoPair->stereoParams[0]->R.at<float>(0), stereoPair->stereoParams[0]->R->data.db[1], stereoPair->stereoParams[0]->R.at<float>(2),
  //            stereoPair->stereoParams[0]->R.at<float>(3), stereoPair->stereoParams[0]->R.at<float>(4), stereoPair->stereoParams[0]->R.at<float>(5),
  //            stereoPair->stereoParams[0]->R.at<float>(6), stereoPair->stereoParams[0]->R.at<float>(7), stereoPair->stereoParams[0]->R.at<float>(8));
  //    printf("[StereoCalibration::getPoint] Translation = \n[%f %f %f]\n", stereoPair->stereoParams[0]->T.at<float>(0), stereoPair->stereoParams[0]->T.at<float>(1), stereoPair->stereoParams[0]->T.at<float>(2));

  //printf("[StereoCalibration::getPoint] original datum: [%f %f], [%f %f]\n", datum[0], datum[1], datum[2], datum[3]);

  //undistort image points
  //TODO: should this be using the camera matrix here, or is that info being used
  //twice since it's also used to compute rays through image plane?
  cv::Mat imagePoints1(1, 1, CV_32FC2);
  imagePoints1.at<float>(0) = datum[0];
  imagePoints1.at<float>(1) = datum[1];
  cv::Mat imagePoints2(1, 1, CV_32FC2);
  imagePoints2.at<float>(0) = datum[2];
  imagePoints2.at<float>(1) = datum[3];
  cv::undistortPoints(imagePoints1, imagePoints1, stereoPair->camParams[0]->M, stereoPair->camParams[0]->D);
  imagePoints1.at<float>(0) = imagePoints1.at<float>(0) * stereoPair->camParams[0]->M.at<float>(0,0) + stereoPair->camParams[0]->M.at<float>(0,2);
  imagePoints1.at<float>(1) = imagePoints1.at<float>(1) * stereoPair->camParams[0]->M.at<float>(1,1) + stereoPair->camParams[0]->M.at<float>(1,2);
  cv::undistortPoints(imagePoints2, imagePoints2, stereoPair->camParams[1]->M, stereoPair->camParams[1]->D);
  imagePoints2.at<float>(0) = imagePoints2.at<float>(0) * stereoPair->camParams[1]->M.at<float>(0,0) + stereoPair->camParams[1]->M.at<float>(0,2);
  imagePoints2.at<float>(1) = imagePoints2.at<float>(1) * stereoPair->camParams[1]->M.at<float>(1,1) + stereoPair->camParams[1]->M.at<float>(1,2);

  //printf("[StereoCalibration::getPoint] undistorted datum: [%f %f], [%f %f]\n", imagePoints1.at<float>(0), imagePoints1->data.fl[1], imagePoints2.at<float>(0), imagePoints2->data.fl[1]);

  //build rays from cameras through pixel
  pcl::ModelCoefficients::Ptr line_cam1(new pcl::ModelCoefficients());
  //point on line:  (0,0,0)
  line_cam1->values.push_back(0.0);
  line_cam1->values.push_back(0.0);
  line_cam1->values.push_back(0.0);
  //direction vector:  dir_vec = (x,y,z) = ((u/fx)-(cx/fx), (v/fy)-(cy/fy), 1)
  line_cam1->values.push_back((imagePoints1.at<float>(0) / stereoPair->camParams[0]->M.at<float>(0,0)) - (stereoPair->camParams[0]->M.at<float>(0,2) / stereoPair->camParams[0]->M.at<float>(0,0)));
  line_cam1->values.push_back((imagePoints1.at<float>(1) / stereoPair->camParams[0]->M.at<float>(1,1)) - (stereoPair->camParams[0]->M.at<float>(1,2) / stereoPair->camParams[0]->M.at<float>(1,1)));
  line_cam1->values.push_back(1.0);

  //temporary correction to match opencv camera calibration coordinates (change x and z signs)
  line_cam1->values[3] = -line_cam1->values[3];
  line_cam1->values[5] = -line_cam1->values[5];


  //    printf("[StereoCalibration::getPoint] Line 1 = \n [%f %f %f] [%f %f %f]\n", line_cam1->values[0], line_cam1->values[1], line_cam1->values[2],
  //            line_cam1->values[3], line_cam1->values[4], line_cam1->values[5]);

  pcl::ModelCoefficients::Ptr line_cam2(new pcl::ModelCoefficients());
  //point on line: R*(0,0,0) + t = t
  line_cam2->values.push_back(stereoPair->stereoParams[0]->T.at<float>(0));
  line_cam2->values.push_back(stereoPair->stereoParams[0]->T.at<float>(1));
  line_cam2->values.push_back(stereoPair->stereoParams[0]->T.at<float>(2));
  //direction vector:  R*(dir_vec) + t = R*(dir_vec)
  double dirVecX = (imagePoints2.at<float>(0) / stereoPair->camParams[1]->M.at<float>(0,0)) - (stereoPair->camParams[1]->M.at<float>(0,2) / stereoPair->camParams[1]->M.at<float>(0,0));
  double dirVecY = (imagePoints2.at<float>(1) / stereoPair->camParams[1]->M.at<float>(1,1)) - (stereoPair->camParams[1]->M.at<float>(1,2) / stereoPair->camParams[1]->M.at<float>(1,1));
  double dirVecZ = 1.0;

  //temporary correction to match opencv camera calibration coordinates (change x and z signs)
  dirVecX = -dirVecX;
  dirVecZ = -dirVecZ;

  line_cam2->values.push_back(stereoPair->stereoParams[0]->R.at<float>(0) * dirVecX + stereoPair->stereoParams[0]->R.at<float>(1) * dirVecY + stereoPair->stereoParams[0]->R.at<float>(2) * dirVecZ);
  line_cam2->values.push_back(stereoPair->stereoParams[0]->R.at<float>(3) * dirVecX + stereoPair->stereoParams[0]->R.at<float>(4) * dirVecY + stereoPair->stereoParams[0]->R.at<float>(5) * dirVecZ);
  line_cam2->values.push_back(stereoPair->stereoParams[0]->R.at<float>(6) * dirVecX + stereoPair->stereoParams[0]->R.at<float>(7) * dirVecY + stereoPair->stereoParams[0]->R.at<float>(8) * dirVecZ);

  //    printf("[StereoCalibration::getPoint] Line 2 = \n [%f %f %f] [%f %f %f]\n", line_cam2->values[0], line_cam2->values[1], line_cam2->values[2],
  //            line_cam2->values[3], line_cam2->values[4], line_cam2->values[5]);

  //find 3D point minimizing distance between rays
  Eigen::Vector4f localPoint3d;
  Eigen::VectorXf coeff1 = Eigen::VectorXf::Map(&(line_cam1->values[0]), line_cam1->values.size());
  Eigen::VectorXf coeff2 = Eigen::VectorXf::Map(&(line_cam2->values[0]), line_cam2->values.size());

  //bool intersection_result = pcl::lineWithLineIntersection(coeff1, coeff2, localPoint3d);

  Eigen::Vector4f p1_seg;
  Eigen::Vector4f p2_seg;
  pcl::lineToLineSegment(coeff1, coeff2, p1_seg, p2_seg);

  localPoint3d = (p1_seg + p2_seg) / 2.0;

  point3d[0] = static_cast<double> (localPoint3d.x());
  point3d[1] = static_cast<double> (localPoint3d.y());
  point3d[2] = static_cast<double> (localPoint3d.z());

  //printf("[StereoCalibration::getPoint] segment 1: [%f %f %f]\n", p1_seg.x(), p1_seg.y(), p1_seg.z());
  //printf("[StereoCalibration::getPoint] segment 2: [%f %f %f]\n", p2_seg.x(), p2_seg.y(), p2_seg.z());
  //printf("[StereoCalibration::getPoint] 3D intersection: [%f %f %f]\n", point3d[0], point3d[1], point3d[2]);
  //printf("\n\n");

  //START TESTING /////////////////////////
  //    pcl::ModelCoefficients::Ptr line_cam1(new pcl::ModelCoefficients());
  //    line_cam1->values.push_back(-1.0);
  //    line_cam1->values.push_back(-1.0);
  //    line_cam1->values.push_back(0.0);
  //    line_cam1->values.push_back(1.0);
  //    line_cam1->values.push_back(1.0);
  //    line_cam1->values.push_back(0.0);
  //
  //    pcl::ModelCoefficients::Ptr line_cam2(new pcl::ModelCoefficients());
  //    line_cam2->values.push_back(1.0);
  //    line_cam2->values.push_back(-1.0);
  //    line_cam2->values.push_back(1.0);
  //    line_cam2->values.push_back(-1.0);
  //    line_cam2->values.push_back(1.0);
  //    line_cam2->values.push_back(0.0);
  //
  //    //find 3D point minimizing distance between rays
  //    Eigen::Vector4f localPoint3d;
  //    Eigen::VectorXf coeff1 = Eigen::VectorXf::Map(&(line_cam1->values[0]), line_cam1->values.size());
  //    Eigen::VectorXf coeff2 = Eigen::VectorXf::Map(&(line_cam2->values[0]), line_cam2->values.size());
  //    Eigen::Vector4f p1_seg;
  //    Eigen::Vector4f p2_seg;
  //    pcl::lineToLineSegment(coeff1, coeff2, p1_seg, p2_seg);
  //
  //    localPoint3d = (p1_seg + p2_seg) / 2.0;
  //
  //    printf("[StereoCalibration::getPoint] segment 1: [%f %f %f]\n", p1_seg.x(), p1_seg.y(), p1_seg.z());
  //    printf("[StereoCalibration::getPoint] segment 2: [%f %f %f]\n", p2_seg.x(), p2_seg.y(), p2_seg.z());
  //    printf("[StereoCalibration::getPoint] 3D intersection: [%f %f %f]\n", localPoint3d.x(), localPoint3d.y(), localPoint3d.z());
  // END TESTING ///////////////////////////

}

bool StereoCalibration::findCheckerboardPoints(const int imgIndex) {
  int cornersDetected = 0;
  int numCorners = cornersX * cornersY;

  //FIND CHESSBOARDS AND CORNERS:
  int result = cv::findChessboardCorners(
          images[imgIndex], cv::Size(cornersX, cornersY),
          pointsPixel[imgIndex],
          cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
          );

  if (result && cornersDetected == numCorners) {
    //Calibration will suffer without sub-pixel interpolation
    cv::find4QuadCornerSubpix(
            images[imgIndex], pointsPixel[imgIndex],
            cv::Size(3, 3)
            );

    //display
    std::ostringstream oss;
    oss << imgIndex;
    std::string windowName = "StereoCalibration";
    windowName += oss.str();

    diarc::Display::createWindowIfDoesNotExist(windowName);
    images[imgIndex].copyTo(framesToDraw[imgIndex]);
    for (int i = 0; i < cornersDetected; ++i) {
      cv::circle(framesToDraw[imgIndex], cv::Point(pointsPixel[imgIndex][i].x, pointsPixel[imgIndex][i].y),
              2, CV_RGB(255, 255, 255), 1, 8, 0);
    }
    diarc::Display::displayFrame(framesToDraw[imgIndex], windowName);
    return true;
  } else {
    printf("StereoCalibration failed to detect corners on image: %d.\n", imgIndex);
    return false;
  }
}

void StereoCalibration::setParameters(const double* params) {
  int i = 0;
  //extrinsic Rotation
  stereoPair->stereoParams[0]->R.at<float>(0) = params[i++];
  stereoPair->stereoParams[0]->R.at<float>(1) = params[i++];
  stereoPair->stereoParams[0]->R.at<float>(2) = params[i++];
  stereoPair->stereoParams[0]->R.at<float>(3) = params[i++];
  stereoPair->stereoParams[0]->R.at<float>(4) = params[i++];
  stereoPair->stereoParams[0]->R.at<float>(5) = params[i++];
  stereoPair->stereoParams[0]->R.at<float>(6) = params[i++];
  stereoPair->stereoParams[0]->R.at<float>(7) = params[i++];
  stereoPair->stereoParams[0]->R.at<float>(8) = params[i++];

  //extrinsic Translation
  stereoPair->stereoParams[0]->T.at<float>(0) = params[i++];
  stereoPair->stereoParams[0]->T.at<float>(1) = params[i++];
  stereoPair->stereoParams[0]->T.at<float>(2) = params[i++];

  //intrinsic Camera 1
  stereoPair->camParams[0]->M.at<float>(0,0) = params[i++];
  stereoPair->camParams[0]->M.at<float>(1,1) = params[i++];
  stereoPair->camParams[0]->M.at<float>(0,2) = params[i++];
  stereoPair->camParams[0]->M.at<float>(1,2) = params[i++];
  stereoPair->camParams[0]->D.at<float>(0) = params[i++];
  stereoPair->camParams[0]->D.at<float>(1) = params[i++];
  stereoPair->camParams[0]->D.at<float>(2) = params[i++];
  stereoPair->camParams[0]->D.at<float>(3) = params[i++];

  //intrinsic Camera 2
  stereoPair->camParams[1]->M.at<float>(0,0) = params[i++];
  stereoPair->camParams[1]->M.at<float>(1,1) = params[i++];
  stereoPair->camParams[1]->M.at<float>(0,2) = params[i++];
  stereoPair->camParams[1]->M.at<float>(1,2) = params[i++];
  stereoPair->camParams[1]->D.at<float>(0) = params[i++];
  stereoPair->camParams[1]->D.at<float>(1) = params[i++];
  stereoPair->camParams[1]->D.at<float>(2) = params[i++];
  stereoPair->camParams[1]->D.at<float>(3) = params[i++];
}
