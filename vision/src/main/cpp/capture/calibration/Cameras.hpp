/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef CAMERAS_HPP
#define CAMERAS_HPP

#include "common/VisionConstants.hpp"
#include <boost/shared_ptr.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <log4cxx/logger.h>
#include <opencv2/opencv.hpp>

struct CameraParameters {
  typedef boost::shared_ptr<CameraParameters> Ptr;
  typedef boost::shared_ptr<const CameraParameters> ConstPtr;

  CameraParameters(const int imgWidth, const int imgHeight)
          : width(imgWidth),
            height(imgHeight),
            flen(0),
            fovx(42.0), //defaults to 42 deg
            fovy(32.0), //defaults to 32 deg
            sx(0),
            sy(0),
            asp_ratio(0),
            intrinsicSet(false),
            extrinsicSet(false) {
    M = cv::Mat(3, 3, CV_32F);
    D = cv::Mat(1, 5, CV_32F);
    rVec = cv::Mat(3, 1, CV_32F);
    tVec = cv::Mat(3, 1, CV_32F);
    mx = cv::Mat(imgHeight, imgWidth, CV_32F);
    my = cv::Mat(imgHeight, imgWidth, CV_32F);

    //init params
    M = cv::Mat::zeros(M.size(), M.type());
    M.at<float>(0,0) = M.at<float>(1,1) = 300.0f;       //fx, fy
    M.at<float>(0,2) = imgWidth / 2.0f;                //cx
    M.at<float>(1,2) = imgHeight / 2.0f;               //cy
    M.at<float>(2,2) = 1.0f;

    D = cv::Mat::zeros(D.size(), D.type());

    rVec.at<float>(0) = rVec.at<float>(1) = rVec.at<float>(2) = 0.0;
    tVec.at<float>(0) = tVec.at<float>(1) = tVec.at<float>(2) = 0.0;
  }

  void copyExtrinsicParams(const CameraParameters &params) {
    params.rVec.copyTo(rVec);
    params.tVec.copyTo(tVec);
    extrinsicSet = params.extrinsicSet;
  }

  void copyIntrinsicParams(const CameraParameters &params) {
    params.M.copyTo(M);
    params.D.copyTo(D);
    params.mx.copyTo(mx);
    params.my.copyTo(my);
    flen = params.flen;
    fovx = params.fovx;
    fovy = params.fovy;
    sx = params.sx;
    sy = params.sy;
    asp_ratio = params.asp_ratio;
    intrinsicSet = params.intrinsicSet;
  }

  const int width;
  const int height;

  cv::Mat M, D; //intrinsic camera parameters. Camera matrix and Distortion matrix (k_1, k_2, p_1, p_2, [k_3])
  cv::Mat rVec, tVec; //extrinsic camera parameters. Pose of camera w.r.t. calibration object
  cv::Mat mx, my; //matrices to undistort images (used for cv::remap)

  double flen; //focal length [mm]
  double fovx; //field of view [deg]
  double fovy; //field of view [deg]
  double sx; //pixel size [mm/pix]
  double sy; //pixel size [mm/pix]
  double asp_ratio; //pixel size x/y

  //flags
  bool intrinsicSet;
  bool extrinsicSet;
};

struct StereoCameraParameters {
  typedef boost::shared_ptr<StereoCameraParameters> Ptr;
  typedef boost::shared_ptr<const StereoCameraParameters> ConstPtr;

  StereoCameraParameters(const int imgWidth, const int imgHeight)
          : set(false) {
    R = cv::Mat(3, 3, CV_64F);
    T = cv::Mat(3, 1, CV_64F);
    Rec = cv::Mat(3, 3, CV_32F);
    P = cv::Mat(3, 4, CV_32F);
    Rinv = cv::Mat(3, 1, CV_64F);
    Tinv = cv::Mat(3, 1, CV_64F);
    mx = cv::Mat(imgHeight, imgWidth, CV_32F);
    my = cv::Mat(imgHeight, imgWidth, CV_32F);
    Q = cv::Mat(4, 4, CV_32F);

    //init params
    R.at<float>(0,1)= R.at<float>(0,2) = R.at<float>(1,0)= R.at<float>(1,2) = R.at<float>(2,0) = R.at<float>(2,1) = 0.0;
    R.at<float>(0,0) = R.at<float>(1,1) = R.at<float>(2,2) = 1.0;
    T.at<float>(0) = T.at<float>(1) = T.at<float>(2) = 0.0;
  }

  cv::Mat R, T; //rotation and translation to rectify cameras
  cv::Mat Rec, P; //rectification transform and projection matrix for camera in (virtual) rectified coordinate system
  cv::Mat Rinv, Tinv; //matrices to "undo" rectification (from Rec and P) when projecting 3D depth information onto image planes
  cv::Mat mx, my; //matrices to undistort and rectify images (used for cvRemap). built using intrinsic params and Rec and P.
  cv::Mat Q; //perspective transform (disparity->depth) matrix. (duplicate matrix in both right and left stereo params)

  //flags
  bool set;
};

//class that represents a stereo pair of cameras and their calibration info/
//also used to hold mono camera parameters when stereo isn't being used

class Cameras {
public:
  static Cameras *getNewInstance(const int pairIndex, const int img1_width, const int img1_height, const int img2_width,
                                 const int img2_height);
  static Cameras *getInstance(const int pairIndex = 0);

  //get pointer to const values - no copying done  - get methods ARE thread safe.
  CameraParameters::ConstPtr getCameraParameters(const int camIndex) const;
  StereoCameraParameters::ConstPtr getStereoParameters(const int camIndex) const;

  void setCameraParameters(const int camIndex, CameraParameters::Ptr);
  void setStereoParameters(const int camIndex, StereoCameraParameters::Ptr);

  //NOTE: calibration methods are NOT thread safe!

  //single camera parameters
  bool calibrationSingleCamSave(const int camIndex, const char *filename) const;
  bool calibrationSingleCamLoad(const int camIndex, const char *filename);

  bool calibrateIntrSingleCameraStart(const int camIndex, const int cornersX, const int cornersY, const double gridSize);
  bool calibrateExtrSingleCameraStart(const int camIndex, const int cornersX, const int cornersY, const double gridSize);

  bool takeSnapshot();
  bool takeSnapshots(const int numSnapshots, const int numAllowedFailures = -1);
  bool runCalibration();
  void cancelCalibration();

  //stereo camera parameter methods - load/save also loads/saves CameraParameters info
  bool calibrateStereoCamerasStart(const int cornersX, const int cornersY, const double gridSize);
  bool calibrationStereoSave(const char *filename) const;
  bool calibrationStereoLoad(const char *filename);

private:
  enum CALIB_MODE {
    SINGLE_INTR,
    SINGLE_EXTR,
    STEREO
  };

  Cameras(const int img1_width, const int img1_height, const int img2_width, const int img2_height);
  ~Cameras();

  bool writeMat(FILE *stream, cv::Mat mat) const;
  bool loadMat(FILE *stream, cv::Mat mat);

  // calibration methods
  bool calibrateIntrSingleCameraEnd();
  bool calibrateExtrSingleCameraEnd();
  bool calibrateStereoCamerasEnd();
  bool calibrationStart(const enum CALIB_MODE mode, const int cornersX, const int cornersY, const double gridSize,
                        const int cameraIndex = -1);
  bool calibrationAddSample(cv::Mat imageLeft, cv::Mat imageRight);
  void finishStereoCalculations();
  void set3dProjectionMatrices(); //sets matrices used to project depth data onto image planes
  void computeAdditionaIntrinsiclValues(CameraParameters::Ptr params,
                                        const int cameraIndex); //calculates real world camera values (focal length, aspect ratio, ...)


  //! cameras logger
  static log4cxx::LoggerPtr logger;

  const std::string stereoVersion = "0.2";
  const std::string singleCamVersion = "0.1";
  CameraParameters::Ptr camParams[2];
  StereoCameraParameters::Ptr stereoParams[2];

  mutable boost::shared_mutex cameras_mutex; //reader/writer lock for camParams and stereoParams only!
  //chesboard board corners X,Y, N = X*Y ,  number of corners = (number of cells - 1)
  int cornersX, cornersY, cornersN;
  int sampleCount;

  //chessboard corner points for calibration
  std::vector<cv::Point2d> imgPointsTemp[2];
  std::vector<std::vector<cv::Point3d>> objectPoints;
  std::vector<std::vector<cv::Point2d>> imagePoints[2]; //for calibration
  std::vector<cv::Mat> rVecs;
  std::vector<cv::Mat> tVecs;

  //current calibration task parameters
  bool calibrationStarted;
  CALIB_MODE calibmode;
  int camIndex;
  double gridSize;

  static std::vector<Cameras *> instances;

  friend class StereoCalibration;
};

#endif  //CAMERAS_HPP
