/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include <string>
#include <opencv2/opencv.hpp>

#include "Cameras.hpp"
#include "display/Display.hpp"
#include "Capture.hpp"

std::vector<Cameras*> Cameras::instances;
log4cxx::LoggerPtr Cameras::logger = log4cxx::Logger::getLogger("ade.capture.calibration.Cameras");

Cameras* Cameras::getNewInstance(const int pairIndex, const int img1_width, const int img1_height, const int img2_width, const int img2_height) {
  //initialize all to NULL - can have up to 3 stereo pairs per vision server
  if (instances.size() == 0) {
    instances.resize(3, NULL);
  }

  Cameras* instance = instances[pairIndex];
  if (instance == NULL) {
    instance = new Cameras(img1_width, img1_height, img2_width, img2_height);
    instances[pairIndex] = instance;
  }
  return instance;
}

Cameras* Cameras::getInstance(const int pairIndex) {
  //initialize all to NULL - can have up to 3 stereo pairs per vision server
  if (instances.size() == 0) {
    printf("[Cameras::getInstance] ERROR: Index %d does not exist yet.\n", pairIndex);
    return NULL;
  }

  Cameras* instance = instances[pairIndex];
  if (instance == NULL) {
    printf("[Cameras::getInstance] ERROR: Index %d does not exist.\n", pairIndex);
  }
  return instance;
}

Cameras::Cameras(const int img1_width, const int img1_height, const int img2_width, const int img2_height)
: calibrationStarted(false),
camIndex(-1) {
  camParams[0] = CameraParameters::Ptr(new CameraParameters(img1_width, img1_height));
  camParams[1] = CameraParameters::Ptr(new CameraParameters(img2_width, img2_height));
  stereoParams[0] = StereoCameraParameters::Ptr(new StereoCameraParameters(img1_width, img1_height));
  stereoParams[1] = StereoCameraParameters::Ptr(new StereoCameraParameters(img2_width, img2_height));
}

Cameras::~Cameras() {
  for (int i = 0; i < instances.size(); ++i) {
    if (instances[i] != NULL) {
      delete instances[i];
    }
  }
}

CameraParameters::ConstPtr Cameras::getCameraParameters(const int cameraIndex) const {
  //reader lock
  boost::shared_lock<boost::shared_mutex> lock(cameras_mutex);

  return camParams[cameraIndex];
}

StereoCameraParameters::ConstPtr Cameras::getStereoParameters(const int cameraIndex) const {
  //reader lock
  boost::shared_lock<boost::shared_mutex> lock(cameras_mutex);

  return stereoParams[cameraIndex];
}

void Cameras::setCameraParameters(const int cameraIndex, CameraParameters::Ptr cameraParameters) {
  //write lock
  boost::unique_lock<boost::shared_mutex> lock(cameras_mutex);

  camParams[cameraIndex] = cameraParameters;
}

void Cameras::setStereoParameters(const int cameraIndex, StereoCameraParameters::Ptr stereoParameters) {
  //write lock
  boost::unique_lock<boost::shared_mutex> lock(cameras_mutex);

  stereoParams[cameraIndex] = stereoParameters;
}

//
//void Cameras::getCameraParameters(const int cameraIndex, CameraParameters::Ptr params) const {
//    //reader lock
//    boost::shared_lock<boost::shared_mutex> lock(cameras_mutex);
//
//    if (CameraParameters::Ptr() == params) params = CameraParameters::Ptr(new CameraParameters(VisionConstants::imgWidth, VisionConstants::imgHeight));
//
//    cvCopy(camParams[cameraIndex]->M, params->M);
//    cvCopy(camParams[cameraIndex]->D, params->D);
//    cvCopy(camParams[cameraIndex]->rVec, params->rVec);
//    cvCopy(camParams[cameraIndex]->tVec, params->tVec);
//    cvCopy(camParams[cameraIndex]->mx, params->mx);
//    cvCopy(camParams[cameraIndex]->my, params->my);
//    params->flen = camParams[cameraIndex]->flen;
//    params->fovx = camParams[cameraIndex]->fovx;
//    params->fovy = camParams[cameraIndex]->fovy;
//    params->sx = camParams[cameraIndex]->sx;
//    params->sy = camParams[cameraIndex]->sy;
//    params->asp_ratio = camParams[cameraIndex]->asp_ratio;
//    params->intrinsicSet = camParams[cameraIndex]->intrinsicSet;
//    params->extrinsicSet = camParams[cameraIndex]->extrinsicSet;
//}
//
//void Cameras::getStereoParameters(const int cameraIndex, StereoCameraParameters::Ptr params) const {
//    //reader lock
//    boost::shared_lock<boost::shared_mutex> lock(cameras_mutex);
//
//    if (StereoCameraParameters::Ptr() == params) params = StereoCameraParameters::Ptr(new StereoCameraParameters(VisionConstants::imgWidth, VisionConstants::imgHeight));
//
//    cvCopy(stereoParams[cameraIndex]->R, params->R);
//    cvCopy(stereoParams[cameraIndex]->P, params->P);
//    cvCopy(stereoParams[cameraIndex]->Rinv, params->Rinv);
//    cvCopy(stereoParams[cameraIndex]->Tinv, params->Tinv);
//    cvCopy(stereoParams[cameraIndex]->mx, params->mx);
//    cvCopy(stereoParams[cameraIndex]->my, params->my);
//    cvCopy(stereoParams[cameraIndex]->Q, params->Q);
//    params->set = stereoParams[cameraIndex]->set;
//}

//********************* single camera parameter methods ****************************

bool Cameras::calibrateIntrSingleCameraStart(const int cameraIndex, const int cornersX, const int cornersY, const double gridSize) {
  return calibrationStart(SINGLE_INTR, cornersX, cornersY, gridSize, cameraIndex); //num of inner corners on checkerboard
}

bool Cameras::calibrateIntrSingleCameraEnd() {
  if (!calibrationStarted || SINGLE_INTR != calibmode) return false;

  // HARVEST CHESSBOARD 3D OBJECT POINT LIST:
  objectPoints.resize(sampleCount);
  for (int k = 0; k < sampleCount; k++) {
    std::vector<cv::Point3d> patternPoints;
    for (int i = 0; i < cornersY; i++) {
      for (int j = 0; j < cornersX; j++) {
        patternPoints.push_back(cv::Point3d(i, j, 0));
      }
    }
    objectPoints.push_back(patternPoints);
  }


  //SET SINGLE CAMERA INTRINSIC PARAMETERS

  //get image dimensions
  int imgWidth = camParams[camIndex]->width;
  int imgHeight = camParams[camIndex]->height;
  printf("[calibrateIntrSingleCameraEnd] width: %d height: %d\n", imgWidth, imgHeight);

  //writer lock
  boost::unique_lock<boost::shared_mutex> lock(cameras_mutex);

  //keep existing extrinsic params in case they were already set
  CameraParameters::Ptr tempCamParams = camParams[camIndex];
  camParams[camIndex].reset(new CameraParameters(imgWidth, imgHeight));
  camParams[camIndex]->copyExtrinsicParams(*tempCamParams);

  //reset intrinsic params
  cv::setIdentity(camParams[camIndex]->M);
  camParams[camIndex]->D = cv::Mat::zeros(camParams[camIndex]->D.size(), camParams[camIndex]->D.type());

  //calc intrinsic params
  double error = cv::calibrateCamera(objectPoints, imagePoints[camIndex], cv::Size(imgWidth, imgHeight),
                                     camParams[camIndex]->M, camParams[camIndex]->D,
                                     rVecs, tVecs); //, NULL, NULL, CV_CALIB_SAME_FOCAL_LENGTH + CV_CALIB_ZERO_TANGENT_DIST);
  printf("Calibration finished with error: %f\n", error);

  computeAdditionaIntrinsiclValues(camParams[camIndex], camIndex);

  camParams[camIndex]->intrinsicSet = true;
  calibrationStarted = false;

  return true;
}

void Cameras::computeAdditionaIntrinsiclValues(CameraParameters::Ptr params, const int cameraIndex) {

  double ap_width = 0;
  double ap_height = 0;
  double fovx = 0;
  double fovy = 0;
  double flen = 0;
  cv::Point2d pp(0, 0);
  double asp_ratio = 0;
  int imgWidth = camParams[cameraIndex]->width;
  int imgHeight = camParams[cameraIndex]->height;

  cv::calibrationMatrixValues(params->M, cv::Size(imgWidth, imgHeight),
          ap_width, ap_height, fovx, fovy, flen, pp, asp_ratio);

  //convert fov from degrees to radians
  double fovx_rad = (fovx)*(PI / 180.0);
  double fovy_rad = (fovy)*(PI / 180.0);

  //compute pixel size [mm/pix]
  double sx = (2.0 * flen * std::tan(fovx_rad / 2.0)) / static_cast<double> (imgWidth);
  double sy = (2.0 * flen * std::tan(fovy_rad / 2.0)) / static_cast<double> (imgHeight);

  params->flen = flen;
  params->fovx = fovx;
  params->fovy = fovy;
  params->sx = sx;
  params->sy = sy;
  params->asp_ratio = asp_ratio;

  //compute undistort maps
  cv::Mat newM;
  cv::initUndistortRectifyMap(params->M, params->D,
                              cv::Mat::eye(3,3,CV_64F), newM,
                              cv::Size(camParams[0]->width, camParams[0]->height), CV_32FC1,
                              params->mx, params->my);

  //    cout << "apeture [w h]: " << ap_width << " " << ap_height << endl;
  //    cout << "focal len: " << flen << endl;
  //    cout << "focal len pix [w h]: " << cvmGet(camParams[camIndex].M,0,0) << " " << cvmGet(camParams[camIndex].M,1,1) << endl;
  //    cout << "fov [w h]: " << fovx << " " << fovy << endl;
  //    cout << "pp [w h]: " << pp.x << " " << pp.y << endl;
  //    cout << "pp pix [w h]: " << cvmGet(camParams[camIndex].M,0,2) << " " << cvmGet(camParams[camIndex].M,1,2) << endl;
  //    cout << "aspect ratio: " << asp_ratio << endl;
  //    cout << "pixel size(mm) [w h]: " << sx << " " << sy << endl;
  //
  //    cout << "distortion: " << cvmGet(camParams[camIndex].D, 0, 0) << " " << cvmGet(camParams[camIndex].D, 0, 1) << " " << cvmGet(camParams[camIndex].D, 0, 2)
  //            << " " << cvmGet(camParams[camIndex].D, 0, 3) << " " << cvmGet(camParams[camIndex].D, 0, 4) << endl;

}

bool Cameras::calibrateExtrSingleCameraStart(const int cameraIndex, const int cornersX, const int cornersY, const double gridSize) {
  if (!camParams[cameraIndex]->intrinsicSet) {
    LOG4CXX_ERROR(logger,"[calibrateExtrSingleCameraStart] Failed to perform extrinsic calibration. Be sure intrinsic camera calibration has been done!");
    return false;
  }

  return calibrationStart(SINGLE_EXTR, cornersX, cornersY, gridSize, cameraIndex); //num of inner corners on checkerboard
}

bool Cameras::calibrateExtrSingleCameraEnd() {
  if (!calibrationStarted || SINGLE_EXTR != calibmode) return false;

  // HARVEST CHESSBOARD 3D OBJECT POINT LIST:
  objectPoints.resize(sampleCount);
  for (int k = 0; k < sampleCount; k++) {
    std::vector<cv::Point3d> patternPoints;
    for (int i = 0; i < cornersY; i++) {
      for (int j = 0; j < cornersX; j++) {
        patternPoints.push_back(cv::Point3d(i, j, 0));
      }
    }
    objectPoints.push_back(patternPoints);
  }

  //CALIBRATE SINGLE CAMERA EXTRINSIC PARAMETERS
  double error = cv::calibrateCamera(objectPoints, imagePoints[camIndex],
                                     cv::Size(camParams[camIndex]->width, camParams[camIndex]->height),
                                     camParams[camIndex]->M, camParams[camIndex]->D,
                                     rVecs, tVecs);

  // only care about a single calibration image when calculating extrinsic for single cam
  cv::Mat rVec = rVecs[0];
  cv::Mat tVec = tVecs[0];

  //Convert to Pose of camera w.r.t. calibration object
  cv::Mat r(3, 3, CV_64F);
  cv::Rodrigues(rVec, r);

  cv::Mat rInv(3, 3, CV_64F);
  cv::invert(r, rInv);

  cv::Mat rInvVec(3, 1, CV_64F);
  cv::Rodrigues(rInv, rInvVec);

  cv::Mat tInvVec(3, 1, CV_64F);
  tInvVec = rInv * tVec;

  //cout << "rot: " << cvmGet(rInvVec, 0, 0) << " " << cvmGet(rInvVec, 1, 0) << " " << cvmGet(rInvVec, 2, 0) << endl;
  //cout << "tran: " << cvmGet(tInvVec, 0, 0) << " " << cvmGet(tInvVec, 1, 0) << " " << cvmGet(tInvVec, 2, 0) << endl;
  //cout << "tran: " << -cvmGet(tInvVec, 0, 0) * VisionConstants::mPerSquare << " " << -cvmGet(tInvVec, 1, 0) * VisionConstants::mPerSquare << " " << -cvmGet(tInvVec, 2, 0) * VisionConstants::mPerSquare << endl;

  //get image dimensions
  int imgWidth = camParams[camIndex]->width;
  int imgHeight = camParams[camIndex]->height;

  //writer lock
  boost::unique_lock<boost::shared_mutex> lock(cameras_mutex);

  //keep existing intrinsic params
  CameraParameters::Ptr tempCamParams = camParams[camIndex];
  camParams[camIndex].reset(new CameraParameters(imgWidth, imgHeight));
  camParams[camIndex]->copyIntrinsicParams(*tempCamParams); //helper that makes deep copy

  //set parameters
  camParams[camIndex]->tVec.at<float>(0) = -tInvVec.at<float>(0) * gridSize;
  camParams[camIndex]->tVec.at<float>(1) = -tInvVec.at<float>(1) * gridSize;
  camParams[camIndex]->tVec.at<float>(2) = -tInvVec.at<float>(2) * gridSize;
  camParams[camIndex]->rVec.at<float>(0) = -rInvVec.at<float>(0);
  camParams[camIndex]->rVec.at<float>(1) = -rInvVec.at<float>(1);
  camParams[camIndex]->rVec.at<float>(2) = -rInvVec.at<float>(2);

  camParams[camIndex]->extrinsicSet = true;
  calibrationStarted = false;

  return true;
}

bool Cameras::writeMat(FILE* stream, cv::Mat mat) const {
  int rows = mat.rows;
  int cols = mat.cols;
  for (size_t r = 0; r < rows; ++r) {
    for (size_t c = 0; c < cols; ++c) {
      if (fprintf(stream, "%f ", mat.at<float>(r,c)) < 0) return false;
    }
    fprintf(stream, "\n");
  }
  fprintf(stream, "\n");
  return true;
}

bool Cameras::loadMat(FILE* stream, cv::Mat mat) {
  int rows = mat.rows;
  int cols = mat.cols;
  for (size_t r = 0; r < rows; ++r) {
    for (size_t c = 0; c < cols; ++c) {
      if (fscanf(stream, "%f", &(mat.at<float>(r,c))) != 1) return false;
    }
  }
  return true;
}

bool Cameras::calibrationSingleCamSave(const int cameraIndex, const char* filename) const {
  FILE* f = fopen(filename, "wt");
  if (!f) {
    LOG4CXX_ERROR(logger,boost::format("[calibrationSingleCamSave] Could not open file: ") % filename);
    return false;
  }

  //reader lock
  boost::shared_lock<boost::shared_mutex> lock(cameras_mutex);

  //version number
  fprintf(f, "Version %s\n\n", singleCamVersion.c_str());

  //image size
  int imgWidth = camParams[cameraIndex]->width;
  int imgHeight = camParams[cameraIndex]->height;
  fprintf(f, "Image Size =\n");
  fprintf(f, "%d %d\n\n", imgWidth, imgHeight);

  //save intrinsic params
  fprintf(f, "Intrinsic Parameters:\n");
  fprintf(f, "Camera matrix =\n");
  if (!writeMat(f, camParams[cameraIndex]->M)) return false;
  fprintf(f, "Distortion params =\n");
  if (!writeMat(f, camParams[cameraIndex]->D)) return false;

  //save extrinsic params
  fprintf(f, "Extrinsic Parameters:\n");
  fprintf(f, "Translation params =\n");
  if (!writeMat(f, camParams[cameraIndex]->tVec)) return false;
  fprintf(f, "Rotation params =\n");
  if (!writeMat(f, camParams[cameraIndex]->rVec)) return false;

  fclose(f);
  return true;
}

bool Cameras::calibrationSingleCamLoad(const int cameraIndex, const char* filename) {
  FILE* f = fopen(filename, "rt");
  if (!f) {
    LOG4CXX_ERROR(logger,boost::format("[calibrationSingleCamLoad] Could not open file: ") % filename);
    return false;
  }

  //tmp cam params incase load fails
  int imgWidth = camParams[cameraIndex]->width;
  int imgHeight = camParams[cameraIndex]->height;
  CameraParameters::Ptr tmpCamParams = CameraParameters::Ptr(new CameraParameters(imgWidth, imgHeight));

  //version number
  char tmp[1000];
  if (fscanf(f, "%*s %s", tmp) == EOF) return false;
  std::string version = tmp;
  if (version.compare(singleCamVersion) != 0) {
    LOG4CXX_ERROR(logger, boost::format("[calibrationSingleCamLoad] trying to load calibration file with wrong version number: ") % filename);
    return false;
  }

  //image size
  int calibratedImgWidth = -1;
  int calibratedImgHeight = -1;
  if (fscanf(f, "%*s %*s %*s") == EOF) return false;
  if (fscanf(f, "%d %d", &calibratedImgWidth, &calibratedImgHeight) != 2) return false;

  //load intrinsic params
  if (fscanf(f, "%*s %*s") == EOF) return false;
  if (fscanf(f, "%*s %*s %*s") == EOF) return false;
  if (!loadMat(f, tmpCamParams->M)) return false;
  if (fscanf(f, "%*s %*s %*s") == EOF) return false;
  if (!loadMat(f, tmpCamParams->D)) return false;

  //check if camera matrix needs to be scaled
  if (calibratedImgWidth != imgWidth || calibratedImgHeight != imgHeight) {
    float x_scale = static_cast<float> (imgWidth) / static_cast<float> (calibratedImgWidth);
    float y_scale = static_cast<float> (imgHeight) / static_cast<float> (calibratedImgHeight);

    tmpCamParams->M.at<float>(0,0) *= x_scale; //f_x
    tmpCamParams->M.at<float>(0,2) *= x_scale; //c_x
    tmpCamParams->M.at<float>(1,1) *= y_scale; //f_y
    tmpCamParams->M.at<float>(1,2) *= y_scale; //c_y
  }

  //load extrinsic params
  if (fscanf(f, "%*s %*s") == EOF) return false;
  if (fscanf(f, "%*s %*s %*s") == EOF) return false;
  if (!loadMat(f, tmpCamParams->tVec)) return false;
  if (fscanf(f, "%*s %*s %*s") == EOF) return false;
  if (!loadMat(f, tmpCamParams->rVec)) return false;

  fclose(f);

  //if load successful -> replace params

  //writer lock
  boost::unique_lock<boost::shared_mutex> lock(cameras_mutex);

  //reset camera params in case some other thread is using them
  camParams[cameraIndex] = tmpCamParams;

  //TODO: load/save the values generated in this method?
  computeAdditionaIntrinsiclValues(camParams[cameraIndex], cameraIndex);

  camParams[cameraIndex]->intrinsicSet = true;
  camParams[cameraIndex]->extrinsicSet = true;

  return true;
}
//********************* END single camera parameter methods ************************

//********************* stereo parameter methods *******************************

bool Cameras::calibrateStereoCamerasStart(const int cornersX, const int cornersY, const double gridSize) {
  if (!camParams[0]->intrinsicSet && !camParams[1]->intrinsicSet) {
    LOG4CXX_ERROR(logger, "[calibrateStereoCamerasStart] Failed to perform stereo calibration. Be sure intrinsic camera calibration has been done!");
    return false;
  }

  return calibrationStart(STEREO, cornersX, cornersY, gridSize); //num of inner corners on checkerboard
}

bool Cameras::calibrateStereoCamerasEnd() {
  if (!calibrationStarted || STEREO != calibmode) return false;

  // HARVEST CHESSBOARD 3D OBJECT POINT LIST:
  objectPoints.resize(sampleCount);
  for (int k = 0; k < sampleCount; k++) {
    std::vector<cv::Point3d> patternPoints;
    for (int i = 0; i < cornersY; i++) {
      for (int j = 0; j < cornersX; j++) {
        patternPoints.push_back(cv::Point3d(i, j, 0));
      }
    }
    objectPoints.push_back(patternPoints);
  }

  //CALIBRATE THE STEREO CAMERAS

  //NOTE: EAK: both camera's intrinsic parameters must have been found to use this stereo calibration method

  //rotation (from one cam to other), translation (from one cam to other), essential, fundamental matrices.
  cv::Mat _E(3, 3, CV_64F), _F(3, 3, CV_64F);

  //writer lock
  boost::unique_lock<boost::shared_mutex> lock(cameras_mutex);

  //get img dimensions
  int imgWidth1 = camParams[0]->width;
  int imgHeight1 = camParams[0]->height;

  int imgWidth2 = camParams[1]->width;
  int imgHeight2 = camParams[1]->height;

  //reset stereo params in case some other thread is using them
  stereoParams[0].reset(new StereoCameraParameters(imgWidth1, imgHeight1));
  stereoParams[1].reset(new StereoCameraParameters(imgWidth2, imgHeight2));

  //cout << "using stereo calibrated" << endl;
  double stereoError = cv::stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
          camParams[0]->M, camParams[0]->D, camParams[1]->M, camParams[1]->D,
          cv::Size(imgWidth1, imgHeight1),
          stereoParams[0]->R, stereoParams[0]->T, _E, _F);
          //,cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5)); // API change in opencv >= 3 changed order of args
  //,
  //       CV_CALIB_FIX_INTRINSIC
  //      ); //CV_CALIB_FIX_INTRINSIC ensures M and D aren't modified. if this is removed, camParams should be reset above (but still keeping existing params)

  printf("Stereo calibration finished with error: %f\n", stereoError);

  //should undistort imagepoints here? or before cvStereoCalibrate?
  //Always work in undistorted space
  //cvUndistortPoints(&_imagePoints1, &_imagePoints1, camParams[0]->M, _D1, 0, camParams[0]->M);
  //cvUndistortPoints(&_imagePoints2, &_imagePoints2, _M2, _D2, 0, _M2);

  finishStereoCalculations();

  stereoParams[0]->set = true;
  stereoParams[1]->set = true;
  calibrationStarted = false;

  return true;

  //    } else {
  //        //NOTE: EAK this stereo calibration method does NOT calculate the necessary data to calculate depth information
  //        //from disparityh data (ie. no Q matrix is generated). This method is used if the intrinsic camera parameters have
  //        //not been found for each camera
  //
  //        cout << "using stereo uncalibrated " << endl;
  //
  //        cvSetIdentity(camParams[0]->M);
  //        cvSetIdentity(camParams[1]->M);
  //        cvZero(camParams[0]->D);
  //        cvZero(camParams[1]->D);
  //
  //        cvStereoCalibrate(&_objectPoints, &_imagePoints1,
  //                &_imagePoints2, &_npoints,
  //                camParams[0]->M, camParams[0]->D, camParams[1]->M, camParams[1]->D,
  //                imageSize, _R, _T, _E, _F,
  //                cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5),
  //                CV_CALIB_FIX_ASPECT_RATIO + CV_CALIB_ZERO_TANGENT_DIST + CV_CALIB_SAME_FOCAL_LENGTH
  //                );
  //
  //
  //        //Always work in undistorted space
  //        cvUndistortPoints(&_imagePoints1, &_imagePoints1, camParams[0]->M, camParams[0]->D, 0, camParams[0]->M);
  //        cvUndistortPoints(&_imagePoints2, &_imagePoints2, camParams[1]->M, camParams[1]->D, 0, camParams[1]->M);
  //
  //        //COMPUTE AND DISPLAY RECTIFICATION
  //
  //        //HARTLEY'S RECTIFICATION METHOD
  //        double H1[3][3], H2[3][3], iM[3][3];
  //        CvMat _H1 = cvMat(3, 3, CV_64F, H1);
  //        CvMat _H2 = cvMat(3, 3, CV_64F, H2);
  //        CvMat _iM = cvMat(3, 3, CV_64F, iM);
  //
  //        cvStereoRectifyUncalibrated(
  //                &_imagePoints1, &_imagePoints2, _F,
  //                imageSize,
  //                &_H1, &_H2, 3
  //                );
  //        cvInvert(camParams[0]->M, &_iM);
  //        cvMatMul(&_H1, camParams[0]->M, camParams[0]->R);
  //        cvMatMul(&_iM, camParams[0]->R, camParams[0]->R);
  //        cvInvert(camParams[1]->M, &_iM);
  //        cvMatMul(&_H2, camParams[1]->M, camParams[1]->R);
  //        cvMatMul(&_iM, camParams[1]->R, camParams[1]->R);
  //
  //
  //        //Precompute map for cvRemap()
  //        cvInitUndistortRectifyMap(camParams[0]->M, camParams[0]->D, camParams[0]->R, camParams[0]->M, camParams[0]->mx, camParams[0]->my);
  //        cvInitUndistortRectifyMap(camParams[1]->M, camParams[1]->D, camParams[1]->R, camParams[1]->M, camParams[1]->mx, camParams[1]->my);
  //
  //        calibrationStarted = false;
  //        calibrationDone = true;
  //        intrinsicCalculated[0] = intrinsicCalculated[1] = true;
  //
  //        return true;
  //    }
}

bool Cameras::calibrationStereoSave(const char* filename) const {
  FILE* f = fopen(filename, "wt");
  if (!f) return false;

  //reader lock
  boost::shared_lock<boost::shared_mutex> lock(cameras_mutex);

  //version number
  fprintf(f, "Version %s\n\n", stereoVersion.c_str());

  //image size
  //TODO: this is a bit strange since it appears OpenCV requires both images 
  //to be the same size, not sure how to handle currently
  int imgWidth1 = camParams[0]->width;
  int imgHeight1 = camParams[0]->height;
  int imgWidth2 = camParams[1]->width;
  int imgHeight2 = camParams[1]->height;

  // CAM1 PARAMS

  fprintf(f, "CAMERA ONE\n");

  fprintf(f, "Image Size =\n");
  fprintf(f, "%d %d\n\n", imgWidth1, imgHeight1);

  fprintf(f, "Intrinsic Parameters:\n");
  fprintf(f, "Camera matrix =\n");
  if (!writeMat(f, camParams[0]->M)) return false;
  fprintf(f, "Distortion params =\n");
  if (!writeMat(f, camParams[0]->D)) return false;

  fprintf(f, "Extrinsic Parameters:\n");
  fprintf(f, "Translation params =\n");
  if (!writeMat(f, camParams[0]->tVec)) return false;
  fprintf(f, "Rotation params =\n");
  if (!writeMat(f, camParams[0]->rVec)) return false;

  // CAM2 PARAMS

  fprintf(f, "CAMERA TWO\n");

  fprintf(f, "Image Size =\n");
  fprintf(f, "%d %d\n\n", imgWidth2, imgHeight2);

  fprintf(f, "Intrinsic Parameters:\n");
  fprintf(f, "Camera matrix =\n");
  if (!writeMat(f, camParams[1]->M)) return false;
  fprintf(f, "Distortion params =\n");
  if (!writeMat(f, camParams[1]->D)) return false;

  fprintf(f, "Extrinsic Parameters:\n");
  fprintf(f, "Translation params =\n");
  if (!writeMat(f, camParams[1]->tVec)) return false;
  fprintf(f, "Rotation params =\n");
  if (!writeMat(f, camParams[1]->rVec)) return false;

  // STEREO PARAMS
  fprintf(f, "STEREO DATA (from cam1 to cam2)\n");
  fprintf(f, "Rotation Matrix = \n");
  if (!writeMat(f, stereoParams[0]->R)) return false;
  fprintf(f, "Translation Vector =\n");
  if (!writeMat(f, stereoParams[0]->T)) return false;

  fclose(f);
  return true;
}

bool Cameras::calibrationStereoLoad(const char* filename) {
  LOG4CXX_DEBUG(logger, "[calibrationStereoLoad] calibration file: " + std::string(filename));

  FILE* f = fopen(filename, "rb");
  if (!f) {
    LOG4CXX_ERROR(logger, "[calibrationStereoLoad] file not found: " + std::string(filename));
    return false;
  }

  int imgWidth1 = camParams[0]->width;
  int imgHeight1 = camParams[0]->height;
  int imgWidth2 = camParams[1]->width;
  int imgHeight2 = camParams[1]->height;

  //temp params in case load fails
  CameraParameters::Ptr tmpCamParams[2];
  tmpCamParams[0] = CameraParameters::Ptr(new CameraParameters(imgWidth1, imgHeight1));
  tmpCamParams[1] = CameraParameters::Ptr(new CameraParameters(imgWidth2, imgHeight2));
  StereoCameraParameters::Ptr tmpStereoParams[2];
  tmpStereoParams[0] = StereoCameraParameters::Ptr(new StereoCameraParameters(imgWidth1, imgHeight1));
  tmpStereoParams[1] = StereoCameraParameters::Ptr(new StereoCameraParameters(imgWidth2, imgHeight2));

  //version number
  char tmp[1000];
  if (fscanf(f, "%*s %s", tmp) == EOF) {
    LOG4CXX_ERROR(logger, "[calibrationStereoLoad] failed to parse version: " + std::string(filename));
    return false;
  }
  std::string version = tmp;
  if (version.compare(stereoVersion) != 0) {
    LOG4CXX_ERROR(logger, boost::format("[calibrationStereoLoad] trying to load stereo calibration file with wrong version number: ") % filename);
    return false;
  }

  // CAM1 PARAMS
  if (fscanf(f, "%*s %*s") == EOF) return false;

  // image size
  int tmpImgWidth1 = 1;
  int tmpImgHeight1 = 1;
  if (fscanf(f, "%*s %*s %*s") == EOF) return false;
  if (fscanf(f, "%d %d", &tmpImgWidth1, &tmpImgHeight1) != 2) return false;

  if (fscanf(f, "%*s %*s") == EOF) return false;
  if (fscanf(f, "%*s %*s %*s") == EOF) return false;
  if (!loadMat(f, tmpCamParams[0]->M)) return false;
  if (fscanf(f, "%*s %*s %*s") == EOF) return false;
  if (!loadMat(f, tmpCamParams[0]->D)) return false;

  if (fscanf(f, "%*s %*s") == EOF) return false;
  if (fscanf(f, "%*s %*s %*s") == EOF) return false;
  if (!loadMat(f, tmpCamParams[0]->tVec)) return false;
  if (fscanf(f, "%*s %*s %*s") == EOF) return false;
  if (!loadMat(f, tmpCamParams[0]->rVec)) return false;

  // CAM2 PARAMS
  if (fscanf(f, "%*s %*s") == EOF) return false;

  // image size
  int tmpImgWidth2 = 1;
  int tmpImgHeight2 = 1;
  if (fscanf(f, "%*s %*s %*s") == EOF) return false;
  if (fscanf(f, "%d %d", &tmpImgWidth2, &tmpImgHeight2) != 2) return false;

  if (fscanf(f, "%*s %*s") == EOF) return false;
  if (fscanf(f, "%*s %*s %*s") == EOF) return false;
  if (!loadMat(f, tmpCamParams[1]->M)) return false;
  if (fscanf(f, "%*s %*s %*s") == EOF) return false;
  if (!loadMat(f, tmpCamParams[1]->D)) return false;

  if (fscanf(f, "%*s %*s") == EOF) return false;
  if (fscanf(f, "%*s %*s %*s") == EOF) return false;
  if (!loadMat(f, tmpCamParams[1]->tVec)) return false;
  if (fscanf(f, "%*s %*s %*s") == EOF) return false;
  if (!loadMat(f, tmpCamParams[1]->rVec)) return false;

  // STEREO PARAMS
  if (fscanf(f, "%*s %*s %*s %*s %*s %*s") == EOF) return false;
  if (fscanf(f, "%*s %*s %*s") == EOF) return false;
  if (!loadMat(f, tmpStereoParams[0]->R)) return false;
  if (fscanf(f, "%*s %*s %*s") == EOF) return false;
  if (!loadMat(f, tmpStereoParams[0]->T)) return false;

  LOG4CXX_INFO(logger, "[calibrationStereoLoad] successfully parsed: " + std::string(filename));

  //check if camera matrices need to be scaled
  if (tmpImgWidth1 != imgWidth1 || tmpImgHeight1 != imgHeight1) {
    float x_scale = static_cast<float> (imgWidth1) / static_cast<float> (tmpImgWidth1);
    float y_scale = static_cast<float> (imgHeight1) / static_cast<float> (tmpImgHeight1);

    tmpCamParams[0]->M.at<float>(0,0) *= x_scale; //f_x
    tmpCamParams[0]->M.at<float>(0,2) *= x_scale; //c_x
    tmpCamParams[0]->M.at<float>(1,1) *= y_scale; //f_y
    tmpCamParams[0]->M.at<float>(1,2) *= y_scale; //c_y
  }

  if (tmpImgWidth2 != imgWidth2 || tmpImgHeight2 != imgHeight2) {
    float x_scale = static_cast<float> (imgWidth2) / static_cast<float> (tmpImgWidth2);
    float y_scale = static_cast<float> (imgHeight2) / static_cast<float> (tmpImgHeight2);

    tmpCamParams[1]->M.at<float>(0,0) *= x_scale; //f_x
    tmpCamParams[1]->M.at<float>(0,2) *= x_scale; //c_x
    tmpCamParams[1]->M.at<float>(1,1) *= y_scale; //f_y
    tmpCamParams[1]->M.at<float>(1,2) *= y_scale; //c_y
  }

  fclose(f);

  //if load was successful, replace old values

  //writer lock
  boost::unique_lock<boost::shared_mutex> lock(cameras_mutex);

  //reset cam params in case some other thread is using them
  camParams[0] = tmpCamParams[0];
  camParams[1] = tmpCamParams[1];

  //REMOVE
  //printf("M(%d): \n", 0);
  //for (int r = 0; r < 3; ++r) {
  //  for (int c = 0; c < 3; ++c) {
  //    std::cout << cvmGet(camParams[0]->M, r, c) << " ";
  //  }
  //  std::cout << std::endl;
  //}
  //printf("M(%d): \n", 1);
  //for (int r = 0; r < 3; ++r) {
  //  for (int c = 0; c < 3; ++c) {
  //    std::cout << cvmGet(camParams[1]->M, r, c) << " ";
  //  }
  //  std::cout << std::endl;
  //}
  //printf("R:\n");
  //for (int r = 0; r < 3; ++r) {
  //  for (int c = 0; c < 3; ++c) {
  //    std::cout << cvmGet(R, r, c) << " ";
  //  }
  //  std::cout << std::endl;
  //}
  //END REMOVE

  computeAdditionaIntrinsiclValues(camParams[0], 0);
  computeAdditionaIntrinsiclValues(camParams[1], 1);

  //reset stereo params in case some other thread is using them
  stereoParams[0] = tmpStereoParams[0];
  stereoParams[1] = tmpStereoParams[1];

  //set rot and trans of cam2's stereoParams (i.e., from cam2 to cam1)
  cv::invert(stereoParams[0]->R, stereoParams[1]->R);
  stereoParams[1]->T.at<float>(0, 0) = -stereoParams[0]->T.at<float>(0, 0);
  stereoParams[1]->T.at<float>(1, 0) = -stereoParams[0]->T.at<float>(1, 0);
  stereoParams[1]->T.at<float>(2, 0) = -stereoParams[0]->T.at<float>(2, 0);

  finishStereoCalculations();

  camParams[0]->intrinsicSet = true;
  camParams[1]->intrinsicSet = true;
  camParams[0]->extrinsicSet = true;
  camParams[1]->extrinsicSet = true;
  stereoParams[0]->set = true;
  stereoParams[1]->set = true;

  return true;
}

void Cameras::finishStereoCalculations() {
  //TODO: this is a bit strange since it appears OpenCV requires both images 
  //to be the same size, not sure how to handle currently
  int imgWidth1 = camParams[0]->width;
  int imgHeight1 = camParams[0]->height;

  cv::stereoRectify(camParams[0]->M, camParams[0]->D, camParams[1]->M, camParams[1]->D, cv::Size(imgWidth1, imgHeight1),
          stereoParams[0]->R, stereoParams[0]->T, stereoParams[0]->Rec, stereoParams[1]->Rec, stereoParams[0]->P, stereoParams[1]->P, stereoParams[0]->Q);

  //copy duplicate data into other stereo structure
  stereoParams[0]->Q.copyTo(stereoParams[1]->Q);

  //Precompute map for cv::remap()
  cv::initUndistortRectifyMap(camParams[0]->M, camParams[0]->D, stereoParams[0]->Rec, stereoParams[0]->P,
                              cv::Size(camParams[0]->width, camParams[0]->height), CV_32FC1,
                              stereoParams[0]->mx, stereoParams[0]->my);
  cv::initUndistortRectifyMap(camParams[1]->M, camParams[1]->D, stereoParams[1]->Rec, stereoParams[1]->P,
                              cv::Size(camParams[1]->width, camParams[1]->height), CV_32FC1,
                              stereoParams[1]->mx, stereoParams[1]->my);

  //set matrices to project 3D depth data onto captured (unrectified and distorted) image planes
  set3dProjectionMatrices();
}

//set matrices to project 3D depth data onto captured (unrectified and distorted) image planes

void Cameras::set3dProjectionMatrices() {

  //Rvec for cam1
  cv::Mat R1_inv(3, 3, CV_64F);
  cv::invert(stereoParams[0]->Rec, R1_inv);
  cv::Rodrigues(R1_inv, stereoParams[0]->Rinv);

  //Rvec for cam2
  cv::Mat R2_inv(3, 3, CV_64F);
  cv::invert(stereoParams[1]->Rec, R2_inv);
  cv::Rodrigues(R2_inv, stereoParams[1]->Rinv);

  //Tvec for cam 1
  stereoParams[0]->Tinv = cv::Mat::zeros(3, 1, CV_64F);

  //Tvec for cam 2
  stereoParams[1]->Tinv = cv::Mat::zeros(3, 1, CV_64F);
  stereoParams[1]->Tinv.at<float>(0) = -stereoParams[1]->P.at<float>(0, 3) / stereoParams[1]->P.at<float>(0, 0);
}
//********************* END stereo parameter methods ***************************

//********************* helper parameter methods *******************************

bool Cameras::takeSnapshot() {
  if (!calibrationStarted) return false;

  bool result;
  try {
    //get frames
    CaptureNotification::ConstPtr captureNotification = ade::capture::Capture::getLastCaptureNotification();
    const cv::Mat frame1 = captureNotification->captureData->frame.clone();
    const cv::Mat frame2 = captureNotification->captureData->frame2.clone();

    cv::Mat currGrayFrame1(frame1.rows, frame1.cols, CV_8U, 1);
    cv::Mat currGrayFrame2;
    if (STEREO == calibmode) {
      currGrayFrame2 = cv::Mat(frame2.rows, frame2.cols, CV_8U, 1);
    }

    //if color image, convert to grayscale
    if (frame1.channels() == 3) {
      cv::cvtColor(frame1, currGrayFrame1, cv::COLOR_BGR2GRAY);
    } else {
      frame1.copyTo(currGrayFrame1);
    }
    if (STEREO == calibmode) {
      if (frame2.channels() == 3) {
        cv::cvtColor(frame1, currGrayFrame2, cv::COLOR_BGR2GRAY);
      } else {
        frame2.copyTo(currGrayFrame2);
      }
    }

    //add frame(s)
    result = calibrationAddSample(currGrayFrame1, currGrayFrame2);

    //} catch (cv::Exception e) {
    //cout << "OpenCV exception in calibrateSingleCamera: " << e.what() << endl;
    //  return false;
  } catch (...) {
    printf("Exception caught in calibrateSingleCamera: Catch All\n");
    return false;
  }

  return result;
}

bool Cameras::takeSnapshots(const int numSnapshots, const int numAllowedFailures) {
  if (!calibrationStarted) return false;

  try {
    int successCount = 0;
    int failCount = 0;
    while (calibrationStarted && successCount < numSnapshots) {
      bool result = takeSnapshot();
      if (!result) {
        ++failCount;
        printf("Failed to find corners: %d fails.\n", failCount);
        if (numAllowedFailures != -1 && failCount >= 10) {
          printf("[CameraCalibration] Failed %d times.  Giving up and exiting calibration.\n", failCount);
          return false;
        }
      } else {
        ++successCount;
        printf("[CameraCalibration] Processed %d frames successfully.\n", successCount);
      }

      //wait two second between snapshots
      sleep(2);
    }

    //} catch (cv::Exception e) {
    //cout << "OpenCV exception in calibrateSingleCamera: " << e.what() << endl;
    //  return false;
  } catch (...) {
    printf("[CameraCalibration] Catch All\n");
    return false;
  }

  return true;
}

bool Cameras::calibrationStart(const enum CALIB_MODE calibmode, const int cornersX, const int cornersY, const double gridSize, const int cameraIndex) {
  if (calibrationStarted) return false;

  this->calibmode = calibmode;
  this->camIndex = cameraIndex;
  this->gridSize = gridSize;

  this->cornersX = cornersX;
  this->cornersY = cornersY;
  this->cornersN = cornersX*cornersY;
  imgPointsTemp[0].resize(cornersN);
  imgPointsTemp[1].resize(cornersN);
  sampleCount = 0;
  calibrationStarted = true;

  return true;
}

bool Cameras::runCalibration() {
  if (!calibrationStarted) return false;

  if (SINGLE_INTR == calibmode) {
    return calibrateIntrSingleCameraEnd();
  } else if (SINGLE_EXTR == calibmode) {
    return calibrateExtrSingleCameraEnd();
  } else if (STEREO == calibmode) {
    return calibrateStereoCamerasEnd();
  }

  return false;
}

void Cameras::cancelCalibration() {
  imgPointsTemp[0].resize(cornersN);
  imgPointsTemp[1].resize(cornersN);
  sampleCount = 0;
  calibrationStarted = false;
}

bool Cameras::calibrationAddSample(cv::Mat imageLeft, cv::Mat imageRight) {

  if (!calibrationStarted) return false;

  int numToProc = (!imageRight.empty()) ? 2 : 1;

  cv::Mat image[2] = {imageLeft, imageRight};

  int successes = 0;

  for (int lr = 0; lr < numToProc; lr++) {
    //        CvSize imageSize = cvGetSize(image[lr]);
    //        if (imageSize.width != VisionConstants::imgWidth || imageSize.height != VisionConstants::imgHeight)
    //            return false;

    //FIND CHESSBOARDS AND CORNERS THEREIN:
    int result = cv::findChessboardCorners(
            image[lr], cv::Size(cornersX, cornersY),
            imgPointsTemp[lr],
            cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
            );

    if (result && imgPointsTemp[lr].size() == cornersN) {

      //Calibration will suffer without subpixel interpolation
      int windowSize = std::sqrt(std::pow(image[lr].cols, 2) + std::pow(image[lr].rows, 2)) / 50;
      printf("[calibrationAddSample] windowsize: %d\n", windowSize);
      cv::find4QuadCornerSubpix(
              image[lr], imgPointsTemp[lr],
              cv::Size(windowSize, windowSize)
              );
      successes++;

      // display detected corners
      std::string windowName = "Calibration";
      ade::Display::createWindowIfDoesNotExist(windowName);
      for (int i = 0; i < imgPointsTemp[lr].size(); ++i) {
        cv::circle(image[lr], cv::Point(imgPointsTemp[lr][i].x, imgPointsTemp[lr][i].y),
                2, CV_RGB(255, 255, 255), 1, 8, 0);
      }
      ade::Display::displayFrame(image[lr], windowName);
    }
  }

  // add corners from image to collection of corners for all images
  if (numToProc == successes) {
    for (int lr = 0; lr < numToProc; lr++) {
      imagePoints[lr].push_back(imgPointsTemp[lr]);
    }
    sampleCount++;
    return true;
  } else {
    return false;
  }
}
//********************* END helper parameter methods ***************************
