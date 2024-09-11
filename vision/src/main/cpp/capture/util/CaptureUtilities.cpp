/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "CaptureUtilities.hpp"
#include "common/VisionConstants.hpp"
#include "display/Display.hpp"
#include <stdio.h>

namespace diarc {
  namespace capture {
    namespace util {

      CAM_MODE stringToCAM_MODE(const std::string& str) {
        if (str == "MONO_FW") return MONO_FW;
        else if (str == "STEREO_FW") return STEREO_FW;
        else if (str == "STEREO_SYNC") return STEREO_SYNC;
        else if (str == "MONO_V4L2") return MONO_V4L2;
        else if (str == "STEREO_V4L2") return STEREO_V4L2;
        else if (str == "MONO_AVI") return MONO_AVI;
        else if (str == "MONO_FILE") return MONO_FILE;
        else if (str == "MULTI_FILE_LOG") return MULTI_FILE_LOG;
        else if (str == "OPENNI") return OPENNI;
        else if (str == "OPENNI_ONI") return OPENNI_ONI;
        else if (str == "OPENNI2") return OPENNI2;
        else if (str == "KINECT") return OPENNI; // for backwards compatibility -- prefer OPENNI or OPENNI2
        else if (str == "XTION") return OPENNI; // for backwards compatibility -- prefer OPENNI or OPENNI2
        else if (str == "REALSENSE") return REALSENSE;
        else if (str == "KINECT_ONI") return OPENNI_ONI; // for backwards compatibility -- prefer OPENNI_ONI
        else if (str == "SIM_PC1") return SIM_PC1;
        else if (str == "SIM_PC2") return SIM_PC2;
        else if (str == "PCD") return SIM_PC2;
        else if (str == "KINECT_PLUS") return KINECT_PLUS;
        else if (str == "DRONE") return DRONE;
        else if (str == "ROS") return ROS;
        else if (str == "NAO") return NAO;
        else if (str == "GSTREAMER") return GSTREAMER;
        else if (str == "MULTISENSE") return MULTISENSE;
        else {printf("ERROR [CaptureUtilities]: %s is not a valid CAM_MODE.\n", str.c_str()); return NUM_MODES;}
      }

      bool isFrameDark(const cv::Mat frame, const int avgDarknessThresh) {
        int total = 0;
        int width = frame.cols;
        int height = frame.rows;
        int channels = frame.channels();

        bool verbose = false;
        if (verbose) {
          printf("image width: %d height: %d depth: %d\n", width, height, channels);
        }

        for (int x = 0; x < width; ++x) {
          for (int y = 0; y < height; ++y) {
            for (int c = 0; c < channels; ++c) {
              int loc = (x + y * width) * channels + c;
              total += (unsigned char) (frame.data[loc]);
            }
          }
        }

        int avg = total / (width * height * channels);
        if (verbose) {
          printf("total intensity: %d\n", total);
          printf("avg intensity: %d\n", avg);
        }
        return (avg < avgDarknessThresh);
      }

      void undistortFrame(const cv::Mat src, cv::Mat dst, CameraParameters::ConstPtr camParams, int cvInterpolationFlag) {
        //printf("CaptureUtilities::undistortFrame\n");
        cv::remap(src, dst, camParams->mx, camParams->my, cvInterpolationFlag);
      }

      void rectifyFrame(const cv::Mat src, cv::Mat dst, StereoCameraParameters::ConstPtr stereoParams) {
        //printf("CaptureUtilities::rectifyFrame\n");
        cv::remap(src, dst, stereoParams->mx, stereoParams->my, cv::INTER_NEAREST);
      }

      //here, left refers to the robot's left
      bool generateDepthMaps(const cv::Mat imageSrcLeft, const cv::Mat imageSrcRight,
                             cv::Mat depthMapLeft, cv::Mat depthMapRight,
              StereoCameraParameters::ConstPtr stereoParamsLeft, StereoCameraParameters::ConstPtr stereoParamsRight,
              CameraParameters::ConstPtr camParamsLeft, CameraParameters::ConstPtr camParamsRight) {

        if (!stereoParamsLeft->set || !stereoParamsRight->set) return false;
        //printf("CaptureUtilities::generateDepthMaps\n");

        //local params
        cv::Size imageSize = imageSrcLeft.size();
        static cv::Mat imagesRectified[2]; //grayscale imgs rectified by mx1, my1, mx2, my2
        static cv::Mat disparityImage, real_disparity, disparityImageNormalized; //raw disparity calc (real_disparity*16), real disparity values (after dividing by 16), disparity sclaed between [0 255]
        static cv::Mat graySclImages[2];
        static std::vector<cv::Point2d> imgPoints[2]; //for reprojecting from 3d to image plane

        //convert to grayscale
        graySclImages[0] = cv::Mat(VisionConstants::imgWidth, VisionConstants::imgHeight, CV_8U, 1);
        graySclImages[1] = cv::Mat(VisionConstants::imgWidth, VisionConstants::imgHeight, CV_8U, 1);
        if (imageSrcLeft.channels() == 1) {
            imageSrcLeft.copyTo(graySclImages[0]);
        } else {
          cv::cvtColor(imageSrcLeft, graySclImages[0], cv::COLOR_BGR2GRAY);
        }
        if (imageSrcRight.channels() == 1) {
          imageSrcRight.copyTo(graySclImages[1]);
        } else {
          cv::cvtColor(imageSrcRight, graySclImages[1], cv::COLOR_BGR2GRAY);
        }

        //undistort and rectify images
        imagesRectified[0] = cv::Mat(VisionConstants::imgHeight, VisionConstants::imgWidth, CV_8U);
        imagesRectified[1] = cv::Mat(VisionConstants::imgHeight, VisionConstants::imgWidth, CV_8U);
        cv::remap(graySclImages[0], imagesRectified[0], stereoParamsLeft->mx, stereoParamsLeft->my, cv::INTER_NEAREST);
        cv::remap(graySclImages[1], imagesRectified[1], stereoParamsRight->mx, stereoParamsRight->my, cv::INTER_NEAREST);

        try {
          if (!diarc::Display::windowExists("Rectified camera 1")) {
            diarc::Display::createWindowIfDoesNotExist("Rectified camera 1");
          }
          diarc::Display::displayFrame(imagesRectified[0], "Rectified camera 1");

          if (!diarc::Display::windowExists("Rectified camera 2")) {
            diarc::Display::createWindowIfDoesNotExist("Rectified camera 2");
          }
          diarc::Display::displayFrame(imagesRectified[1], "Rectified camera 2");
        } catch (cv::Exception e) {
          printf("[CaptureUtilities::generateDepthMaps] ERROR: OpenCV exception caught. Details: %s\n", e.what());
        }

        if (true) {
          //real-time stereo correspondance
          // Creating an object of StereoSGBM algorithm
          cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create();

          stereo->setPreFilterSize(41);
          stereo->setPreFilterCap(31);
          stereo->setSpeckleWindowSize(41);
          stereo->setMinDisparity(-64); //1;//
          stereo->setNumDisparities(128); //256;//
          stereo->setTextureThreshold(10);
          stereo->setUniquenessRatio(15);

          disparityImage = cv::Mat(VisionConstants::imgHeight, VisionConstants::imgWidth, CV_16S); //CV_32F);
          disparityImageNormalized = cv::Mat(VisionConstants::imgHeight, VisionConstants::imgWidth, CV_8U);
          stereo->compute(imagesRectified[0], imagesRectified[1], disparityImage);
          cv::normalize(disparityImage, disparityImageNormalized, 0, 256, cv::NORM_MINMAX);
        } else {
          //NON-REAL-TIME  stereo correspondance
          //experimental, not complete

          //        CvStereoGCState *GCState = cvCreateStereoGCState(128, 2);
          //            GCState->minDisparity = -64; //1;//
          //            GCState->numberOfDisparities = 128; //256;//

          //        CvMat* disparity_right = cvCreateMat(VisionConstants::imgHeight, VisionConstants::imgWidth, CV_16S);

          //        cvFindStereoCorrespondenceGC(imagesRectified[0], imagesRectified[1], disparityImage, disparity_right, GCState, 0);
          //        cvNormalize(disparityImage, disparityImageNormalized, 0, 256, CV_MINMAX);

          //        cvReleaseMat(&disparity_right);
          //        cvReleaseStereoGCState(&GCState);
        }


        //CALCULATE DEPTH MAPS FROM DISPARITY

        //get real disparity values by rescaling disparityImage
        real_disparity = cv::Mat(VisionConstants::imgHeight, VisionConstants::imgWidth, CV_16S);
        cv::convertScaleAbs(disparityImage, real_disparity, 1.0 / 16, 0);

        //calculate 3D points from disparity
        cv::Mat _3dPoints = cv::Mat(VisionConstants::imgHeight, VisionConstants::imgWidth, CV_32FC3);
        cv::reprojectImageTo3D(real_disparity, _3dPoints, stereoParamsLeft->Q);

        cv::Mat object_points(_3dPoints);
        object_points.reshape(0,1);

        int size = VisionConstants::imgWidth * VisionConstants::imgHeight;
        cv::Mat image_points1(1, size, CV_32FC2);
        cv::Mat image_points2(1, size, CV_32FC2);

        //reproject 3D points onto image planes
        cv::projectPoints(object_points, stereoParamsLeft->Rinv, stereoParamsLeft->Tinv, camParamsLeft->M, camParamsLeft->D, image_points1); //camera 1 keeper
        cv::projectPoints(object_points, stereoParamsRight->Rinv, stereoParamsRight->Tinv, camParamsRight->M, camParamsRight->D, image_points2); //camera 2 keeper

        depthMapLeft.setTo(0);
        depthMapRight.setTo(0);
        int width = VisionConstants::imgWidth;
        int x = 0;
        int y = 0;
        float z = 0;

        for (int i = 0; i < size; ++i) {
          float* object_points_data = (float*)object_points.data;
          float* image_points1_data = (float*)image_points1.data;
          if (((short*)disparityImage.data)[i] != -1040 && !std::isinf(object_points_data[(i * 3)]) && object_points_data[(i * 3)] == object_points_data[(i * 3)]) {
            //cout << image_points.data.fl[(i*2)] << " " << image_points.data.fl[(i*2)+1] << endl;
            //camera 1
            x = image_points1_data[(i * 2)];
            y = image_points1_data[(i * 2) + 1];
            z = image_points1_data[(i * 3) + 2];
            // +z is into image plane (ie larger negative z means further away from camera)
            if (z > 0) {
              z = 0;
            } else {
              z = std::abs(z) * VisionConstants::mPerSquare; //convert to positive m
            }
            //cout << "x: " << x << " y: " << y << " z: " << z << endl;
            float* data = (float*) depthMapLeft.data;
            if (x >= 0 && x < VisionConstants::imgWidth && y >= 0 && y < VisionConstants::imgHeight) {
              //depthImages[0]->imageData[y * step + x] = graySclImages[0]->imageData[y * step + x];// * static_cast<uchar>(std::abs(object_points->data.fl[(i * 3) + 2]))/255;
              data[y * width + x] = z;
            }

            //camera 2
            float* image_points2_data = (float*)image_points2.data;
            x = image_points2_data[(i * 2)];
            y = image_points2_data[(i * 2) + 1];
            //same z value as above, just different (x,y) image location
            data = (float*) depthMapRight.data;
            if (x >= 0 && x < VisionConstants::imgWidth && y >= 0 && y < VisionConstants::imgHeight) {
              //                        depthImages[1]->imageData[y * step + x] = graySclImages[1]->imageData[y * step + x];// * static_cast<uchar>(std::abs(object_points->data.fl[(i * 3) + 2]))/255;
              data[y * width + x] = z;
            }
            //cout << x << " " << y << " " << object_points->data.fl[(i * 3) + 2] << endl;;
            //cout << _3dPoints->data.fl[(i*3)+2] << endl;
          }
        }


        //TODO: does this copy date?
        //disparity = cvGetImage(disparityImageNormalized, disparity);

        return true;

      }

      double toRadians(const double& deg) {
        return (PI / 180.0) * deg;
      }

      cv::Vec3d calcDirection(const int camIndex, const int x, const int y) {
        CameraParameters::ConstPtr camParams = Cameras::getInstance()->getCameraParameters(camIndex);
        float f_x = camParams->M.at<float>(0,0); //f_x
        float c_x = camParams->M.at<float>(0,2); //c_x
        float c_y = camParams->M.at<float>(1,2); //c_y
        cv::Vec3d direction(x - c_x, y - c_y, f_x);

        // make unit vector
        double normValue = 1.0 / cv::norm(direction);
        direction[0] *= normValue;
        direction[1] *= normValue;
        direction[2] *= normValue;
        return direction;
      }

      void calcPanTilt(const int camIndex, const int x, const int y, double& pan, double& tilt) {
        CameraParameters::ConstPtr camParams = Cameras::getInstance()->getCameraParameters(camIndex);
        //printf("[calcPanTilt] index: %d, x: %d, y: %d, fovx: %f fovy: %f\n", camIndex, x, y, camParams->fovx, camParams->fovy);
        pan = (VisionConstants::imgWidth / 2.0 - x) * toRadians(camParams->fovx) / static_cast<double> (VisionConstants::imgWidth);
        tilt = (VisionConstants::imgHeight / 2.0 - y) * toRadians(camParams->fovy) / static_cast<double> (VisionConstants::imgHeight);
      }

      void calc3dToPanTiltDist(const int camIndex, const double& x, const double& y, const double& z, double& pan, double& tilt, double& dist) {
        CameraParameters::ConstPtr camParams = Cameras::getInstance()->getCameraParameters(camIndex);
        //printf("[calcPanTilt] index: %d, x: %d, y: %d, fovx: %f fovy: %f\n", camIndex, x, y, camParams->fovx, camParams->fovy);

        int xx, yy;
        projectPoint(x, y, z, xx, yy, camIndex);
        pan = (VisionConstants::imgWidth / 2.0 - xx) * toRadians(camParams->fovx) / static_cast<double> (VisionConstants::imgWidth);
        tilt = (VisionConstants::imgHeight / 2.0 - yy) * toRadians(camParams->fovy) / static_cast<double> (VisionConstants::imgHeight);
        dist = std::sqrt(x * x + y * y + z * z);
      }

      //! project 3D point (x,y,z) onto image plane (xx,yy)
      void projectPoint(const float& x, const float& y, const float& z, int& xx, int& yy, const int camIndex) {
        Cameras* cameras = Cameras::getInstance();
        CameraParameters::ConstPtr camParams = cameras->getCameraParameters(camIndex);
        StereoCameraParameters::ConstPtr stereoParams = cameras->getStereoParameters(camIndex);

        //project 3D point onto color image to get pixel location
        cv::Mat rVec = (cv::Mat_<double>(1, 3) << 0, 0, 0);
        cv::Mat rMat = stereoParams->R; //(3, 3, CV_64F);
        cv::Rodrigues(rMat, rVec);
        cv::Mat tVec = stereoParams->T; // = cv::Mat(camParams1->tVec) - cv::Mat(camParams2->tVec);

        cv::Point3f oPoint;
        oPoint.x = x;
        oPoint.y = y;
        oPoint.z = z;
        std::vector<cv::Point3f> oPoints;
        oPoints.push_back(oPoint);
        //cv::Mat objectPoints = (cv::Mat_<float>(3, 1) << x, y, z);

        std::vector<cv::Point2f> imgPoints;
        cv::projectPoints(cv::Mat(oPoints), rVec, tVec, camParams->M, camParams->D, imgPoints);

        xx = imgPoints[0].x;
        yy = imgPoints[0].y;
      }

      //! project 3D points (x,y,z) onto image plane (xx,yy)
      void projectPoints(const cv::Mat& objectPoints, std::vector<cv::Point2f>& imagePoints, const int camIndex) {
        //make sure there are actually points to project
        if (0 == objectPoints.rows || 0 == objectPoints.cols || 0 == objectPoints.channels()) {
          return;
        }

        Cameras* cameras = Cameras::getInstance();
        CameraParameters::ConstPtr camParams = cameras->getCameraParameters(camIndex);
        StereoCameraParameters::ConstPtr stereoParams = cameras->getStereoParameters(camIndex);

        //project 3D point onto color image to get pixel location
        cv::Mat rVec = (cv::Mat_<double>(1, 3) << 0, 0, 0);
        cv::Mat rMat = stereoParams->R; //(3, 3, CV_64F);
        cv::Rodrigues(rMat, rVec);
        cv::Mat tVec = stereoParams->T; // = cv::Mat(camParams1->tVec) - cv::Mat(camParams2->tVec);
        
//        printf("cont: %s depth: %d chan: %d rows: %d cols: %d\n", (objectPoints.isContinuous()?"true":"false"), objectPoints.depth(), objectPoints.channels(), objectPoints.rows, objectPoints.cols);
        cv::projectPoints(cv::Mat(objectPoints), rVec, tVec, camParams->M, camParams->D, imagePoints);
      }

      //! project 3D points (x,y,z) onto image plane (xx,yy)
      void projectPoints(const std::vector<cv::Point3f>& objectPoints, std::vector<cv::Point2f>& imagePoints, const int camIndex) {
        projectPoints(cv::Mat(objectPoints), imagePoints, camIndex);
      }

      // if camera paremeters are set for camIndex, use them, otherwise 
      // use default kinect parametrs
      void depthTo3d(const cv::Mat img_depth, cv::Mat img_3d, const int camIndex) {
        if (3 == img_depth.channels()) {
          //printf("[depthToPointCloud]: depth image incorrect nChannels.\n");
          img_depth.copyTo(img_3d);
          return;
        }

        const int imgWidth = img_depth.cols;
        const int imgHeight = img_depth.rows;

        Cameras* cameras = Cameras::getInstance();
        CameraParameters::ConstPtr camParams = cameras->getCameraParameters(camIndex);
        float fx, fy, cx, cy = 0.0;
        if (camParams->intrinsicSet) {
          //printf("[depthTo3d] using openni params!\n");
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

        float* dataDepth = (float*) img_depth.data;
        float* data3d = (float*) img_3d.data;

        for (int xx = 0; xx < imgWidth; xx++) {
          for (int yy = 0; yy < imgHeight; yy++) {
            float z_ir = dataDepth[xx + imgWidth * yy];
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
            data3d[(xx + yy * imgWidth)*3 + 0] = x_ir;
            data3d[(xx + yy * imgWidth)*3 + 1] = y_ir;
            data3d[(xx + yy * imgWidth)*3 + 2] = z_ir;
          }
        }
      }

      void convertFramesBasedOnOpenCVConstant(cv::Mat image, const int cvConstant) {
        if (image.channels() > 1)
          cv::cvtColor(image, image, cvConstant);
      }

      void convertToRGB(cv::Mat image, COLOR_SPACE currColor) {
        if (currColor == RGB)
          return; // nothing to do.  Sweet.
        else if (currColor == YCrCb)
          convertFramesBasedOnOpenCVConstant(image, cv::COLOR_YCrCb2BGR);
        else if (currColor == Luv)
          convertFramesBasedOnOpenCVConstant(image, cv::COLOR_Luv2BGR);
        else if (currColor == BGR)
          convertFramesBasedOnOpenCVConstant(image, cv::COLOR_BGR2RGB);
      }

      void convertColorsIfNecessary(cv::Mat image, COLOR_SPACE currColor, COLOR_SPACE desiredColor) {
        if (currColor != desiredColor) {
          // this means that want some different color.
          // all color conversions must take place through an RGB intermediary.
          convertToRGB(image, currColor);

          // if want rgb, convert to it from wherever it is now:
          if (desiredColor == Luv)
            convertFramesBasedOnOpenCVConstant(image, cv::COLOR_BGR2Luv);
          else if (desiredColor == YCrCb)
            convertFramesBasedOnOpenCVConstant(image, cv::COLOR_BGR2YCrCb);
          //else if (desiredColor == RGB) nothing to do
        }

        /* code for neutralizing intensity for non-RGB color space.
        /     admittedly, I (MZ) didn't notice particularly better results with it
        /     on, but it is left here for anyone who wishes to experiment with it...
            int height     = frame->height;
        int width      = frame->width;
        int step       = frame->widthStep/sizeof(uchar);
        int channels   = frame->nChannels;
        uchar* data    = (uchar *)frame->imageData;

        if (COLOR_SPACE != COLOR_SPACE_CONST_RGB)
        {
            for (int i = 0; i < WIN_SIZE_H; i++)
            {
                for (int j = 0; j < WIN_SIZE_W; j++)
                {
                    data[i*step+j*channels+0] = 255;
                }
            }
        }
         */

      }

    } //namespace util
  } //namespace capture
} //namespace diarc
