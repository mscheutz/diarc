/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include <opencv2/opencv.hpp>

#include "CaptureJNI.hpp"
#include "capture/Capture.hpp"
#include "capture/util/CaptureUtilities.hpp"

JNIEXPORT jboolean JNICALL Java_edu_tufts_hrilab_vision_capture_swig_CaptureModuleJNI_checkDarkness(JNIEnv *env, jclass jcls) {
    const cv::Mat frame = diarc::capture::Capture::getLastCaptureNotification()->captureData->frame;
    return diarc::capture::util::isFrameDark(frame);
}


JNIEXPORT void JNICALL Java_edu_tufts_hrilab_vision_capture_swig_CaptureModuleJNI_passBackImageArray(JNIEnv* env, jclass jcls, jbyteArray passedArray, jint blurAmount) {
  const cv::Mat frame = diarc::capture::Capture::getLastCaptureNotification()->captureData->frame;

  cv::Mat blurFrame;
  if (blurAmount > 0) {
    cv::GaussianBlur(frame, blurFrame,
                     cv::Size(blurAmount, blurAmount),
                     0);
  } else {
    blurFrame = frame;
  }

  unsigned char* dataLocal = (unsigned char*) env->GetPrimitiveArrayCritical(passedArray, 0);

  for (int i = 0; i < blurFrame.rows * blurFrame.cols * 3; i = i + 3) {
    dataLocal[i] = (unsigned char) blurFrame.data[i + 2];
    dataLocal[i + 1] = (unsigned char) blurFrame.data[i + 1];
    dataLocal[i + 2] = (unsigned char) blurFrame.data[i];
    //        dataLocal[i] = (unsigned char) 0;
    //        dataLocal[i + 1] = (unsigned char) 0;
    //        dataLocal[i + 2] = (unsigned char) 255;
  }

  // must release array!
  env->ReleasePrimitiveArrayCritical(passedArray, dataLocal, 0);
}

JNIEXPORT void JNICALL Java_edu_tufts_hrilab_vision_capture_swig_CaptureModuleJNI_passBackDisparityArray(JNIEnv* env, jclass jcls, jbyteArray passedArray) {
  const cv::Mat frame = diarc::capture::Capture::getLastCaptureNotification()->captureData->depthFrameIntensity;
  float* dataLocal = (float*) env->GetPrimitiveArrayCritical(passedArray, 0);

  for (int r = 0; r < frame.rows; ++r) {
	  for (int c = 0; c < frame.cols; ++c) {
	    int i = r * frame.cols + c;
	    dataLocal[i] = frame.at<float>(r, c);
	  }
  }

  // must release array!
  env->ReleasePrimitiveArrayCritical(passedArray, dataLocal, 0);
}

JNIEXPORT void JNICALL Java_edu_tufts_hrilab_vision_capture_swig_CaptureModuleJNI_passBackDepthArray(JNIEnv* env, jclass jcls, jbyteArray passedArray) {
  const cv::Mat frame = diarc::capture::Capture::getLastCaptureNotification()->captureData->depthFrame;
  float* dataLocal = (float*) env->GetPrimitiveArrayCritical(passedArray, 0);

  for (int r = 0; r < frame.rows; ++r) {
    for (int c = 0; c < frame.cols; ++c) {
      int i = r * frame.cols + c;
      dataLocal[i] = frame.at<float>(r, c);
    }
  }

  // must release array!
  env->ReleasePrimitiveArrayCritical(passedArray, dataLocal, 0);
}
