/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   CommonJNI.hpp
 * Author: evankrause
 *
 * Created on May 10, 2012, 10:55 AM
 */

#ifndef COMMONJNI_HPP
#define	COMMONJNI_HPP


#include <jni.h>

// include log4cxx header files.
#include <log4cxx/basicconfigurator.h>
#include <log4cxx/propertyconfigurator.h>
#include <log4cxx/helpers/exception.h>

#ifdef __cplusplus
extern "C" {
#endif

// set the log4cxx logging configuration file
JNIEXPORT void JNICALL Java_edu_tufts_hrilab_vision_common_swig_CommonModuleJNI_setLoggingConfiguration(JNIEnv* env, jclass cls, jstring filename) {
  jboolean isCopy;
  const char* utf_string = env->GetStringUTFChars(filename, &isCopy);
  std::string filename_str = utf_string;
  
  // Set up a simple configuration that logs on the console.
  //log4cxx::BasicConfigurator::configure();
  log4cxx::PropertyConfigurator::configure(filename_str);
  //LOG4CXX_INFO(logger, boost::format("log4cxx configured with: %s.") % filename_str);
  
  env->ReleaseStringUTFChars(filename, utf_string);
}

//methods to check #defines from Java side

JNIEXPORT jboolean JNICALL Java_edu_tufts_hrilab_vision_common_swig_CommonModuleJNI_hasV4R(JNIEnv* env, jclass cls) {
#ifdef USE_V4R
    return true;
#else
    return false;
#endif  //USE_V4R
}

JNIEXPORT jboolean JNICALL Java_edu_tufts_hrilab_vision_common_swig_CommonModuleJNI_hasV4RV0(JNIEnv* env, jclass cls) {
#ifdef USE_V4R_V0
  return true;
#else
  return false;
#endif  //USE_V4R_V0
}

JNIEXPORT jboolean JNICALL Java_edu_tufts_hrilab_vision_common_swig_CommonModuleJNI_hasAgileGrasp(JNIEnv* env, jclass cls) {
#ifdef USE_AGILEGRASP
  return true;
#else
  return false;
#endif  //USE_AGILEGRASP
}

JNIEXPORT jboolean JNICALL Java_edu_tufts_hrilab_vision_common_swig_CommonModuleJNI_hasOpenPose(JNIEnv* env, jclass cls) {
#ifdef USE_OPENPOSE
  return true;
#else
  return false;
#endif  //USE_OPENPOSE
}

JNIEXPORT jboolean JNICALL Java_edu_tufts_hrilab_vision_common_swig_CommonModuleJNI_hasZBar(JNIEnv* env, jclass cls) {
#ifdef USE_ZBAR
  return true;
#else
  return false;
#endif  //USE_ZBAR
}

JNIEXPORT jboolean JNICALL Java_edu_tufts_hrilab_vision_common_swig_CommonModuleJNI_hasOpenCVTracking(JNIEnv* env, jclass cls) {
#ifdef OPENCV_TRACKING
  return true;
#else
  return false;
#endif  //OPENCV_TRACKING
}

#ifdef __cplusplus
}
#endif


#endif	/* COMMONJNI_HPP */

