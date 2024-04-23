/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   CaptureJNI.hpp
 * Author: evankrause
 *
 * Created on May 11, 2012, 4:17 PM
 */

#ifndef CAPTUREJNI_HPP
#define	CAPTUREJNI_HPP

#include <jni.h>

#ifdef __cplusplus
extern "C" {
#endif

JNIEXPORT jboolean JNICALL Java_com_vision_capture_swig_CaptureModuleJNI_checkDarkness
        (JNIEnv *env, jclass jcls);

JNIEXPORT void JNICALL Java_com_vision_capture_swig_CaptureModuleJNI_passBackImageArray
        (JNIEnv *env, jclass jcls, jbyteArray passedArray, jint blurAmount);

JNIEXPORT void JNICALL Java_com_vision_capture_swig_CaptureModuleJNI_passBackDisparityArray
        (JNIEnv *env, jclass jcls, jbyteArray passedArray);

JNIEXPORT void JNICALL Java_com_vision_capture_swig_CaptureModuleJNI_passBackDepthArray
        (JNIEnv *env, jclass jcls, jbyteArray passedArray);


#ifdef __cplusplus
}
#endif

#endif  /* CAPTUREJNI_HPP */

