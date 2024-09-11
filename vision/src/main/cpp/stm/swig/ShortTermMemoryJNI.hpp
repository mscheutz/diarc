/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   ShortTermMemoryJNI.hpp
 * Author: evankrause
 *
 * Created on May 11, 2012, 4:17 PM
 */

#ifndef SHORTTERMMEMORYJNI_HPP
#define	SHORTTERMMEMORYJNI_HPP

#include <jni.h>

#ifdef __cplusplus
extern "C" {
#endif


/////////////////////////////////////////////////////////////////////////////////////
// Native Tracked Objects -> Java STM methods ///////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
JNIEXPORT jobject JNICALL Java_edu_tufts_hrilab_vision_stm_swig_ShortTermMemoryModuleJNI_getMemoryObjects
(JNIEnv* env, jclass cls);

JNIEXPORT jobject JNICALL Java_edu_tufts_hrilab_vision_stm_swig_ShortTermMemoryModuleJNI_getMemoryObjectsByTypeId
(JNIEnv* env, jclass cls, jlong typeId);

JNIEXPORT jlongArray JNICALL Java_edu_tufts_hrilab_vision_stm_swig_ShortTermMemoryModuleJNI_getMemoryObjectTypeIds
(JNIEnv* env, jclass cls);

JNIEXPORT jlongArray JNICALL Java_edu_tufts_hrilab_vision_stm_swig_ShortTermMemoryModuleJNI_getMemoryObjectIds
(JNIEnv* env, jclass cls, jlong typeId);

JNIEXPORT jobject JNICALL Java_edu_tufts_hrilab_vision_stm_swig_ShortTermMemoryModuleJNI_getMemoryObject
(JNIEnv* env, jclass cls, jlong tokenId);

JNIEXPORT void JNICALL Java_edu_tufts_hrilab_vision_stm_swig_ShortTermMemoryModuleJNI_clearMemory
(JNIEnv* env, jclass cls);

JNIEXPORT jobject JNICALL Java_edu_tufts_hrilab_vision_stm_swig_ShortTermMemoryModuleJNI_getRemovedMemoryObjects
(JNIEnv* env, jclass cls);

#ifdef __cplusplus
}
#endif

#endif //SHORTTERMMEMORYJNI_HPP
