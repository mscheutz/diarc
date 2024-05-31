/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   GraspsJNI.hpp
 * Author: evankrause
 */

#ifndef GRASPSJNI_HPP
#define	GRASPSJNI_HPP

#include <jni.h>

#ifdef __cplusplus
extern "C" {
#endif


/////////////////////////////////////////////////////////////////////////////////////
// Native Grasps -> Java Grasps ///////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
JNIEXPORT jobject JNICALL Java_edu_tufts_hrilab_vision_grasp_swig_GraspsJNI_calculateGraspPoses
(JNIEnv* env, jclass cls, jobject mo);

#ifdef __cplusplus
}
#endif

#endif //GRASPSJNI_HPP
