/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "ShortTermMemoryJNI.hpp"

#include <opencv2/opencv.hpp>

#include "stm/MemoryObjectInterface.hpp"           // Interface with Java data struct
#include "stm/ArrayListInterface.hpp"                 // Interface with Java data struct.  can use for ArrayList of any java object
#include "stm/MemoryObject.hpp"
#include "common/VisionConstants.hpp"
#include "stm/TrackedObjects.hpp"

using namespace diarc::stm;

JNIEXPORT jobject JNICALL Java_edu_tufts_hrilab_vision_stm_swig_ShortTermMemoryModuleJNI_getMemoryObjects(JNIEnv* env, jclass cls, jdouble conf) {
  ArrayListInterface memoryObjects;
  memoryObjects.initialize(env);
  TrackedObjects::getInstance()->getMemoryObjects(memoryObjects, (double) conf, env);
  return memoryObjects.getJavaObject();
}

JNIEXPORT jobject JNICALL Java_edu_tufts_hrilab_vision_stm_swig_ShortTermMemoryModuleJNI_getMemoryObjectsByTypeId(JNIEnv* env, jclass cls, jlong typeId, jdouble conf) {
  ArrayListInterface memoryObjects;
  memoryObjects.initialize(env);
  TrackedObjects::getInstance()->getMemoryObjectsByTypeId(memoryObjects, (long long) typeId, (double) conf, env);
  //printf("Java_JNIversion_stm_ShortTermMemoryInterface_getMemoryObjectsByTypeId\n");
  return memoryObjects.getJavaObject();
}

JNIEXPORT jlongArray JNICALL Java_edu_tufts_hrilab_vision_stm_swig_ShortTermMemoryModuleJNI_getMemoryObjectTypeIds(JNIEnv* env, jclass cls, jdouble conf) {
  jlongArray idsArray;
  TrackedObjects::getInstance()->getMemoryObjectTypeIds(idsArray, (double) conf, env);
  return idsArray;
}

JNIEXPORT jlongArray JNICALL Java_edu_tufts_hrilab_vision_stm_swig_ShortTermMemoryModuleJNI_getMemoryObjectIds(JNIEnv* env, jclass cls, jlong typeId, jdouble conf) {
  jlongArray idsArray; // = NewLongArray();
  TrackedObjects::getInstance()->getMemoryObjectIds(idsArray, (long long) typeId, (double) conf, env);
  return idsArray;
}

JNIEXPORT jobject JNICALL Java_edu_tufts_hrilab_vision_stm_swig_ShortTermMemoryModuleJNI_getMemoryObject(JNIEnv* env, jclass cls, jlong tokenId, jdouble conf) { //return MO
  MemoryObjectInterface memoryObject;
  memoryObject.initialize(env);
  //cout << "jlong: " << tokenId << endl;
  //long nativeid = (long)tokenId;
  //cout << "long: " << nativeid << endl;
  TrackedObjects::getInstance()->getMemoryObject(memoryObject, (long long) tokenId, (double) conf, env);
  return memoryObject.getJavaObject();
}

JNIEXPORT jobject JNICALL Java_edu_tufts_hrilab_vision_stm_swig_ShortTermMemoryModuleJNI_getMemoryObjectsByTokenId(JNIEnv* env, jclass cls, jlong tokenId, jdouble conf) { //return MO
  ArrayListInterface memoryObjects;
  memoryObjects.initialize(env);
  TrackedObjects::getInstance()->getMemoryObjectsByTokenId(memoryObjects, (long long) tokenId, (double) conf, env);
  return memoryObjects.getJavaObject();
}

JNIEXPORT void JNICALL Java_edu_tufts_hrilab_vision_stm_swig_ShortTermMemoryModuleJNI_clearMemory(JNIEnv* env, jclass cls) { //return boolean
  printf("WARNING: STM clearMemory not curently implemented!\n");
  //TODO: implement this!
  //TrackedObjects::getInstance()->clearTrackedObjects();
}

JNIEXPORT jobject JNICALL Java_edu_tufts_hrilab_vision_stm_swig_ShortTermMemoryModuleJNI_getRemovedMemoryObjects(JNIEnv* env, jclass cls) {
  ArrayListInterface memoryObjects;
  memoryObjects.initialize(env);
  TrackedObjects::getInstance()->getRemovedMemoryObjects(memoryObjects, env);
  return memoryObjects.getJavaObject();
}