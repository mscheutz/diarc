/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "GraspInterface.hpp"

log4cxx::LoggerPtr GraspInterface::logger = log4cxx::Logger::getLogger("diarc.stm.GraspInterface");

GraspInterface::GraspInterface() {
}

void GraspInterface::initialize(JNIEnv* newEnv, jobject obj) {
  initialize(newEnv);
  j_object = obj;
}

void GraspInterface::initialize(JNIEnv* newEnv) {

  env = newEnv;
  j_class = env->FindClass("edu/tufts/hrilab/vision/stm/Grasp");
  if (j_class == NULL) {
    LOG4CXX_ERROR(logger, "[GraspInterface] class not found.");
    return;
  }

  j_constructor = env->GetMethodID(j_class, "<init>", "()V");
  if (j_constructor == NULL) {
    LOG4CXX_ERROR(logger, "[GraspInterface] default constructor not found.");
    return;
  }

  j_object = env->NewObject(j_class, j_constructor, NULL);

  jmethod_setPoint = env->GetMethodID(j_class, "setPoint", "(IDDD)V");
  if (jmethod_setPoint == NULL)
    LOG4CXX_ERROR(logger, "[GraspInterface] setPoint method not found.");

  jmethod_setOrientation = env->GetMethodID(j_class, "setOrientation", "(DDDD)V");
  if (jmethod_setOrientation == NULL)
    LOG4CXX_ERROR(logger, "[GraspInterface] setOrientation method not found.");
}

jobject GraspInterface::getJavaObject() {
  return j_object;
}

void GraspInterface::setPoint(int i, double x, double y, double z) {
  env->CallVoidMethod(j_object, jmethod_setPoint, (jint) i, (jdouble) x, (jdouble) y, (jdouble) z);
}

void GraspInterface::setOrientation(double x, double y, double z, double w) {
  env->CallVoidMethod(j_object, jmethod_setOrientation, (jdouble) x, (jdouble) y, (jdouble) z, (jdouble) w);
}
