/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "GraspInterface.hpp"


using namespace ade::stm;

log4cxx::LoggerPtr GraspInterface::logger = log4cxx::Logger::getLogger("ade.stm.GraspInterface");

GraspInterface::GraspInterface() {
}

void GraspInterface::initialize(JNIEnv* newEnv, jobject obj) {
  initialize(newEnv);
  j_object = obj;
}

void GraspInterface::initialize(JNIEnv* newEnv) {

  env = newEnv;
  j_class = env->FindClass("com/mega/Grasp");
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

  jmethod_setType = env->GetMethodID(j_class, "setType", "(I)V");
  if (jmethod_setType == NULL)
    LOG4CXX_ERROR(logger, "[GraspInterface] setType method not found.");

  jmethod_setPoint = env->GetMethodID(j_class, "setPoint", "(IDDD)V");
  if (jmethod_setPoint == NULL)
    LOG4CXX_ERROR(logger, "[GraspInterface] setPoint method not found.");

  jmethod_setOrientation = env->GetMethodID(j_class, "setOrientation", "(IDDDD)V");
  if (jmethod_setOrientation == NULL)
    LOG4CXX_ERROR(logger, "[GraspInterface] setOrientation method not found.");
}

jobject GraspInterface::getJavaObject() {
  return j_object;
}

void GraspInterface::setType(ade::stm::Grasp::Type type) {
  //explicit cast from c++ enum -> int
  env->CallVoidMethod(j_object, jmethod_setType, (jint) static_cast<int>(type));
}

void GraspInterface::setPoint(int i, double x, double y, double z) {
  env->CallVoidMethod(j_object, jmethod_setPoint, (jint) i, (jdouble) x, (jdouble) y, (jdouble) z);
}

void GraspInterface::setOrientation(int i, double x, double y, double z, double w) {
  env->CallVoidMethod(j_object, jmethod_setOrientation, (jint) i, (jdouble) x, (jdouble) y, (jdouble) z, (jdouble) w);
}
