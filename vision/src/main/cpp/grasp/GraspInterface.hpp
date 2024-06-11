/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   GraspInterface.hpp
 * Author: evan
 *
 * Created on July 22, 2015, 10:09 PM
 */

#ifndef GRASPINTERFACE_HPP
#define	GRASPINTERFACE_HPP

#include <jni.h>
#include <stdio.h>
#include <map>
#include <unistd.h>
#include <log4cxx/logger.h>
#include "Grasp.hpp"

class GraspInterface {
public:
  GraspInterface();
  void initialize(JNIEnv* newEnv, jobject vec);
  void initialize(JNIEnv* newEnv);
  jobject getJavaObject();

  void setPoint(int i, double x, double y, double z);
  void setOrientation(double x, double y, double z, double w);

private:
  JNIEnv *env;
  jobject j_object;
  jclass j_class;
  jmethodID j_constructor;

  jmethodID jmethod_setPoint;
  jmethodID jmethod_setOrientation;

  static log4cxx::LoggerPtr logger;
};

#endif	/* GRASPINTERFACE_HPP */

