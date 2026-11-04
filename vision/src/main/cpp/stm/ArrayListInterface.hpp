/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef ARRAYLISTINTERFACE_HPP
#define ARRAYLISTINTERFACE_HPP
#include <jni.h>
#include <stdio.h>
#include <map>
#include <unistd.h>

class ArrayListInterface
{
public:
  ArrayListInterface ();
  void initialize (JNIEnv* newEnv, jobject vec);
  void initialize (JNIEnv* newEnv);
  bool add (jobject newObject);
  void clear ();
  jobject get (jint i);
  int size ();
  void ensureCapacity (int capacity);
  jobject getJavaObject ();
  
private:
  JNIEnv *env;
  jobject j_object;
  jclass j_class;
  jmethodID j_constructor;

  jmethodID jmethod_add;
  jmethodID jmethod_clear;
  jmethodID jmethod_get;
  jmethodID jmethod_size;
  jmethodID jmethod_ensureCapacity;
};

#endif
