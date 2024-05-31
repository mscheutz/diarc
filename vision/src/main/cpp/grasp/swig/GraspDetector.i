/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/* File: GraspDetector.i */
%module GraspDetectorModule

%include "std_string.i" /* for std::string type-maps */

/* to allow use of boost shared_ptr */
// %include <boost_shared_ptr.i>
// %shared_ptr(GraspDetector)

%{
/* native includes */
  #include "GraspsJNI.hpp"
%}

//typemaps
%typemap(jstype) ArrayList "java.util.ArrayList"
%typemap(jtype) ArrayList "java.util.ArrayList"
%typemap(javaout) ArrayList { return $jnicall; }

%typemap(jstype) MemoryObject "edu.tufts.hrilab.vision.stm.MemoryObject"
%typemap(jtype) MemoryObject "edu.tufts.hrilab.vision.stm.MemoryObject"
%typemap(jni) MemoryObject "jobject" // C++ MemoryObject is treated as a jobject in the C++ part of the JNI code
%typemap(javain) MemoryObject "$javainput"

//native hand-written methods (defined in GraspsJNI.hpp)
// TODO: change this to take in just the segmented point cloud (double[][] or double[]) and transform (double[][] or double[])
%native(calculateGraspPoses) ArrayList calculateGraspPoses(MemoryObject mo);

