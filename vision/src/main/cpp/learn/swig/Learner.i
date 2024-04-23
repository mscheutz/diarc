/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/* File: Learner.i */
%module LearnerModule

/* import base class */
%import "visionproc/swig/VisionProcess.i"
%typemap(javaimports) SWIGTYPE, SWIGTYPE * "import edu.tufts.hrilab.vision.visionproc.swig.*;"

//%pragma(java) jniclassimports="import JNIversion.visionproc.swig.*;" 
//%pragma(java) moduleimports="import JNIversion.visionproc.swig.*;" 

%include "std_string.i" /* for std::string type-maps */
%include "enums.swg"	/* for proper Java enum classes */
%include <boost_shared_ptr.i>
%shared_ptr(Learner)

%{
#include "../Learner.hpp"
%}

/* rename java class to prevent name conflict with existing java class*/
%rename (NativeLearner) Learner;

/* build java wrapper of class(es) */
%include "../Learner.hpp"

%ignore factoryLogger;
