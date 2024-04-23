/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/* File: ImageProcessor.i */
%module ImgProcModule

/* import base class */
%import "visionproc/swig/VisionProcess.i"
%typemap(javaimports) SWIGTYPE, SWIGTYPE * "import edu.tufts.hrilab.vision.visionproc.swig.*;"

//%pragma(java) jniclassimports="import JNIversion.visionproc.swig.*;" 
//%pragma(java) moduleimports="import JNIversion.visionproc.swig.*;" 

%include "std_string.i" /* for std::string type-maps */
%include "enums.swg"	/* for proper Java enum classes */
%include <boost_shared_ptr.i>
%shared_ptr(ImageProcessor)

%{
#include "../ImageProcessor.hpp"
%}

/* rename java class to prevent name conflict with existing java class*/
%rename (NativeImageProcessor) ImageProcessor;

/* build java wrapper of class(es) */
%include "../ImageProcessor.hpp"

%ignore factoryLogger;
