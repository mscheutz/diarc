/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/* File: ObjectDetector.i */
%module DetectorModule

/* import base class */
%import "visionproc/swig/VisionProcess.i"
%typemap(javaimports) SWIGTYPE, SWIGTYPE * "import edu.tufts.hrilab.vision.visionproc.swig.*;"

%include "std_string.i" /* for std::string type-maps */
%include "enums.swg"	/* for proper Java enum classes */

%include "typemaps.i"
/* to enable use of "int&" and "bool&" in ObjectDetector.perform(...) */
%apply int &INOUT { int& };
%apply bool &INOUT { bool& };
//%apply int *INPUT  { int *x, int *y};
//%apply int *OUTPUT  { int *x, int *y};

/* to allow use of boost shared_ptr */
%include <boost_shared_ptr.i>
%shared_ptr(ObjectDetector)

%{
/* native includes */
#include "../ObjectDetector.hpp"
%}

/* rename java class to maintin consistent naming convention */
%rename (NativeDetector) ObjectDetector;

/* build java wrapper of these classes */
%include "../ObjectDetector.hpp"

