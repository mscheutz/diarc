/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/* File: ObjectTracker.i */
%module TrackerModule

/* import base class */
%import "visionproc/swig/VisionProcess.i"
%typemap(javaimports) SWIGTYPE, SWIGTYPE * "import edu.tufts.hrilab.vision.visionproc.swig.*;"

%include "std_string.i" /* for std::string type-maps */
%include "enums.swg"	/* for proper Java enum classes */

%include "typemaps.i"
/* to enable use of "int&" and "bool&" in ObjectTracker.perform(...) */
%apply int &INOUT { int& numAddedObjects };
%apply bool &INOUT { bool& dataProcessed, bool& objectTracked };
//%apply int *INPUT  { int *x, int *y};
//%apply int *OUTPUT  { int *x, int *y};

/* to allow use of boost shared_ptr */
%include <boost_shared_ptr.i>
%shared_ptr(ObjectTracker)

%{
/* native includes */
#include "ObjectTracker.hpp"
%}

/* rename java class to maintin consistent naming convention */
%rename (NativeTracker) ObjectTracker;

/* build java wrapper of these classes */
%include "ObjectTracker.hpp"

