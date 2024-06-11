/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/* File: Capture.i */
%module CaptureModule

%include "std_string.i" /* for std::string type-maps */
%include "enums.swg"	/* for proper Java enum classes */
%include "various.i"    /* for char* -> byte[] */

/* to enable use of "double[]" */
%include "arrays_java.i";

/* to enable boost shared_ptr */
%include <boost_shared_ptr.i>
%shared_ptr(diarc::capture::Capture)

// don't generate Java versions of these methods
%ignore getLastCaptureNotification;

%{
//"pass-through" include directives
#include "CaptureEnums.hpp"
#include "Capture.hpp"
#include "CaptureJNI.hpp"
%}

//generate wrapper code
%include "Capture.hpp"
%include "CaptureEnums.hpp"

//native hand-written methods (defined in CaptureJNI.hpp)
%native(passBackImageArray) void passBackImageArray(char* BYTE, int blurAmount);
%native(passBackDisparityArray) void passBackDisparityArray(char* BYTE);
%native(passBackDepthArray) void passBackDepthArray(char* BYTE);
%native(checkDarkness) bool checkDarkness();

