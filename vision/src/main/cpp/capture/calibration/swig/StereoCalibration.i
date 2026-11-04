/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/* File: StereoCalibration.i */
%module StereoCalibrationModule

/* to enable use of "double[]" */
%include "arrays_java.i";
%apply double[] { double* };

%include "std_string.i" /* for std::string type-maps */
%include "enums.swg"	/* for proper Java enum classes */
%include "typemaps.i"
/* to enable use of "double&" */
%apply double &INOUT { double& };
//%apply int &INOUT { int& };


%{
#include "./../StereoCalibration.hpp"
%}

%include "./../StereoCalibration.hpp"
