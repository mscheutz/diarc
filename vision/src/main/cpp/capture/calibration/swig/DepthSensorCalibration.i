/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/* File: DepthSensorCalibration.i */
%module DepthSensorCalibrationModule

//%include "arrays_java.i"
//%apply double[] {double *};

//%import "./../../swig/Capture.i"

%include "enums.swg"	/* for proper Java enum classes */
%include "typemaps.i"
/* to enable use of "double&" */
%apply double &INOUT { double& };
//%apply int &INOUT { int& };


%{
#include "./../DepthSensorCalibration.hpp"
%}

%include "./../DepthSensorCalibration.hpp"
