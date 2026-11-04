/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/* File: CameraCalibration.i */
%module CameraCalibrationModule

%include "std_string.i" /* for std::string type-maps */

%{
#include "./../Cameras.hpp"
%}

/* rename java class (to reflect future name of Cameras class) */
%rename (CameraCalibration) Cameras;

%include "./../Cameras.hpp"
        