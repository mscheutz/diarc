/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/* File: Common.i */
%module CommonModule

%include "std_string.i" /* for std::string type-maps */

%{
#include "CommonJNI.hpp"
%}

%native(setLoggingConfiguration) void setLoggingConfiguration(std::string);
%native(hasV4R) bool hasV4R();
%native(hasV4RV0) bool hasV4RV0();
%native(hasOpenPose) bool hasOpenPose();
%native(hasAgileGrasp) bool hasAgileGrasp();
%native(hasZBar) bool hasZBar();
%native(hasOpenCVTracking) bool hasOpenCVTracking();
%native(hasOpenniPeople) bool hasOpenniPeople();