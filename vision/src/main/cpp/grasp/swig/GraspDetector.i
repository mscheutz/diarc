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
#include "../GraspDetector.hpp"
%}

/* build java wrapper of these classes */
%include "../GraspDetector.hpp"

