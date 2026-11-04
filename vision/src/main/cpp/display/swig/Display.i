/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/* File: Display.i */
%module DisplayModule

//%include "std_string.i" /* for std::string type-maps */
//%include "enums.swg"	/* for proper Java enum classes */
//%include <boost_shared_ptr.i>
//%shared_ptr(Display)

%{
#include "../Display.hpp"
%}

/* rename java class to maintain consistent naming convention */
%rename (NativeDisplay) Display;

%include "../Display.hpp"

