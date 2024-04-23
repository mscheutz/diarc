/* File: FloorSupport.i */
%module FloorSupportNative

%include "std_string.i"
%javaconst(1);

%{
#include "../FloorSupport.h"
%} 

%include "../FloorSupport.h"
