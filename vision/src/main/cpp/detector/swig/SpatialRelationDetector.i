/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/* File: SpatialRelationDetector.i */
%module SpatialRelationDetectorModule

%import "ObjectDetector.i" /* import base class */

%include "std_string.i" /* for std::string type-maps */
%include "typemaps.i"

%include "various.i"
%apply char **STRING_ARRAY { char ** };

//%include "string_array.i"
//%include "carrays.i"
//%array_class(string, stringArray);

/* to enable use of "int[]" */
%include "arrays_java.i";

/* to enable use of "int&" and "bool&" */
%apply int &INOUT { int& };
%apply bool &INOUT { bool& };

/* to allow use of boost shared_ptr */
%include <boost_shared_ptr.i>
%shared_ptr(SpatialRelationDetector)


%{
/* native includes */
#include "../SpatialRelationDetector.hpp"
%}


/* build java wrapper of these classes */
%include "../SpatialRelationDetector.hpp"


/* to allow down-casting in java */
%extend SpatialRelationDetector {
    static SpatialRelationDetector::Ptr dynamic_cast(ObjectDetector::Ptr detector) {
	return boost::dynamic_pointer_cast<SpatialRelationDetector>(detector);
    }
};