/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/* File: BlobDetector.i */
%module BlobDetectorModule

%import "ObjectDetector.i" /* import base class */

%include "std_string.i" /* for std::string type-maps */
%include "typemaps.i"

%include "various.i"
%apply char **STRING_ARRAY { char ** };

//%include "string_array.i"
//%include "carrays.i"
//%array_class(string, stringArray);

/* to enable use of "int[]" */
%include "arrays_java.i"

/* to enable use of "int&" and "bool&" */
%apply int &INOUT { int& };
%apply bool &INOUT { bool& };

/* to allow use of boost shared_ptr */
%include <boost_shared_ptr.i>
%shared_ptr(BlobDetector)


%{
/* native includes */
#include "../BlobDetector.hpp"
%}


/* build java wrapper of these classes */
%include "../BlobDetector.hpp"


/* to allow down-casting in java */
%extend BlobDetector {
    static BlobDetector::Ptr dynamic_cast(ObjectDetector::Ptr detector) {
	return boost::dynamic_pointer_cast<BlobDetector>(detector);
    }
};

/* to throw incorrect boost down-cast as Java exception */
/* note: boost doesn't throw exception, just returns empty shared_ptr */
/*
%exception FaceDetector::dynamic_cast(ObjectDetectorPtr detector) {
    $action
    if (result == FaceDetectorPtr()) {
        jclass excep = jenv->FindClass("java/lang/ClassCastException");
        if (excep) {
            jenv->ThrowNew(excep, "boost::dynamic_pointer_cast exception");
        }
        return $null;
    }
}
*/

