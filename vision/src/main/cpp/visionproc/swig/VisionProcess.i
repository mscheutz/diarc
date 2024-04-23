/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/* File: VisionProcess.i */
%module NativeVisionProcModule


%include "std_vector.i"
%template(VectorLong) std::vector<long long>;

%include "std_string.i" /* for std::string type-maps */
%include "typemaps.i"
%include <boost_shared_ptr.i>
%shared_ptr(VisionProcess)

// to properly include/import things from the "common/fol" swig module
%import "common/fol/swig/fol.i"
%pragma(java) jniclassimports=%{
  import edu.tufts.hrilab.vision.common.fol.swig.Predicate;
%}
%typemap(javaimports) VisionProcess "import edu.tufts.hrilab.vision.common.fol.swig.Predicate;"


%{
#include "VisionProcess.hpp"
%}

/* ignore notify() method. isn't needed from Java side */
//%ignore notify();
%ignore notify(Notification::Ptr n);

/* rename java class to prevent name conflict with existing java class*/
%rename (NativeVisionProcess) VisionProcess;

/* build java wrapper of class(es) */
%include "../VisionProcess.hpp"

