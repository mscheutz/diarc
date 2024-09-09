/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/* File: fol.i */
%module folModule

// to change getCPtr from protected to public so it can be used in other swig modules
SWIG_JAVABODY_METHODS(protected, public, SWIGTYPE)

%include "std_string.i" /* for std::string type-maps */

/* to allow use of boost shared_ptr */
//%include <boost_shared_ptr.i>
//%shared_ptr(diarc::common::fol::Symbol)
//%shared_ptr(diarc::common::fol::Variable)
//%shared_ptr(diarc::common::fol::Predicate)
//        
/* to enable use of "Symbol[]" */
//%include "arrays_java.i";
////JAVA_ARRAYSOFCLASSES(diarc::common::fol::Symbol);
//JAVA_ARRAYSOFCLASSES(diarc::common::fol::Symbol::Ptr);
//JAVA_ARRAYSOFCLASSES(boost::shared_ptr<diarc::common::fol::Symbol>);
//JAVA_ARRAYSOFCLASSES(boost::shared_ptr<Symbol>);

%{
#include "../Symbol.hpp"
%}
%include "../Symbol.hpp"

%{
#include "../Variable.hpp"
%}
%include "../Variable.hpp"

%ignore Predicate(const std::string& name, const std::vector<Symbol::ConstPtr>& args);
%ignore getArg(const long unsigned int i);
%{
#include "../Predicate.hpp"
%}

%include "../Predicate.hpp"
        
%{
#include "../PredicateBuilder.hpp"
%}

%include "../PredicateBuilder.hpp"

