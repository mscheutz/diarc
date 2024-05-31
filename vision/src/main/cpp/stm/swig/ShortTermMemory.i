/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/* File: ShortTermMemory.i */
%module ShortTermMemoryModule

%include "std_string.i" /* for std::string type-maps */
%include "enums.swg" /* for proper Java enum classes */
%include "various.i" /* for char* -> byte[] */

/* to enable use of "double[]" */
%include "arrays_java.i";

%{
  #include "ShortTermMemoryJNI.hpp"
%}

//typemaps
%typemap(jstype) ArrayList "java.util.ArrayList"
%typemap(jtype) ArrayList "java.util.ArrayList"
%typemap(javaout) ArrayList { return $jnicall; }
%typemap(jstype) long* "long[]"
%typemap(jtype) long* "long[]"
%typemap(javaout) long* { return $jnicall; }
%typemap(jstype) MemoryObject "edu.tufts.hrilab.vision.stm.MemoryObject"
%typemap(jtype) MemoryObject "edu.tufts.hrilab.vision.stm.MemoryObject"
%typemap(javaout) MemoryObject { return $jnicall; }

//native hand-written methods (defined in ShortTermMemoryJNI.hpp)
%native(getMemoryObjects) ArrayList getMemoryObjects(); //ArrayList<MemoryObject>
%native(getMemoryObjectsByTypeId) ArrayList getMemoryObjectsByTypeId(long long moTypeId);
%native(getMemoryObjectTypeIds) long* getMemoryObjectTypeIds();
%native(getMemoryObjectIds) long* getMemoryObjectIds(long long  moTypeId);
%native(getMemoryObject) MemoryObject getMemoryObject(long long  tokenId);
%native(clearMemory) void clearMemory(); //delete all tracked objects
%native(getRemovedMemoryObjects) ArrayList getRemovedMemoryObjects();
