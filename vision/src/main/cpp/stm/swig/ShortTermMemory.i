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
%typemap(jstype) ArrayList getMemoryObjects "java.util.ArrayList"
%typemap(jtype) ArrayList getMemoryObjects "java.util.ArrayList"
%typemap(javaout) ArrayList getMemoryObjects { return $jnicall; }
%typemap(jstype) ArrayList getMemoryObjectsByTypeId "java.util.ArrayList"
%typemap(jtype) ArrayList getMemoryObjectsByTypeId "java.util.ArrayList"
%typemap(javaout) ArrayList getMemoryObjectsByTypeId { return $jnicall; }
%typemap(jstype) long* getMemoryObjectTypeIds "long[]"
%typemap(jtype) long* getMemoryObjectTypeIds "long[]"
%typemap(javaout) long* getMemoryObjectTypeIds { return $jnicall; }
%typemap(jstype) long* getMemoryObjectIds "long[]"
%typemap(jtype) long* getMemoryObjectIds "long[]"
%typemap(javaout) long* getMemoryObjectIds { return $jnicall; }
%typemap(jstype) MemoryObject getMemoryObject "edu.tufts.hrilab.vision.stm.MemoryObject"
%typemap(jtype) MemoryObject getMemoryObject "edu.tufts.hrilab.vision.stm.MemoryObject"
%typemap(javaout) MemoryObject getMemoryObject { return $jnicall; }
%typemap(jstype) ArrayList getRemovedMemoryObjects "java.util.ArrayList"
%typemap(jtype) ArrayList getRemovedMemoryObjects "java.util.ArrayList"
%typemap(javaout) ArrayList getRemovedMemoryObjects { return $jnicall; }

//native hand-written methods (defined in ShortTermMemoryJNI.hpp)
%native(getMemoryObjects) ArrayList getMemoryObjects(double conf); //ArrayList<MemoryObject>
%native(getMemoryObjectsByTypeId) ArrayList getMemoryObjectsByTypeId(long long moTypeId, double conf);
%native(getMemoryObjectTypeIds) long* getMemoryObjectTypeIds(double conf);
%native(getMemoryObjectIds) long* getMemoryObjectIds(long long  moTypeId, double conf);
%native(getMemoryObject) MemoryObject getMemoryObject(long long  tokenId, double conf);
%native(clearMemory) void clearMemory(); //delete all tracked objects
%native(getRemovedMemoryObjects) ArrayList getRemovedMemoryObjects();
