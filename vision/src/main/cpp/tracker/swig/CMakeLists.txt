PROJECT(tracker_swig)

# set package and outdir
LIST(APPEND options -package ${VISION_BASE_PACKAGE}.tracker.swig)
SET_SOURCE_FILES_PROPERTIES(ObjectTracker.i PROPERTIES SWIG_FLAGS "${options}")
SET(CMAKE_SWIG_OUTDIR ${SWIG_SRC_OUTDIR}/tracker/swig)

SET_SOURCE_FILES_PROPERTIES(ObjectTracker.i PROPERTIES CPLUSPLUS ON)
SWIG_ADD_LIBRARY(${PROJECT_NAME} TYPE SHARED LANGUAGE java SOURCES ObjectTracker.i)
