PROJECT(calibration_swig)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

# set package and outdir
LIST(APPEND options -package ${VISION_BASE_PACKAGE}.capture.calibration.swig)
SET_SOURCE_FILES_PROPERTIES(CameraCalibration.i StereoCalibration.i DepthSensorCalibration.i PROPERTIES SWIG_FLAGS "${options}")
SET(CMAKE_SWIG_OUTDIR ${SWIG_SRC_OUTDIR}/capture/calibration/swig)

SET_SOURCE_FILES_PROPERTIES(CameraCalibration.i StereoCalibration.i DepthSensorCalibration.i PROPERTIES CPLUSPLUS ON)
#SWIG_ADD_MODULE(CameraCalibrationModule java CameraCalibration.i ../Cameras.cpp)
#SWIG_ADD_MODULE(StereoCalibrationModule java StereoCalibration.i ../StereoCalibration.cpp)
#SWIG_ADD_MODULE(DepthSensorCalibrationModule java DepthSensorCalibration.i ../DepthSensorCalibration.cpp)
SWIG_ADD_LIBRARY(${PROJECT_NAME} TYPE SHARED LANGUAGE java SOURCES CameraCalibration.i StereoCalibration.i DepthSensorCalibration.i)
