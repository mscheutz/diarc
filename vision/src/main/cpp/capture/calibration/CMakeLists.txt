PROJECT(calibration)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

SET(SOURCES 
	Cameras.cpp 
	DepthSensorCalibration.cpp
	StereoCalibration.cpp
	)

ADD_LIBRARY(${PROJECT_NAME} ${SOURCES})

# build java wrappers via swig
ADD_SUBDIRECTORY(swig)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} calibration_swig)