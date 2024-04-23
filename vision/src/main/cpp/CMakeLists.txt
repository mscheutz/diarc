cmake_minimum_required(VERSION 3.5)

PROJECT(DIARC_VISION)
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})
# set(CMAKE_VERBOSE_MAKEFILE on )

message("DIARC VISION SRC DIR: ${DIARC_VISION_SOURCE_DIR}") # same ${CMAKE_SOURCE_DIR}
INCLUDE_DIRECTORIES(${DIARC_VISION_SOURCE_DIR})

# Global linker options
SET(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-as-needed")

# Set build type (Debug, Release, RelWithDebInfo)
# If the user specifies -DCMAKE_BUILD_TYPE on the command line, take their definition
# and dump it in the cache along with proper documentation, otherwise set CMAKE_BUILD_TYPE
# to RelWithDebInfo prior to calling PROJECT()
IF(DEFINED CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of
	build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug
	Release RelWithDebInfo MinSizeRel.")
ELSE()
   SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose the type of build,
	options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release
	RelWithDebInfo MinSizeRel.")
ENDIF()
#SET(CMAKE_BUILD_TYPE Debug)

# Compile options
#ADD_DEFINITIONS(-std=c++11)
ADD_DEFINITIONS(-fPIC)
ADD_DEFINITIONS(-DBOOST_FILESYSTEM_DEPRECATED)

# turn no return-value warnings into errors
ADD_DEFINITIONS(-Werror=return-type)

# set explciit cmake policies
CMAKE_POLICY(SET CMP0012 NEW)
if(CMAKE_VERSION VERSION_LESS "3.13")
  message("Please consider upgrading CMake to 3.13 or newer")
else()
  CMAKE_POLICY(SET CMP0078 NEW)
  CMAKE_POLICY(SET CMP0086 NEW)
endif()

# Set paths between Vision-Java, Vision-Native and ADE
SET(VISION_BASE_PACKAGE edu.tufts.hrilab.vision)
SET(SWIG_SRC_BASE_OUTDIR ${CMAKE_SOURCE_DIR}/../../swig/java)
SET(SWIG_SRC_OUTDIR ${SWIG_SRC_BASE_OUTDIR}/edu/tufts/hrilab/vision)

# Where to build shared libs
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

#set the default path for built executables
#SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

# Java (required)
FIND_PACKAGE( Java REQUIRED )
INCLUDE_DIRECTORIES(${JAVA_INCLUDE_DIRS})

# JNI (required)
FIND_PACKAGE( JNI REQUIRED )
INCLUDE_DIRECTORIES(${JNI_INCLUDE_DIRS})

# SWIG (required)
FIND_PACKAGE( SWIG 3.0 REQUIRED COMPONENTS java )

# OpenCV (required)
FIND_PACKAGE( OpenCV REQUIRED )
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

# check for dnn module
FIND_PACKAGE( OpenCV 3.4 QUIET OPTIONAL_COMPONENTS dnn)
if(OpenCV3_dnn_FOUND)
	message("Found OpenCV 3 DNN module.")
	add_definitions("-DOPENCV3_DNN")
else()
	message("Did NOT find suitable OpenCV 3 DNN module (must be OpenCV 3.4+).")
endif(OpenCV3_dnn_FOUND)

FIND_PACKAGE( OpenCV 4.5 QUIET OPTIONAL_COMPONENTS dnn)
if(OpenCV_dnn_FOUND)
	message("Found OpenCV 4 DNN module.")
	add_definitions("-DOPENCV_DNN")
	FIND_PACKAGE(CUDNN 8 QUIET)
	if(CUDNN_FOUND)
		message("Found CuDNN. Enabling CUDA features in OpenCV dnn module.")
		add_definitions("-DCUDNN")
	else()
		message("CuDNN 8+ not found at install-time. Cannot use CUDA features in OpenCV dnn module.")
	endif(CUDNN_FOUND)
else()
	message("Did NOT find suitable OpenCV DNN module (must be OpenCV 4.5+).")
endif(OpenCV_dnn_FOUND)

# check for tracking module
FIND_PACKAGE( OpenCV QUIET OPTIONAL_COMPONENTS tracking)
if(OpenCV_tracking_FOUND)
	message("Found OpenCV Tracking module.")
	add_definitions("-DOPENCV_TRACKING")
else()
	message("Did NOT find OpenCV Tracking module.")
endif(OpenCV_tracking_FOUND)

# PCL (required)
FIND_PACKAGE(PCL 1.8 REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})
# this is to fix a PCL bug on ubuntu 22.04 -- this include should be part of PCL_INCLUDE_DIRS but is missing
INCLUDE_DIRECTORIES("/usr/include/vtk-9.1")
#message("PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
#message("PCL_LIBRARY_DIRS: ${PCL_LIBRARY_DIRS}")
#message("PCL_DEFINITIONS: ${PCL_DEFINITIONS}")

if(OPENNI_FOUND)
	ADD_DEFINITIONS(-DUSE_OPENNI)
endif(OPENNI_FOUND)
if(OPENNI2_FOUND)
	ADD_DEFINITIONS(-DUSE_OPENNI2)
endif(OPENNI2_FOUND)

# Boost (required) -- this is only needed to patch the bug in PCLConfig.cmake that leaves out the "thread" component
find_package(Boost COMPONENTS system filesystem date_time iostreams thread)

# SWIG (required)
FIND_PACKAGE(SWIG REQUIRED)
INCLUDE(UseSWIG)

# log4cxx (required)
FIND_PACKAGE( LOG4CXX REQUIRED )
INCLUDE_DIRECTORIES(${LOG4CXX_INCLUDE_DIRS})

# jsoncpp (required)
FIND_PACKAGE( JSONCPP REQUIRED )
INCLUDE_DIRECTORIES(${JSON_INC_PATH})

# SIFT (checked into ADE-Vision repo)
LINK_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/third_party/vlfeat/bin/glnxa64 )

# REALSENSE TODO
 FIND_PACKAGE(REALSENSE)
 if (REALSENSE_FOUND)
    INCLUDE_DIRECTORIES(${REALSENSE_INCLUDE_DIRS})
    LINK_DIRECTORIES(${REALSENSE_LIBRARY_DIRS})
    ADD_DEFINITIONS(-DUSE_REALSENSE)
 endif(REALSENSE_FOUND)

SET(USE_PCL_PEOPLE 0 CACHE BOOL "Use PCL People Detection. Currently not finished." )
if (USE_PCL_PEOPLE)
	if (PCL_GPU_PEOPLE_LIBRARIES)
		SET(PCL_GPU_PEOPLE_FOUND ON)
		ADD_DEFINITIONS(-DUSE_PCL_PEOPLE)
		message("PCL_GPU_PEOPLE_FOUND:${PCL_GPU_PEOPLE_FOUND}")
	endif(PCL_GPU_PEOPLE_LIBRARIES)


	# CUDA
	FIND_PACKAGE(CUDA)
	if(CUDA_FOUND)
		#Include the FindCUDA script
		INCLUDE(FindCUDA)

		INCLUDE_DIRECTORIES(${CUDA_INCLUDE_DIRS})
		LINK_DIRECTORIES(${CUDA_LIBRARY_DIRS})
	endif(CUDA_FOUND)
endif(USE_PCL_PEOPLE)

# ROS
OPTION(USE_ROS "Turn on/off integration with ROS." OFF)
IF(USE_ROS)
	FIND_PACKAGE(ROS COMPONENTS rosconsole roscpp sensor_msgs stereo_msgs image_transport cv_bridge tf pcl_ros)
#	FIND_PACKAGE(ROS COMPONENTS rosconsole roscpp sensor_msgs stereo_msgs image_transport cv_bridge tf pcl_ros c_her)
	IF(ROS_FOUND)
		LINK_DIRECTORIES(${ROS_LIBRARY_DIRS})
		INCLUDE_DIRECTORIES(${ROS_INCLUDE_DIR})
		ADD_DEFINITIONS(-DUSE_ROS_VISION)
	ENDIF(ROS_FOUND)
ENDIF(USE_ROS)

# TODO: FREENECT
if(FREENECT_FOUND)
ADD_DEFINITIONS(-DUSE_KINECT)
endif(FREENECT_FOUND)

# V4R (OLD version)
#FIND_PACKAGE(v4r) #TODO: write proper FindV4R cmake module
#FIND_LIBRARY(V4R_LIBRARIES NAMES v4rEPUtils v4rAttentionModule v4rSurfaceSegmenter)
#message("V4R_LIBRARIES:${V4R_LIBRARIES}")
if(V4R_LIBRARIES)
	SET(V4R_V0_FOUND ON)
	ADD_DEFINITIONS(-DUSE_V4R_V0)
endif(V4R_LIBRARIES)

#V4R (new apt-get version)
#FIND_PACKAGE(V4R)
if(V4R_FOUND)
	message("V4R_LIBRARIES:${V4R_LIBS}")
	message("V4R_INCLUDE_DIRS:${V4R_INCLUDE_DIRS}")
	INCLUDE_DIRECTORIES(${V4R_INCLUDE_DIRS})
	ADD_DEFINITIONS(-DUSE_V4R)
	ADD_DEFINITIONS(-DHAVE_SIFTGPU)
endif(V4R_FOUND)


# NAO
#FIND_PACKAGE(NAO) TODO: write proper FindNAO cmake module
#FIND_LIBRARY(NAO_LIBRARIES NAMES alvision PATHS ${NAO_SDK_DIR}/lib)
message("NAO_LIBRARIES:${NAO_LIBRARIES}")
if(NAO_LIBRARIES)
#	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_GLIBCXX_USE_CXX11_ABI=0")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x -D_GLIBCXX_USE_CXX11_ABI=0")
	SET(NAO_FOUND ON)
	ADD_DEFINITIONS(-DUSE_NAO)
    INCLUDE_DIRECTORIES(${NAO_SDK_DIR}/include)
#	LINK_DIRECTORIES(${NAO_SDK_DIR}/lib)
endif(NAO_LIBRARIES)

# GSTREAMER
OPTION(USE_GSTREAMER "Turn on/off integration with GStreamer." OFF)
if(USE_GSTREAMER)
FIND_PACKAGE(GStreamer COMPONENTS gstreamer-app)
FIND_PACKAGE(GLIB COMPONENTS gobject)
if(GSTREAMER_FOUND AND GLIB_FOUND)
	ADD_DEFINITIONS(-DUSE_GSTREAMER)
	INCLUDE_DIRECTORIES(${GSTREAMER_INCLUDE_DIRS} ${GSTREAMER_APP_INCLUDE_DIRS})
	INCLUDE_DIRECTORIES(${GLIB_INCLUDE_DIRS})
endif(GSTREAMER_FOUND AND GLIB_FOUND)
endif(USE_GSTREAMER)

#AGILEGRASP
FIND_PACKAGE(AGILEGRASP)
if(AGILEGRASP_FOUND)
  INCLUDE_DIRECTORIES(${AGILEGRASP_INCLUDE_DIRS})
  ADD_DEFINITIONS(-DUSE_AGILEGRASP)
endif(AGILEGRASP_FOUND)

#OPENPOSE -- requires c++11
OPTION(USE_OPENPOSE "Turn on/off integration with OpenPose." OFF)
if(USE_OPENPOSE)
FIND_PACKAGE(OpenPose)
if(OpenPose_FOUND)
#    ADD_DEFINITIONS(-std=c++11)
    INCLUDE_DIRECTORIES(${OpenPose_INCLUDE_DIRS})
    ADD_DEFINITIONS(-DUSE_OPENPOSE)
endif(OpenPose_FOUND)
endif(USE_OPENPOSE)

#ZBAR
FIND_PACKAGE(ZBAR)
if(ZBAR_FOUND)
	INCLUDE_DIRECTORIES(${ZBAR_INCLUDE_DIR})
	ADD_DEFINITIONS(-DUSE_ZBAR)
endif(ZBAR_FOUND)


# Unit Tests (using googletest)
#FIND_PACKAGE(GTest)
#if(GTest_FOUND)
#  ADD_SUBDIRECTORY(test)
#endif(GTest_FOUND)

ADD_SUBDIRECTORY(capture)
ADD_SUBDIRECTORY(common)
ADD_SUBDIRECTORY(detector)
ADD_SUBDIRECTORY(display)
ADD_SUBDIRECTORY(imgproc)
ADD_SUBDIRECTORY(learn)
ADD_SUBDIRECTORY(point_clouds)
ADD_SUBDIRECTORY(shape_match)
ADD_SUBDIRECTORY(stm)
ADD_SUBDIRECTORY(third_party)
ADD_SUBDIRECTORY(tracker)
ADD_SUBDIRECTORY(visionproc)
