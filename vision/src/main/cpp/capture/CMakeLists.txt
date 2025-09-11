PROJECT(capture)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

SET(SOURCES 
	Capture.cpp
	CaptureFile.cpp
	CaptureV4L2.cpp
	v4l2/V4L2Helper.cpp
	util/CaptureUtilities.cpp
	)

# ROS capture (using ROS messages)
IF (roscpp_FOUND AND rosconsole_FOUND AND sensor_msgs_FOUND AND stereo_msgs_FOUND AND image_transport_FOUND AND cv_bridge_FOUND AND tf_FOUND AND pcl_ros_FOUND)
	SET(SOURCES ${SOURCES}
		CaptureMultisense.cpp
		CaptureROS.cpp
	)
ENDIF (roscpp_FOUND AND rosconsole_FOUND AND sensor_msgs_FOUND AND stereo_msgs_FOUND AND image_transport_FOUND AND cv_bridge_FOUND AND tf_FOUND AND pcl_ros_FOUND)

# ROS2 capture
IF (rclcpp_FOUND AND rcl_interfaces_FOUND AND sensor_msgs_FOUND AND stereo_msgs_FOUND AND image_transport_FOUND AND cv_bridge_FOUND AND tf2_FOUND AND pcl_conversions_FOUND)
	SET(SOURCES ${SOURCES}
			CaptureROS2.cpp
	)
ENDIF (rclcpp_FOUND AND rcl_interfaces_FOUND AND sensor_msgs_FOUND AND stereo_msgs_FOUND AND image_transport_FOUND AND cv_bridge_FOUND AND tf2_FOUND AND pcl_conversions_FOUND)

# Spot robot capture
if(bosdyn_FOUND)
	SET(SOURCES ${SOURCES}
			CaptureSpot.cpp
	)
endif(bosdyn_FOUND)

# Intel RealSense capture
if(REALSENSE_FOUND)
	SET(SOURCES ${SOURCES}
			CaptureRealSense.cpp
	)
endif(REALSENSE_FOUND)

# Kinect capture (using OpenNI driver - preferred)
if(OPENNI_FOUND)
	SET(SOURCES ${SOURCES}
		openni/OpenniDevice.cpp
		CaptureOpenni.cpp
	)
endif(OPENNI_FOUND)
if(OPENNI2_FOUND)
	SET(SOURCES ${SOURCES}
			CaptureOpenni2.cpp
	)
endif(OPENNI2_FOUND)

# Capture from Nao head camera
if(NAO_FOUND)
	SET(SOURCES ${SOURCES}
		CaptureNao.cpp
	)
endif(NAO_FOUND)

# Capture via GStreamer
if(GSTREAMER_FOUND AND GLIB_FOUND)
	SET(SOURCES ${SOURCES}
		CaptureGStreamer.cpp
	)
endif(GSTREAMER_FOUND AND GLIB_FOUND)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

ADD_SUBDIRECTORY(calibration)

# link libraries
IF (roscpp_FOUND AND rosconsole_FOUND AND sensor_msgs_FOUND AND stereo_msgs_FOUND AND image_transport_FOUND AND cv_bridge_FOUND AND tf_FOUND AND pcl_ros_FOUND)
	TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${roscpp_LIBRARIES}
	                      ${rosconsole_LIBRARIES}
	                      ${sensor_msgs_LIBRARIES}
	                      ${stereo_msgs_LIBRARIES}
	                      ${image_transport_LIBRARIES}
	                      ${cv_bridge_LIBRARIES}
	                      ${tf_LIBRARIES}
	                      ${pcl_ros_LIBRARIES})
ENDIF (roscpp_FOUND AND rosconsole_FOUND AND sensor_msgs_FOUND AND stereo_msgs_FOUND AND image_transport_FOUND AND cv_bridge_FOUND AND tf_FOUND AND pcl_ros_FOUND)

IF (rclcpp_FOUND AND rcl_interfaces_FOUND AND sensor_msgs_FOUND AND stereo_msgs_FOUND AND image_transport_FOUND AND cv_bridge_FOUND AND tf2_FOUND AND pcl_conversions_FOUND)
#	message("rclcpp_LIBRARIES:${rclcpp_LIBRARIES}")
#	message("sensor_msgs_LIBRARIES:${sensor_msgs_LIBRARIES}")
#	message("stereo_msgs_LIBRARIES:${stereo_msgs_LIBRARIES}")
#	message("image_transport_LIBRARIES:${image_transport_LIBRARIES}")
#	message("cv_bridge_LIBRARIES:${cv_bridge_LIBRARIES}")
#	message("tf2_LIBRARIES:${tf2_LIBRARIES}")
#	message("pcl_conversions_LIBRARIES:${pcl_conversions_LIBRARIES}")
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${roscpp_LIBRARIES}
		${rclcpp_LIBRARIES}
		${sensor_msgs_LIBRARIES}
		${stereo_msgs_LIBRARIES}
		${image_transport_LIBRARIES}
		${cv_bridge_LIBRARIES}
		${tf2_LIBRARIES}
		${pcl_conversions_LIBRARIES}
        "/opt/ros/humble/lib/libimage_transport.so"
        "/opt/ros/humble/lib/libcv_bridge.so")
ENDIF (rclcpp_FOUND AND rcl_interfaces_FOUND AND sensor_msgs_FOUND AND stereo_msgs_FOUND AND image_transport_FOUND AND cv_bridge_FOUND AND tf2_FOUND AND pcl_conversions_FOUND)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${LOG4CXX_LIBRARIES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} display common calibration point_clouds)

if (pybind11_FOUND)
	TARGET_LINK_LIBRARIES(${PROJECT_NAME} pyvis)
endif (pybind11_FOUND)
if(OPENNI_FOUND)
	TARGET_LINK_LIBRARIES(${PROJECT_NAME}  ${OPENNI_LIBRARY})
endif(OPENNI_FOUND)
if(REALSENSE_FOUND)
	TARGET_LINK_LIBRARIES(${PROJECT_NAME}  ${REALSENSE_LIBRARY})
endif(REALSENSE_FOUND)
if(NAO_FOUND)
	TARGET_LINK_LIBRARIES(${PROJECT_NAME}  ${NAO_LIBRARIES})
endif(NAO_FOUND)
if(GSTREAMER_FOUND AND GLIB_FOUND)
	TARGET_LINK_LIBRARIES(${PROJECT_NAME}
			${GLIB_LIBRARIES} ${GLIB_GOBJECT_LIBRARIES}
			${GSTREAMER_LIBRARIES} ${GSTREAMER_APP_LIBRARIES})
endif(GSTREAMER_FOUND AND GLIB_FOUND)
if(bosdyn_FOUND)
	TARGET_LINK_LIBRARIES(${PROJECT_NAME} bosdyn::bosdyn_client)
	TARGET_LINK_LIBRARIES(${PROJECT_NAME} bosdyn::bosdyn_api)
endif(bosdyn_FOUND)

# build java wrappers via swig
ADD_SUBDIRECTORY(swig)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} capture_swig)
