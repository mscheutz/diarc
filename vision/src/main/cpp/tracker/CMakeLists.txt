PROJECT(tracker)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/cmt)

SET(SOURCES
	GenericTracker.cpp
	ObjectTracker.cpp
	CMTTracker.cpp
	RelationTracker.cpp
	)

# optional sources
if (OpenCV_tracking_FOUND)
	if (OpenCV_VERSION_MINOR GREATER 1)
	SET(SOURCES ${SOURCES}
			OpenCVTracker.cpp
			KCFTracker.cpp
			TLDTracker.cpp
			)
	endif()
endif(OpenCV_tracking_FOUND)

if (V4R_FOUND)
	SET(SOURCES ${SOURCES}
		V4RTracker.cpp
#		util/V4RTrackingModelIO.cpp
	)
endif(V4R_FOUND)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

# link libraries
TARGET_LINK_LIBRARIES(${PROJECT_NAME} common stm visionproc capture cmt)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${LOG4CXX_LIBRARIES})
if (V4R_FOUND)
	TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${V4R_LIBS})
endif(V4R_FOUND)

# build java wrappers via swig
ADD_SUBDIRECTORY(swig)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} tracker_swig)