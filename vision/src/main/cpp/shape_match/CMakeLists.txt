PROJECT(shape_match)

#set the include directories
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

SET(SOURCES
	shapecontext.cpp
	)


#libraries
ADD_LIBRARY(${PROJECT_NAME} ${SOURCES})

#link the libraries
TARGET_LINK_LIBRARIES(${PROJECT_NAME} shapecontext)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBS} ${LOG4CXX_LIBRARIES})
