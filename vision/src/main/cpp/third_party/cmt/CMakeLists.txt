project(cmt)

add_definitions("-Wall")

SET(SOURCES
        common.cpp
        gui.cpp
        CMT.cpp
        Consensus.cpp
        Fusion.cpp
        Matcher.cpp
        Tracker.cpp
        fastcluster/fastcluster.cpp)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBS} ${LOG4CXX_LIBRARIES})

