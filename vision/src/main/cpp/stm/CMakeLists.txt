PROJECT(stm)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

SET(SOURCES 
	ArrayListInterface.cpp 
	ConfidenceLevel.cpp
	GraspInterface.cpp
	GraspValidationResult.cpp
	MemoryObject.cpp
	MemoryObjectInterface.cpp
	MemoryObjectMask.cpp
    RelationValidationResult.cpp
    TrackedObjects.cpp
    util/StmUtilities.cpp
    ValidationResult.cpp
    ValidationResults.cpp
	)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

# link libraries
#TARGET_LINK_LIBRARIES(${PROJECT_NAME} tracker)
# TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${LOG4CXX_LIBRARIES})

# build java wrappers via swig
ADD_SUBDIRECTORY(swig)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} stm_swig)