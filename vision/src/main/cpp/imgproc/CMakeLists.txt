PROJECT(imgproc)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

SET(SOURCES
	ChangeDetector.cpp
	ImageProcessor.cpp
	OpticalFlowProcessor.cpp
	PlaneProcessor.cpp
	SiftProcessor.cpp
	calcOpticalFlow/CalculateOpticalFlow.cpp
	saliency/MasterSaliencyProcessor.cpp
	saliency/SaliencyProcessor.cpp
	siftFeatures/SiftFeatures.cpp
	validator/ColorValidator.cpp
	validator/DefinitionValidator.cpp
	validator/GlobalFeatureValidator.cpp
	validator/ObjectValidator.cpp
	validator/SpatialRelationValidator.cpp
	validator/SizeValidator.cpp
	)

# optional sources
if (OpenCV_dnn_FOUND)
	SET(SOURCES ${SOURCES}
	    validator/ComponentValidator.cpp
		)
	# if (NOT EXISTS "${YOLO_MODEL_PATH}")
	# 	message("Downloading YOLOv3 model ${YOLO_MODEL_URL}")
	# 	file(DOWNLOAD "${YOLO_MODEL_URL}" "${YOLO_MODEL_PATH}")
	# endif(NOT EXISTS "${YOLO_MODEL_PATH}")
endif(OpenCV3_dnn_FOUND)

if (V4R_V0_FOUND)
	SET(SOURCES ${SOURCES}
		saliency/ColorProcessor.cpp
		saliency/IKNSaliencyMapProcessor.cpp
		saliency/LocationSaliencyProcessor.cpp
		saliency/SymmetryProcessor.cpp
		saliency/OrientationProcessor.cpp
		validator/ShapeValidator.cpp
		validator/SurfaceMarkingValidator.cpp
		validator/OrientationValidator.cpp
	)
endif(V4R_V0_FOUND)

if (V4R_V0_FOUND)
	SET(SOURCES ${SOURCES}
#saliency/RelativeHeightProcessor.cpp		
#saliency/SurfaceCurvatureProcessor.cpp	
#saliency/SurfaceOrientationProcessor.cpp
		)
endif(V4R_V0_FOUND)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES} )

# link libraries
TARGET_LINK_LIBRARIES(${PROJECT_NAME} common stm visionproc capture vl shape_match)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${LOG4CXX_LIBRARIES})
if (V4R_V0_FOUND) 
	TARGET_LINK_LIBRARIES(${PROJECT_NAME} v4rEPUtils v4rAttentionModule)
endif(V4R_V0_FOUND)

# build java wrappers via swig
ADD_SUBDIRECTORY(swig)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} imgproc_swig)


