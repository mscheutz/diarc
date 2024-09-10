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

SET(CONFIG_DIR "../vision/build/resources/main/config/edu/tufts/hrilab/vision/imgProcs")

# optional sources
if (OpenCV_dnn_FOUND)
	SET(SOURCES ${SOURCES}
	    validator/ComponentValidator.cpp
		)

    SET(COMPONENT_MODEL_PATH "${CONFIG_DIR}/componentvalidator/semanticEncoder.onnx")
    SET(COMPONENT_MODEL_URL "https://github.com/hrilabtufts/vision_models/raw/main/componentvalidator/semanticEncoder.onnx")

    if (NOT EXISTS "${COMPONENT_MODEL_PATH}")
    	message("Downloading semanticEncoder model ${COMPONENT_MODEL_URL}")
    	file(DOWNLOAD "${COMPONENT_MODEL_URL}" "${COMPONENT_MODEL_PATH}")
    endif(NOT EXISTS "${COMPONENT_MODEL_PATH}")
endif(OpenCV_dnn_FOUND)

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


