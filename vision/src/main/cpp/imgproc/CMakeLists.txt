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

    # Model trained on 500 one, 1000 two, 2000 three component objects.
    SET(COMPONENT_MODEL_PATH_V1 "${CONFIG_DIR}/componentvalidator/modelv2_less2comp_epochs800.onnx")
    SET(COMPONENT_MODEL_URL_V1 "https://github.com/hrilabtufts/vision_models/raw/main/componentvalidator/modelv2_less2comp_epochs800.onnx")

    if (NOT EXISTS "${COMPONENT_MODEL_PATH_V1}")
    	message("Downloading model: ${COMPONENT_MODEL_URL_V1}")
    	file(DOWNLOAD "${COMPONENT_MODEL_URL_V1}" "${COMPONENT_MODEL_PATH_V1}")
    endif()

    # Model trained on 500 one, 2000 two, 2000 three component objects.
    SET(COMPONENT_MODEL_PATH_V2 "${CONFIG_DIR}/componentvalidator/modelv2_more2comp_epochs800.onnx")
    SET(COMPONENT_MODEL_URL_V2 "https://github.com/hrilabtufts/vision_models/raw/main/componentvalidator/modelv2_more2comp_epochs800.onnx")

    if (NOT EXISTS "${COMPONENT_MODEL_PATH_V2}")
    	message("Downloading model: ${COMPONENT_MODEL_URL_V2}")
    	file(DOWNLOAD "${COMPONENT_MODEL_URL_V2}" "${COMPONENT_MODEL_PATH_V2}")
    endif()
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




