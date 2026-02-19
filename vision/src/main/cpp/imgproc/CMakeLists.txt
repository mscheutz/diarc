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

    # Model trained with objects_c_v5_1500
    SET(COMPONENT_MODEL_PATH "${CONFIG_DIR}/componentvalidator/model_objects_c_v5_1500.onnx")
    SET(COMPONENT_MODEL_URL "https://github.com/hrilabtufts/vision_models/raw/main/componentvalidator/model_objects_c_v5_1500.onnx")

    if (NOT EXISTS "${COMPONENT_MODEL_PATH}")
    	message("Downloading model: ${COMPONENT_MODEL_URL}")
    	file(DOWNLOAD "${COMPONENT_MODEL_URL}" "${COMPONENT_MODEL_PATH}")
    endif()

    # Model trained with objects_c_v1_1500
    SET(COMPONENT_MODEL_PATH_V1 "${CONFIG_DIR}/componentvalidator/model_objects_c_v1_1500.onnx")
    SET(COMPONENT_MODEL_URL_V1 "https://github.com/hrilabtufts/vision_models/raw/main/componentvalidator/model_objects_c_v1_1500.onnx")

    if (NOT EXISTS "${COMPONENT_MODEL_PATH_V1}")
    	message("Downloading model: ${COMPONENT_MODEL_URL_V1}")
    	file(DOWNLOAD "${COMPONENT_MODEL_URL_V1}" "${COMPONENT_MODEL_PATH_V1}")
    endif()
endif(OpenCV_dnn_FOUND)

if (V4R_V0_FOUND)
	SET(SOURCES ${SOURCES}
		saliency/ColorProcessor.cpp
		saliency/IKNSaliencyMapProcessor.cpp
		saliency/LocationSaliencyProcessor.cpp
		saliency/SymmetryProcessor.cpp
		saliency/OrientationProcessor.cpp
		validator/SurfaceMarkingValidator.cpp
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
TARGET_LINK_LIBRARIES(${PROJECT_NAME} common stm visionproc pyvis capture vl shape_match)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${LOG4CXX_LIBRARIES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} pybind11::embed)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${Python3_LIBRARIES})
if (V4R_V0_FOUND) 
	TARGET_LINK_LIBRARIES(${PROJECT_NAME} v4rEPUtils v4rAttentionModule)
endif(V4R_V0_FOUND)

# build java wrappers via swig
ADD_SUBDIRECTORY(swig)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} imgproc_swig)


