PROJECT(detector)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

SET(SOURCES
	ArucoDetector.cpp
	BlobDetector.cpp
	ClusterDetector.cpp
	ClusterDetector2D.cpp
	ColorBasedDetector.cpp
	FaceDetector.cpp
	FoodDetectorByHSV.cpp
	HoughDetector.cpp
	MotionDetector.cpp
	ObjectDetector.cpp
	SiftDetector.cpp
	SurfaceMarkingDetector.cpp
	SpatialRelationDetector.cpp
	fastblobdetect/FastBlobDetector.cpp
	hough/GHT.cpp
	)

SET(USER_HOME_DIR $ENV{HOME})
SET(MODEL_DIR "${USER_HOME_DIR}/.diarc/vision/detectors/")

SET(YOLO_MODEL_PATH "${MODEL_DIR}yolo/yolov3-tiny.weights")
SET(YOLO_MODEL_URL "https://github.com/hrilabtufts/vision_models/raw/main/yolo/yolov3-tiny.weights")

SET(BUTTONDETECTOR_MODEL_PATH "${MODEL_DIR}/buttondetector/buttonModelS320.onnx")
SET(BUTTONDETECTOR_MODEL_URL "https://github.com/hrilabtufts/vision_models/raw/main/buttondetector/buttonModelS320.onnx")

SET(PERSONDETECTOR_MODEL_PATH "${MODEL_DIR}/persondetector/crowdhuman_yolov5m.onnx")
SET(PERSONDETECTOR_MODEL_URL "https://github.com/hrilabtufts/vision_models/raw/main/persondetector/crowdhuman_yolov5m.onnx")

SET(ELEVATORDOORDETECTOR_MODEL_PATH "${MODEL_DIR}/elevatordoordetector/elevatordoordetectorS320.onnx")
SET(ELEVATORDOORDETECTOR_MODEL_URL "https://github.com/hrilabtufts/vision_models/raw/main/elevatordoordetector/elevatordoordetectorS320.onnx")

# optional sources
if (OpenCV3_dnn_FOUND)
	SET(SOURCES ${SOURCES}
			NeuralDetector.cpp
			YoloDetector.cpp
		)
	if (NOT EXISTS "${YOLO_MODEL_PATH}")
		message("Downloading YOLOv3 model ${YOLO_MODEL_URL}")
		file(DOWNLOAD "${YOLO_MODEL_URL}" "${YOLO_MODEL_PATH}")
	endif(NOT EXISTS "${YOLO_MODEL_PATH}")
endif(OpenCV3_dnn_FOUND)

if (OpenCV_dnn_FOUND)
	SET(SOURCES ${SOURCES}
			NeuralDetector.cpp
			YoloDetector.cpp
			ButtonDetector.cpp
			ElevatorDoorDetector.cpp
			PersonDetector.cpp
		)

	if (NOT EXISTS "${YOLO_MODEL_PATH}")
		message("Downloading YOLOv3 model ${YOLO_MODEL_URL}")
		file(DOWNLOAD "${YOLO_MODEL_URL}" "${YOLO_MODEL_PATH}")
	endif(NOT EXISTS "${YOLO_MODEL_PATH}")

	if (NOT EXISTS "${BUTTONDETECTOR_MODEL_PATH}")
		message("Downloading YOLOv5 Elevator Button Detector model ${BUTTONDETECTOR_MODEL_URL}")
		file(DOWNLOAD "${BUTTONDETECTOR_MODEL_URL}" "${BUTTONDETECTOR_MODEL_PATH}")
	endif(NOT EXISTS "${BUTTONDETECTOR_MODEL_PATH}")

	if (NOT EXISTS "${PERSONDETECTOR_MODEL_PATH}")
		message("Downloading YOLOv5 Person Detector model ${PERSONDETECTOR_MODEL_URL}")
		file(DOWNLOAD "${PERSONDETECTOR_MODEL_URL}" "${PERSONDETECTOR_MODEL_PATH}")
	endif(NOT EXISTS "${PERSONDETECTOR_MODEL_PATH}")

	if (NOT EXISTS "${ELEVATORDOORDETECTOR_MODEL_PATH}")
		message("Downloading YOLOv5 Elevator Door Detector model ${ELEVATORDOORDETECTOR_MODEL_URL}")
		file(DOWNLOAD "${ELEVATORDOORDETECTOR_MODEL_URL}" "${ELEVATORDOORDETECTOR_MODEL_PATH}")
	endif(NOT EXISTS "${ELEVATORDOORDETECTOR_MODEL_PATH}")

endif(OpenCV_dnn_FOUND)

if (AGILEGRASP_FOUND)
	SET(SOURCES ${SOURCES}
		GraspDetector.cpp
		grasp/SmallObjectGrasp.cpp
		grasp/AgileGrasp.cpp
	)
endif(AGILEGRASP_FOUND)

#if (PCL_GPU_PEOPLE_FOUND)
if (USE_PCL_PEOPLE AND PCL_GPU_PEOPLE_FOUND)
	message(PCL_GPU_PEOPLE_FOUND)
	SET(SOURCES ${SOURCES}
		PeopleDetector.cpp
	)
endif (USE_PCL_PEOPLE AND PCL_GPU_PEOPLE_FOUND)
#endif(PCL_GPU_PEOPLE_FOUND)

if (V4R_V0_FOUND)
	SET(SOURCES ${SOURCES}
		ClusterDetectorAdvanced.cpp
	)
endif(V4R_V0_FOUND)

if (NITE_FOUND)
	SET(SOURCES ${SOURCES}
		people_openni/PeopleDetector.cpp
	)
endif(NITE_FOUND)

if (OpenPose_FOUND)
	SET(SOURCES ${SOURCES}
		OpPoseDetector.cpp
	)
endif(OpenPose_FOUND)

if (V4R_FOUND)
	SET(SOURCES ${SOURCES}
			V4RDetector.cpp
			)
endif(V4R_FOUND)

if (ZBAR_FOUND)
	SET(SOURCES ${SOURCES}
			BarCodeDetector.cpp
			)
endif(ZBAR_FOUND)



ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

# link libraries
TARGET_LINK_LIBRARIES(${PROJECT_NAME} common stm visionproc display capture imgproc shape_match stdc++fs)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${LOG4CXX_LIBRARIES} ${JSONCPP_LIBRARY})
if (AGILEGRASP_FOUND)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${AGILEGRASP_LIBRARIES})
endif (AGILEGRASP_FOUND)
if (V4R_V0_FOUND)
  TARGET_LINK_LIBRARIES(${PROJECT_NAME} v4rSurfaceSegmenter)
endif (V4R_V0_FOUND)
if (V4R_FOUND)
	TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${V4R_LIBS})
endif(V4R_FOUND)
if (OpenPose_FOUND)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenPose_LIBS})
endif (OpenPose_FOUND)
if (ZBAR_FOUND)
	TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${ZBAR_LIBRARIES})
endif (ZBAR_FOUND)

# build java wrappers via swig
ADD_SUBDIRECTORY(swig)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} detector_swig)
