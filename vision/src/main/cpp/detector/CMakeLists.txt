PROJECT(detector)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

SET(SOURCES
	ArucoDetector.cpp
	BlobDetector.cpp
	ClusterDetector.cpp
	ClusterDetector2D.cpp
	ColorBasedDetector.cpp
	FaceDetector.cpp
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

SET(YOLO3_MODEL_PATH "${MODEL_DIR}yolov3/yolov3-tiny.weights")
SET(YOLO3_MODEL_URL "https://github.com/hrilabtufts/vision_models/raw/main/yolo/yolov3-tiny.weights")

SET(YOLO5_MODEL_PATH "${MODEL_DIR}yolov5/yolov5m.onnx")
SET(YOLO5_MODEL_URL "https://github.com/hrilabtufts/vision_models/raw/main/yolov5/yolov5m.onnx")

SET(YOLO8_MODEL_PATH "${MODEL_DIR}yolov8/yolov8n.onnx")
SET(YOLO8_MODEL_URL "https://github.com/hrilabtufts/vision_models/raw/main/yolov8/yolov8n.onnx")

SET(BUTTONDETECTOR_MODEL_PATH "${MODEL_DIR}/buttondetector/buttonModelS320.onnx")
SET(BUTTONDETECTOR_MODEL_URL "https://github.com/hrilabtufts/vision_models/raw/main/buttondetector/buttonModelS320.onnx")

SET(PERSONDETECTOR_MODEL_PATH "${MODEL_DIR}/persondetector/crowdhuman_yolov5m.onnx")
SET(PERSONDETECTOR_MODEL_URL "https://github.com/hrilabtufts/vision_models/raw/main/persondetector/crowdhuman_yolov5m.onnx")

SET(ELEVATORDOORDETECTOR_MODEL_PATH "${MODEL_DIR}/elevatordoordetector/elevatordoordetectorS320.onnx")
SET(ELEVATORDOORDETECTOR_MODEL_URL "https://github.com/hrilabtufts/vision_models/raw/main/elevatordoordetector/elevatordoordetectorS320.onnx")

SET(DOOR_YOLO3_MODEL_PATH "${MODEL_DIR}door/yolov3-door.weights")
SET(DOOR_YOLO3_CFG_PATH "${MODEL_DIR}door/yolov3-door.cfg")
SET(DOOR_YOLO3_MODEL_URL "https://github.com/hrilabtufts/vision_models/raw/main/door/yolov3-door.weights")
SET(DOOR_YOLO3_CFG_URL "https://github.com/hrilabtufts/vision_models/raw/main/door/yolov3-door.cfg")

# optional sources

if (pybind11_FOUND)
	SET(SOURCES ${SOURCES}
			FastSamDetector.cpp
			OwlV2Detector.cpp
	)
endif (pybind11_FOUND)

if (OpenCV3_dnn_FOUND OR OpenCV_dnn_FOUND)
	SET(SOURCES ${SOURCES}
			NeuralDetector.cpp
		)

	if (NOT EXISTS "${YOLO3_MODEL_PATH}")
		message("Downloading YOLOv3 model ${YOLO3_MODEL_URL}")
		file(DOWNLOAD "${YOLO3_MODEL_URL}" "${YOLO3_MODEL_PATH}")
	endif(NOT EXISTS "${YOLO3_MODEL_PATH}")

	if (NOT EXISTS "${YOLO5_MODEL_PATH}")
		message("Downloading YOLOv5 model ${YOLO5_MODEL_URL}")
		file(DOWNLOAD "${YOLO5_MODEL_URL}" "${YOLO5_MODEL_PATH}")
    endif(NOT EXISTS "${YOLO5_MODEL_PATH}")

    if (NOT EXISTS "${YOLO8_MODEL_PATH}")
        message("Downloading YOLOv5 model ${YOLO8_MODEL_URL}")
        file(DOWNLOAD "${YOLO8_MODEL_URL}" "${YOLO8_MODEL_PATH}")
    endif(NOT EXISTS "${YOLO8_MODEL_PATH}")
endif (OpenCV3_dnn_FOUND OR OpenCV_dnn_FOUND)

if (OpenCV_dnn_FOUND)
	SET(SOURCES ${SOURCES}
			ButtonDetector.cpp
			PersonDetector.cpp
		)

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

    if (NOT EXISTS "${DOOR_YOLO3_MODEL_PATH}")
        message("Downloading YOLOv3 Door Detector model ${DOOR_YOLO3_MODEL_URL}")
        file(DOWNLOAD "${DOOR_YOLO3_MODEL_URL}" "${DOOR_YOLO3_MODEL_PATH}")
        file(DOWNLOAD "${DOOR_YOLO3_CFG_URL}" "${DOOR_YOLO3_CFG_PATH}")
    endif(NOT EXISTS "${DOOR_YOLO3_MODEL_PATH}")

endif(OpenCV_dnn_FOUND)

if (V4R_V0_FOUND)
	SET(SOURCES ${SOURCES}
		ClusterDetectorAdvanced.cpp
	)
endif(V4R_V0_FOUND)

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
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${LOG4CXX_LIBRARIES} ${JSONCPP_LIBRARY} Boost::thread)

if (pybind11_FOUND)
	TARGET_LINK_LIBRARIES(${PROJECT_NAME} pyvis)
	TARGET_LINK_LIBRARIES(${PROJECT_NAME} pybind11::embed)
	TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${Python3_LIBRARIES})
endif (pybind11_FOUND)
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
