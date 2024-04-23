/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "ObjectDetector.hpp"

#include "BlobDetector.hpp"
#include "ColorBasedDetector.hpp"
#include "capture/Capture.hpp"
#include "common/notification/FrameCompletionNotification.hpp"
#include "common/notification/MemoryObjectNotification.hpp"
#include "ClusterDetector.hpp"
#include "ClusterDetector2D.hpp"
#include "FaceDetector.hpp"
#include "HoughDetector.hpp"
#include "SiftDetector.hpp"
#include "SpatialRelationDetector.hpp"
#include "SurfaceMarkingDetector.hpp"
#include "FoodDetectorByHSV.hpp"
#include "MotionDetector.hpp"
#include "ArucoDetector.hpp"

#ifdef OPENCV3_DNN
#include "NeuralDetector.hpp"
#include "YoloDetector.hpp"
#endif //OPENCV_DNN

#ifdef OPENCV_DNN
#include "NeuralDetector.hpp"
#include "ButtonDetector.hpp"
#include "ElevatorDoorDetector.hpp"
#include "PersonDetector.hpp"
#include "YoloDetector.hpp"
#endif //OPENCV_DNN

#ifdef USE_AGILEGRASP
#include "GraspDetector.hpp"
#endif

#ifdef USE_V4R_V0
#include "ClusterDetectorAdvanced.hpp"
#endif //USE_V4R_V0

#ifdef USE_PCL_PEOPLE
#include "PeopleDetector.hpp"
#endif  //USE_PCL_PEOPLE

#ifdef USE_OPENPOSE
#include "OpPoseDetector.hpp"
#endif //USE_OPENPOSE

#ifdef USE_V4R

#include "V4RDetector.hpp"

#endif //USE_V4R

#ifdef USE_ZBAR
#include "BarCodeDetector.hpp"
#endif //USE_ZBAR

using namespace ade::stm;

// NOTE: not named "ade.detector.ObjectDetector" because there could be non-static
// loggers named "ade.detector.ObjectDetector"
log4cxx::LoggerPtr ObjectDetector::factoryLogger = log4cxx::Logger::getLogger("ade.detector.ObjectDetector.Factory");

//Factory method
ObjectDetector::Ptr
ObjectDetector::get(const DetectorType type, const long long &processorId, const int imgWidth, const int imgHeight) {
  switch (type) {
    case ARUCO:
      return ArucoDetector::Ptr(new ArucoDetector(processorId, imgWidth, imgHeight));
    case SIFT:
      return SiftDetector::Ptr(new SiftDetector(processorId, imgWidth, imgHeight));
    case HAAR:
      return FaceDetector::Ptr(new FaceDetector(processorId, imgWidth, imgHeight));
    case YOLO:
#if defined(OPENCV_DNN) || defined(OPENCV3_DNN)
      return YoloDetector::Ptr(new YoloDetector(processorId, imgWidth, imgHeight));
#else
    LOG4CXX_ERROR(factoryLogger, "YoloDetetor not available. Did you have the OpenCV DNN module installed?");
#endif
    break;
    case HOUGH:
      return HoughDetector::Ptr(new HoughDetector(processorId, imgWidth, imgHeight));
    case BLOB:
      return BlobDetector::Ptr(new BlobDetector(processorId, imgWidth, imgHeight));
    case COLORBASED:
      return ColorBasedDetector::Ptr(new ColorBasedDetector(processorId, imgWidth, imgHeight));
    case CLUSTER:
      return ClusterDetector::Ptr(new ClusterDetector(processorId, imgWidth, imgHeight));
    case CLUSTER_ADVANCED:
#ifdef USE_V4R_V0
      return ClusterDetectorAdvanced::Ptr(new ClusterDetectorAdvanced(processorId, imgWidth, imgHeight));
#else
    LOG4CXX_ERROR(factoryLogger, "ClusterDetectorAdvanced not available. Did you compile with V4R?");
#endif
      break;
    case CLUSTER2D:
      return ClusterDetector2D::Ptr(new ClusterDetector2D(processorId, imgWidth, imgHeight));
    case GRASP:
#ifdef USE_AGILEGRASP
      return GraspDetector::Ptr(new GraspDetector(processorId, imgWidth, imgHeight));
#else
      LOG4CXX_ERROR(factoryLogger, "GraspDetector not available. Did you compile vision with AGILE_GRASP?");
#endif
    case SURFACEMARKING:
      return SurfaceMarkingDetector::Ptr(new SurfaceMarkingDetector(processorId, imgWidth, imgHeight));
    case SPATIALRELATION:
      return SpatialRelationDetector::Ptr(new SpatialRelationDetector(processorId, imgWidth, imgHeight));
    case OPENPOSE:
#ifdef USE_OPENPOSE
      return OpPoseDetector::Ptr(new OpPoseDetector(processorId, imgWidth, imgHeight));
#else
    LOG4CXX_ERROR(factoryLogger, "OpPoseDetector not available. Did you compile with OpenPose?");
#endif
      break;
    case V4R:
#ifdef USE_V4R
      return V4RDetector::Ptr(new V4RDetector(processorId, imgWidth, imgHeight));
#else
    LOG4CXX_ERROR(factoryLogger, "V4RDetector not available. Did you compile vision with V4R?");
#endif
      break;
    case BARCODE:
#ifdef USE_ZBAR
      return BarCodeDetector::Ptr(new BarCodeDetector(processorId, imgWidth, imgHeight));
#else
    LOG4CXX_ERROR(factoryLogger, "BarCodeDetector not available. Did you compile vision with ZBAR?");
#endif
      break;
    case HSVFOODDETECT:
      return FoodDetectorByHSV::Ptr(new FoodDetectorByHSV(processorId, imgWidth, imgHeight));
    case MOTION:
      return MotionDetector::Ptr(new MotionDetector(processorId, imgWidth, imgHeight));
    case BUTTONDETECTOR:

#if defined(OPENCV_DNN)
      return ButtonDetector::Ptr(new ButtonDetector(processorId, imgWidth, imgHeight));
#else
    LOG4CXX_ERROR(factoryLogger, "ButtonDetector not available. This require OpenCV DNN module 4.5+.");
#endif
    case ELEVATORDOORDETECTOR:

#if defined(OPENCV_DNN)
      return ElevatorDoorDetector::Ptr(new ElevatorDoorDetector(processorId, imgWidth, imgHeight));
#else
    LOG4CXX_ERROR(factoryLogger, "ElevatorDoorDetector not available. This require OpenCV DNN module 4.5+.");
#endif
    case PERSONDETECTOR:

#if defined(OPENCV_DNN)
      return PersonDetector::Ptr(new PersonDetector(processorId, imgWidth, imgHeight));
#else
    LOG4CXX_ERROR(factoryLogger, "PersonDetector not available. This require OpenCV DNN module 4.5+.");
#endif
  }

  LOG4CXX_ERROR(factoryLogger, "Empty ObjectDetector::Ptr being returned.");
  return ObjectDetector::Ptr();
}

ObjectDetector::ObjectDetector(const long long &processorId, const int imgWidth, const int imgHeight)
        : VisionProcess(processorId, imgWidth, imgHeight),
          dataProcessed_(false),
          numObjectsDetected_(0),
          objectDetected_(false) {
  logger = log4cxx::Logger::getLogger("ade.detector.ObjectDetector");
}

ObjectDetector::~ObjectDetector() {
}

void ObjectDetector::sendDetectionNotifications(ade::stm::MemoryObject::VecPtr newDetectedObjects) {
  LOG4CXX_TRACE(logger,
                boost::format("[sendDetectionNotifications] num detected objects: %d.") % newDetectedObjects->size());

  numObjectsDetected_ += newDetectedObjects->size();

  //  if (newDetectedObjects->size() > 0) {
  //      MemoryObject::MemoryObjectsNotification::Ptr n(new MemoryObject::MemoryObjectsNotification(newDetectedObjects, lastProcessedFrameNum));
  //      sendNotifications(n);
  //    objectDetected_ = true;
  //  }

  MemoryObject::Vec::const_iterator iter;
  for (iter = newDetectedObjects->begin(); iter != newDetectedObjects->end(); ++iter) {
    sendDetectionNotifications(*iter);
    objectDetected_ = true;
  }
}

void ObjectDetector::sendDetectionNotifications(MemoryObject::Ptr newDetectedObject) {
  LOG4CXX_TRACE(logger, "[sendDetectionNotifications] single object detected.");
  if (newDetectedObject) {
    ++numObjectsDetected_;
    objectDetected_ = true;
    MemoryObjectNotification::Ptr n(
            new MemoryObjectNotification(shared_from_this(),
                                         newDetectedObject->getTypeId(),
                                         newDetectedObject->getDetectionMask()->getFrameNumber(),
                                         newDetectedObject));
    sendNotifications(n);
  }
}