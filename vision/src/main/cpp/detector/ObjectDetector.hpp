/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef OBJECTDETECTOR_HPP
#define OBJECTDETECTOR_HPP

#include "stm/MemoryObject.hpp"
#include "visionproc/VisionProcess.hpp"

class ObjectDetector : public VisionProcess {
public:
  typedef boost::shared_ptr<ObjectDetector> Ptr;
  typedef boost::shared_ptr<const ObjectDetector> ConstPtr;

  enum DetectorType {
    ARUCO,
    SIFT, //old sift based method
    BLOB, //color blobs
    COLORBASED, // another color based detector (generates more granular masks instead of BBs)
    HAAR,
    YOLO,
    HOUGH,
    CLUSTER,
    CLUSTER2D,
    CLUSTER_ADVANCED,
    GRASP,
    SURFACEMARKING,
    SPATIALRELATION,
    OPENPOSE,
    V4R,
    BARCODE,
    HSVFOODDETECT,
    MOTION,
    BUTTONDETECTOR,
    ELEVATORDOORDETECTOR,
    PERSONDETECTOR
  };

  /**
   * Factory method to get any and all Detectors.
   * @param type DetectorType being requested
   * @param processorId VisionProcess ID
   * @param imgWidth image width
   * @param imgHeight image height
   * @return 
   */
  static ObjectDetector::Ptr get(const DetectorType type, const long long& processorId, const int imgWidth, const int imgHeight);

  virtual ~ObjectDetector();

protected:
  /**
   * Base class constructor that can only be called by derived types.
   * @param processorId VisionProcess ID
   * @param imgWidth image width
   * @param imgHeight image height
   */
  ObjectDetector(const long long& processorId, const int imgWidth, const int imgHeight);

  /**
   * Send detected objects notifications and update data to be returned to java 
   * side. One of the overloaded methods should be called from overwritten
   * handleNotification method.
   * @param newDetectedObjects new objects detected during iteration
   */
  void sendDetectionNotifications(ade::stm::MemoryObject::VecPtr newDetectedObjects);

  /**
   * Send detected objects notifications and update data to be returned to java 
   * side. One of the overloaded methods should be called from overwritten
   * handleNotification method.
   * @param newDetectedObject new object detected iteration
   */
  void sendDetectionNotifications(ade::stm::MemoryObject::Ptr newDetectedObject);

  //! params updated every iteration and passed back to java side
  bool dataProcessed_;
  int numObjectsDetected_;
  bool objectDetected_;

private:

  //! For logging in factory method only.
  static log4cxx::LoggerPtr factoryLogger;
};


#endif  //OBJECTDETECTOR_HPP
