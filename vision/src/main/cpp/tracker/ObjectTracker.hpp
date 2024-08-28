/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef OBJECTTRACKER_HPP
#define OBJECTTRACKER_HPP

#include <stdio.h>
#include <stdlib.h>
#include <jni.h>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

#include "common/notification/CaptureNotification.hpp"
#include "visionproc/VisionProcess.hpp"
#include "stm/MemoryObject.hpp"
#include "stm/TrackedObjects.hpp"

#ifdef USE_ROS_VISION

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
// #include "/home/dev/catkin_ws/devel/include/c_her/TrackedObjects.h"
// #include "/home/dev/catkin_ws/devel/include/c_her/TrackedObject.h"
// #include <c_her/TrackedObjects.h>
// #include <c_her/TrackedObject.h>

#endif


/*
 *  base class for all object trackers.
 */
class ObjectTracker : public VisionProcess {
public:
  typedef boost::shared_ptr<ObjectTracker> Ptr;
  typedef boost::shared_ptr<const ObjectTracker> ConstPtr;

  enum TrackerType {
    GENERIC,
    TLD,
    KCF,
    CMT,
    RELATION,
    V4R
  };

  /** @brief - ObjectTracker factory method
   * @return newly instantiated ObjectTracker::Ptr */
  static ObjectTracker::Ptr get(const TrackerType type, const long long& processorId, const int imgWidth, const int imgHeight);

  virtual ~ObjectTracker();

  virtual void init();
  virtual void cleanup();

  virtual bool addProcessingDescriptor(const std::string& descriptor, const long long& typeId);

  virtual bool removeProcessingDescriptor(const std::string& descriptor, const long long& typeId);

  bool hasIterationCompleted(const long long& typeId) const;

  long numIterationsCompleted(const long long &typeId) const;

protected:
  ObjectTracker(const long long& processorId, const int imgWidth, const int imgHeight);

  /**
   * @brief If any MemoryObjects are being tracked by tracker.
   */
  bool currentlyTrackingObjects();

  /** @brief - method to merge currently tracked object(s) with newly detected object(s).
   * calls addTrackedObject
   * @return - number of objects added to tracker */
  virtual int mergeObjects(diarc::stm::MemoryObject::VecPtr newlyDetectedObjects);
  virtual int mergeObject(diarc::stm::MemoryObject::Ptr newlyDetectedObject);

  /** @brief - remove tracked objects with confidence below a certain threshold */
  virtual void removeLowConfidenceObjects();
  
  //! Handle incoming capture data and track already tracked object in new frame.
  virtual void handleCaptureNotification(CaptureNotification::ConstPtr notification);

   //! Handle incoming detected memory object and either update already tracked
   //! objects or start tracking new objects.
  virtual void handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification);
  
  //! override automatic forwarding of frame completion notifications and set 
  //! local frame completion info
  virtual void handleFrameCompletionNotification(FrameCompletionNotification::ConstPtr notification);
  
  /**
   * Ask derived tracker if MemoryObject is an acceptable candidate
   * for tracking (e.g., large enough, has enough features, etc). Default
   * implementation returns true. Called before calling startTracking.
   * @param newMemObj
   * @return if MemoryObject can be tracked
   */
  virtual bool canTrack(const diarc::stm::MemoryObject::Ptr& newMemObj);
  /**
   * @brief - Start tracking MemoryObject. This adds the MemoryObject to 
   * the necessary TrackedObjects containers.
   * Derived class need to override this if tracked objects need to
   * be added to "sub-tracker" (e.g., cmt, tld) from base class.
   * @param newMemObj - New object to start tracking.
   */
  virtual void startTracking(const diarc::stm::MemoryObject::Ptr& newMemObj);
  /**
   * @brief Stop tracking the MemoryObject. This removes the MemoryObject from
   * the necessary TrackedObjects containers.
   * Derived classes need to override this if tracked objects need to be removed
   * from "sub-tracker" (e.g., TLD tracker). IMPORTANT: must explicitly call
   * base class method at the *beginning* of the method.
   * @param existingMemObj MemoryObject to remove
   */
  virtual void stopTracking(const diarc::stm::MemoryObject::Ptr& existingMemObj);
  /** 
   * @brief Stop tracking all MemoryObjects of specified typeId. This removes the 
   * MemoryObjects from the necessary TrackedObjects containers.
   * @param typeId SearchType ID of objects to remove
   */
  void stopTrackingType(const long long &typeId);
  /**
   * If there's a MemoryObject notification, and the MemoryObject isn't yet
   * tracked. This handles comparing the newly detected MemoryObject with
   * currently tracked objects and decides if its a re-detection of an already
   * tracked MemoryObject or if it's a new MemoryObject that should be added
   * to the tracker.
   * @param newObject
   */
  virtual void haveNewMemoryObject(diarc::stm::MemoryObject::Ptr newObject);
  /**
   * This is called if the MemoryObject is already being tracked.
   * The MemoryObject's typeIds are updated to reflect it's most recent validation
   * results based on the tracker's list of typeId and description pairs.
   * Note, that it's possible for a MemoryObject to be tracked by more than one
   * Tracker at the same time.
   */
  virtual void haveTrackedMemoryObject(diarc::stm::MemoryObject::Ptr trackedObject);
  /**
   * @brief Main entry point to tracking routine. This method MUST update Memory Object
   * confidence (usually by calling MemoryObject->update()).
   * @param frameNum
   */
  virtual void haveNewImage(CaptureNotification::ConstPtr notification) = 0;

  void sendTrackingNotificationsForAllTrackedObjects();
  void sendTrackingNotifications(diarc::stm::MemoryObject::VecPtr newTrackedObjects);
  void sendTrackingNotifications(diarc::stm::MemoryObject::Ptr newTrackedObjects);

  virtual void displayResults(CaptureData::ConstPtr capture);
  std::string getTrackingSummary();

  //threshold to quit tracking an object
  float trackingConfidenceThreshold; //between [0.0 1.0]
  float detectionConfidenceThreshold; //between [0.0 1.0]

  //per iteration info to return to java side
  bool dataProcessed_;
  bool objectsTracked_;
  int numAddedObjects_;

  //for keeping track of which frame numbers have been fully processed for a particular typeId
  mutable boost::mutex frameCompletion_mutex;
  boost::unordered_map<long long, unsigned long long> lastFrameCompletedByType;
  boost::unordered_map<long long, long> numFramesCompletedByType;

  //currently tracked objects
  diarc::stm::TrackedObjects* trackedObjects;

  // void publishTrackingResultsToROS();

private:

  //! For logging in factory method only.
  static log4cxx::LoggerPtr factoryLogger;
  bool publishToROS;

  // // ros::NodeHandle* nh;
  // ros::Publisher pub;
  // // geometry_msgs::TrackedObjects::Ptr tracked_objects_msg_;
  // // geometry_msgs::TrackedObject::Ptr tracked_object_msg_;
  // // geometry_msgs::Point::Ptr obj_location_msg_;

  // ros::NodeHandle *n_;
  // // ros::AsyncSpinner *spinner_;
};

#endif  //OBJECTTRACKER_HPP
