/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef VISIONPROCESS_HPP
#define VISIONPROCESS_HPP

#include <boost/enable_shared_from_this.hpp>
#include <boost/format.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <tr1/unordered_map> //<hash_map>
#include <tr1/unordered_set> //<hash_set>
//#include <boost/unordered_map.hpp>
//#include <boost/unordered_set.hpp>
#include <list>
#include <log4cxx/logger.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
//#include <bits/unordered_map.h>

#include "common/fol/Predicate.hpp"
#include "common/fol/PredicateHelper.hpp"
#include "common/notification/CaptureNotification.hpp"
#include "common/notification/FrameCompletionNotification.hpp"
#include "common/notification/LearningNotification.hpp"
#include "common/notification/MemoryObjectNotification.hpp"
#include "common/notification/MemoryObjectsNotification.hpp"
#include "common/notification/MotionNotification.hpp"
#include "common/notification/Notification.hpp"
#include "common/notification/PlaneNotification.hpp"
#include "common/notification/SaliencyNotification.hpp"
#include "common/notification/SiftNotification.hpp"
#include "tools/syncedqueue.hpp"

/**
 * This is the base class for the ObjectDetector, ObjectTracker, and
 * ImageProcessor classes.
 *
 * @author Evan Krause
 */
class VisionProcess : public boost::enable_shared_from_this<VisionProcess> {
public:
  typedef boost::shared_ptr<VisionProcess> Ptr;
  typedef boost::shared_ptr<const VisionProcess> ConstPtr;

  //for descriptor containers (way less ugly if there were immutable containers in c++)
  typedef std::tr1::unordered_map<long long, PredicateHelper::Set> DescriptorsByType;
  typedef boost::shared_ptr<DescriptorsByType> DescriptorsByTypePtr;
  typedef boost::shared_ptr<const DescriptorsByType> DescriptorsByTypeConstPtr;
  typedef std::tr1::unordered_map<PredicateHelper, std::tr1::unordered_set<long long>, std::tr1::hash<PredicateHelper>, predhelper_equal_to> TypesByDescriptor;
  typedef boost::shared_ptr<TypesByDescriptor> TypesByDescriptorPtr;
  typedef boost::shared_ptr<const TypesByDescriptor> TypesByDescriptorConstPtr;

  virtual ~VisionProcess();

  virtual void perform();

  /**
   * For any initialization that needs to be done outside of the constructor.
   */
  virtual void init();

  /**
   * For any cleanup that should be done before the destructor is called or for
   * any cleanup needed between stop and start cycles.
   * Also useful for things that need to be cleaned up in the same thread they
   * were created (eg, opengl).
   */
  virtual void cleanup();

  /**
   * Load configuration info from file (e.g., load file mapping "color" to RGB ranges)
   * @param config
   */
  virtual void loadConfig(const std::string& config);


  /**
   * Save configuration info to file (e.g., save file mapping "color" to RGB ranges)
   * @param config
   */
  virtual void saveConfig(const std::string& config);

  /**
   * Set the status of this vision processor. Used to ignore notifications when 
   * not running. Also clears notifications queue when setting to false.
   * @param running
   */
  void setRunningStatus(bool running);

  /**
   * Add a VisionProcess to the list of processes that should be notified
   * by this VisionProcess.
   * @param notifyTarget - VisionProcess that is registering for notifications
   */
  void registerForNotification(VisionProcess::Ptr notifyTarget, const long long& typeId);

  /**
   * Remove the already registered VisionProcess from the list of processes
   * that should be notified by this VisionProcess.
   * @param notifyTarget - VisionProcess that is  unregistering for notifications
   */
  void unregisterForNotification(VisionProcess::Ptr notifyTarget, const long long& typeId);

  /**
   * Register to be notified by capture processor. Separate method needed
   * because capture class does not derive from VisionProcess class.
   */
  virtual void registerForCaptureNotification();

  /**
   * Un-register to be notified by capture processor. Separate method needed
   * because capture class does not derive from VisionProcess class.
   */
  virtual void unregisterForCaptureNotification();


  /**
   * Receive notification from a VisionProcess.
   * @param n - Notification being received
   */
  void notify(Notification::Ptr n);

  /**
   * Add descriptors to process (e.g., add "red" to color processor)
   * @param descriptor
   * @param typeId
   * @return 
   */
  virtual bool addProcessingDescriptor(const std::string& descriptor, const long long& typeId);

  virtual bool addProcessingDescriptor(const ade::common::fol::Predicate& predicate, const long long& typeId);
  void printPredicate();

  /**
   * Remove descriptors from process (e.g., add "red" to color processor).
   * @param descriptor
   * @param typeId
   * @return 
   */
  virtual bool removeProcessingDescriptor(const std::string& descriptor, const long long& typeId);

  /**
  * If this vision processor has successfully learned (by way of a learning notification).
  * @return
  */
  virtual bool hasLearned() const;

  /**
  * It's often the case that a single vision processor will need to learn several times, so this
  * exposes a way to reset the underlying mechanism (up to derived class) responsible for hasLearned().
  */
  virtual void resetHasLearned();

  //display methods - thread safe
  //overriding methods need to ensure same level of thread-safety
  virtual void turnDisplayOn(const std::string& windowName);
  virtual void turnDisplayOff();
  bool getDisplayFlag() const;
  std::string getDisplayName() const;

  void setIncrementalProcessing(bool incrementalProcessing_);
  bool getIncrementalProcessing() const;
  void setSerialProcessing(bool serialProcessing_);
  bool getSerialProcessing() const;

  long long getProcessorId() const;

  /**
  * Get all vision processor IDs registered to be notified as part of typeId (visual search).
  * @param typeId
  * @return
  */
  std::vector <long long> getRegisteredProcessorIds(const long long &typeId);

  /**
   * This interrupts a wait for notifications. This is necessary to stop threads
   * that are waiting for notifications that may never come.
   */
  void interruptNotificationWait();

  //! get descriptors (hash keys) and corresponding type ids (hash values)
  TypesByDescriptorConstPtr getDescriptors() const;

  //! get type ids (hash keys) and corresponding descriptors (hash values)
  DescriptorsByTypeConstPtr getTypes() const;

  std::string visionProcessName;

protected:
  VisionProcess(const long long& processorId, const unsigned int imgwidth, const unsigned int imgheight);

  /**
   * Handles all incoming notification, casts them to the appropriate notification
   * type and calls the appropriate handle___Notification method which must be
   * implemented by derived classes.
   * @param n incoming notification
   * @return if any data was processed
   */
  virtual bool handleNotification(Notification::Ptr n);

  /**
   * All notification handling methods that can be implemented by derived classes.
   * Called automatically by handleNotification method when incoming notifications
   * are received.
   * @param notification
   */
  virtual void handleCaptureNotification(CaptureNotification::ConstPtr notification);
  virtual void handleFrameCompletionNotification(FrameCompletionNotification::ConstPtr notification);
  virtual void handleLearningNotification(LearningNotification::ConstPtr notification);
  virtual void handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification);
  virtual void handleMemoryObjectsNotification(MemoryObjectsNotification::ConstPtr notification);
  virtual void handleMotionNotification(MotionNotification::ConstPtr notification);
  virtual void handlePlaneNotification(PlaneNotification::ConstPtr notification);
  virtual void handleSaliencyNotification(SaliencyNotification::ConstPtr notification);
  virtual void handleSiftNotification(SiftNotification::ConstPtr notification);

  /**
   * Notify target (this) processor of successful registration with source (notifying) processor
   * @param notifyingSource - processor that will send notifications to this processor
   */
  virtual void notifyOfRegistration(VisionProcess::Ptr notifyingSource);

  /**
   * Notify target (this) processor of successful un-registration with source (notifying) processor
   * @param notifyingSource - processor that was sending notifications to this processor
   */
  virtual void notifyOfUnregistration(VisionProcess::Ptr notifyingSource);

  /**
   * Send notification to all registered processors.
   */
  void sendNotifications(Notification::Ptr n);

  /**
   * Get all currently pending notifications, possibly none, and return immediately.
   */
  void getPendingNotifications(std::list<Notification::Ptr> &pending);

  /**
   * Wait until there is at least one pending notification, then return with all
   * pending notifications.
   */
  void waitPendingNotifications(std::list<Notification::Ptr> &pending);

  /**
   * Wait until there is a pending notification and return that notification.
   */
  void waitPendingNotification(Notification::Ptr &pending);

  /**
   * Clear all currently pending notifications, possibly none, and return immediately.
   */
  void clearPendingNotifications();

  /**
   * Called after each handleNotification call if ignoreOldNotifications flag is true
   * to remove older notifications in the pending queue. The exact method of removal
   * is defined based on the notification type. This can be overridden to change the
   * behavior of removal.
   * @param pending - list of pending notifications to be processed this iteration
   * @param notification - notification just handled
   */
  virtual void removeOldNotifications(std::list<Notification::Ptr> &pending, Notification::Ptr notification);

  /**
   * Remove all notifications of the given type from the given list of notifications.
   * Note that this is different from clearPendingNotifications() which simply clears
   * the processor's whole internal notification queue irrespective of type.
   */
  void removeNotificationsOfType(std::list<Notification::Ptr> &pending, Notification::Type type);

  /**
   * Remove the notifications of the given type from the given list of notifications that
   * are older than the given frame number.
   * Note that this is different from clearPendingNotifications() which simply clears
   * the processor's whole internal notification queue irrespective of type.
   */
  void removeNotificationsOfType(std::list<Notification::Ptr> &pending, Notification::Type type,
          unsigned long int frameNum);

  /**
   * @brief display MemoryObject and its validation results (i.e., properties)
   * @param mo - MemoryObject to display
   * @param combine - if validations of same type and descriptor should be combined
   * into the same image
   */
  void displayMemoryObject(ade::stm::MemoryObject::Ptr& mo, const std::string& variableName, bool binaryMask = false, bool combine = true);

  void displayMemoryObjectValidation(const ade::stm::MemoryObject::ConstPtr& mo, bool combine = true);
  
  void displayMemoryObjectRelations(const ade::stm::MemoryObject::ConstPtr& mo);

  const long long processorId;
  const int img_width;
  const int img_height;
  cv::Mat displayFrame; //use to draw processing results

  //! processors that that will be notified (hashed by typeId)
  boost::unordered_map<long long, boost::unordered_set<VisionProcess::Ptr> > registeredProcessors;
  mutable boost::mutex reg_procs_mutex;
  syncedqueue<Notification::Ptr> notifications;

  //protected, so that overriding methods turnDisplayOn and turnDisplayOff
  //! can use this to ensure thread-safety
  mutable boost::mutex display_mutex;

  //! to ensure public methods are locked -- TODO: still needed?
  boost::mutex perform_mutex;

  /**
   * If set to true (the default), then whenever a notification of type T is found
   * in the list of pending notifications, all older notifications of the same type
   * are ignored. Useful if your notifier (e.g. capture) is faster than your processing.
   * If you do not want to miss any notification, then set to false.
   */
  bool ignoreOlderNotifications;

  log4cxx::LoggerPtr logger;

private:
  //TEST
  std::vector<ade::common::fol::Predicate::ConstPtr> tmpPredicates;

  //disallow copying and assignment
  VisionProcess(const VisionProcess&);
  VisionProcess& operator=(const VisionProcess&);

  void printRegistration();
  void printUsedProcessors();

  //if process is running. used to ignore notifications when not running.
  bool isRunning;

  //descriptor data and mutex - private, so derived classes don't have to worry about locking, use get methods
  DescriptorsByTypePtr descriptorsByType;
  TypesByDescriptorPtr typesByDescriptor;
  mutable boost::mutex descriptor_mutex;

  //display data - private, so derived classes don't have to worry about locking, use get methods
  bool displayFlag;
  std::string displayName;

  bool incrementalProcessing;
  bool serialProcessing;

  mutable boost::mutex incrementalProcessing_mutex;
  mutable boost::mutex serialProcessing_mutex;
};

#endif  //VISIONPROCESS_HPP
