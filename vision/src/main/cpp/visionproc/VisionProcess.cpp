/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "capture/Capture.hpp"
#include "common/fol/Variable.hpp"
#include "display/Display.hpp"
#include "display/util/DisplayUtilities.hpp"
#include "VisionProcess.hpp"

#include <boost/smart_ptr/make_shared.hpp>

using namespace ade::common::fol;
using namespace ade::stm;

VisionProcess::VisionProcess(const long long &processor_id, const unsigned int imgWidth, const unsigned int imgHeight)
        : visionProcessName("Not Set"),
          processorId(processor_id),
          img_width(imgWidth),
          img_height(imgHeight),
          displayFrame(),
          reg_procs_mutex(),
          registeredProcessors(),
          notifications(),
          display_mutex(),
          logger(log4cxx::Logger::getLogger("ade.visionproc.VisionProcess")),
          isRunning(false),
          descriptorsByType(new DescriptorsByType()),
          typesByDescriptor(new TypesByDescriptor()),
          descriptor_mutex(),
          displayFlag(false),
          displayName("Not Set"),
          incrementalProcessing(true),
          serialProcessing(false),
          incrementalProcessing_mutex(),
          serialProcessing_mutex(),
          ignoreOlderNotifications(true),
          tmpPredicates() {
}

VisionProcess::~VisionProcess() {
}

void VisionProcess::perform() {
  // wait for notifications
  std::list<Notification::Ptr> pending;
  waitPendingNotifications(pending);

  // log all pending notifications
  if (logger->isTraceEnabled()) {
    LOG4CXX_DEBUG(logger, boost::format("%d PENDING NOTIFICATIONS:") % pending.size());
    std::list<Notification::Ptr>::const_iterator pending_itr;
    for (pending_itr = pending.begin(); pending_itr != pending.end(); ++pending_itr) {
      VisionNotification::Ptr vn = boost::dynamic_pointer_cast<VisionNotification>(*pending_itr);
      if (vn) {
        if (vn->notifier) {
          LOG4CXX_TRACE(logger, boost::format("Type: %d. FrameNum: %llu. ProcId: %lld.")
                                % vn->type % vn->frameNumber % vn->notifier->getProcessorId());
        } else {
          // capture processor doesn't inherit from VisionProcess
          LOG4CXX_TRACE(logger, boost::format("Type: %d. FrameNum: %llu. ProcId: Capture.")
                                % vn->type % vn->frameNumber);
        }
      } else {
        // capture notification -- currently only notification that doesn't inherit from VisionNotification
        LOG4CXX_TRACE(logger,
                      boost::format("Type: %d. FrameNum: %llu.") % (*pending_itr)->type % (*pending_itr)->frameNumber);
      }
    }
  }

  // save all frame completion notifications to process after all other notifications
  // nee to do this because of the order of pending notifications (i.e., newest first
  // which means that FCNs appear before other notifications for that frame number)
  //std::list<Notification::Ptr> pendingFCNs;
  Notification::Ptr onDeckFCN;

  // lock
  boost::lock_guard<boost::mutex> lock(perform_mutex);

  // handle all the types of notifications we are interested in and discard the rest
  while (!pending.empty()) {
    LOG4CXX_TRACE(logger, boost::format("%d PENDING NOTIFICATIONS LEFT:") % pending.size());
    Notification::Ptr n = pending.back();
    pending.pop_back();

    // used to notify entire pipeline when processing of an entire frame has completed
    // this is needed to know when a full detection -> validation -> tracking cycle has finished
    if (n->type == Notification::FRAME_COMPLETION) {
      // send the previously seen frame completion notification
      // 
      if (onDeckFCN) {
        handleNotification(onDeckFCN);
      }
      onDeckFCN = n;
      //pendingFCNs.push_back(n);
    } else {
      //handle all other relevant notification types
      handleNotification(n);
    }

    // remove old notifications in the pending queue (removal process depends on notification type)
    if (ignoreOlderNotifications) {
      //TODO: get rid of notifications older than onDeckFCN ??
      removeOldNotifications(pending, n);
    }
  }

  // finally process all frame completion notifications
  //  LOG4CXX_DEBUG(logger, boost::format("%d PENDING FRAME_COMPLETION NOTIFICATIONS:") % pendingFCNs.size());
  //  while (!pendingFCNs.empty()) {
  //    Notification::Ptr n = pendingFCNs.front();
  //    pendingFCNs.pop_front();
  //    handleNotification(n);
  //  }
  if (onDeckFCN) {
    handleNotification(onDeckFCN);
  }
}

void VisionProcess::removeOldNotifications(std::list<Notification::Ptr> &pending, Notification::Ptr notification) {
  switch (notification->type) {
    case Notification::CAPTURE:
    case Notification::SALIENCY:
    case Notification::SCENE_CHANGE:
    case Notification::SIFT:
      removeNotificationsOfType(pending, notification->type);
      break;
    case Notification::PLANE:
      removeNotificationsOfType(pending, notification->type, notification->frameNumber);
      break;
    case Notification::FRAME_COMPLETION: {
      FrameCompletionNotification::Ptr fcn = boost::dynamic_pointer_cast<FrameCompletionNotification>(notification);
      for (std::list<Notification::Ptr>::iterator i = pending.begin(); i != pending.end();) {
        if ((*i)->type == fcn->type) {
          FrameCompletionNotification::Ptr i_fcn = boost::dynamic_pointer_cast<FrameCompletionNotification>(*i);
          if (i_fcn->typeIds == fcn->typeIds && i_fcn->frameNumber <= fcn->frameNumber) {
            i = pending.erase(i);
          } else {
            ++i;
          }
        } else {
          ++i;
        }
      }
    }
      break;
    case Notification::MEMORY_OBJECT:
      removeNotificationsOfType(pending, notification->type, notification->frameNumber); //EAK: do we want to do this?
    case Notification::MEMORY_OBJECTS:
      break;
  }
  //case Notification::MEMORY_OBJECT:
  //TODO: potentially remove notifications older than the last FRAME_COMPLETION notification (per typeId)??
  //removeNotificationsOfType(pending, n->type, n->frameNum); //EAK: do we want to do this?
}

void VisionProcess::init() {
}

void VisionProcess::cleanup() {
}

void VisionProcess::setRunningStatus(bool running) {
  LOG4CXX_TRACE(logger, boost::format("[setRunningStatus] to %s.") % (running ? "true" : "false"));
  isRunning = running;
  if (!isRunning) {
    clearPendingNotifications();
  }
}

void VisionProcess::setIncrementalProcessing(bool incrementalProcessing_) {
  boost::unique_lock<boost::mutex> lock(incrementalProcessing_mutex);
  incrementalProcessing = incrementalProcessing_;
}

bool VisionProcess::getIncrementalProcessing() const {
  boost::unique_lock<boost::mutex> lock(incrementalProcessing_mutex);
  return incrementalProcessing;
}

void VisionProcess::setSerialProcessing(bool serialProcessing_) {
  boost::unique_lock<boost::mutex> lock(serialProcessing_mutex);
  serialProcessing = serialProcessing_;
}

bool VisionProcess::getSerialProcessing() const {
  boost::unique_lock<boost::mutex> lock(serialProcessing_mutex);
  return serialProcessing;
}

long long VisionProcess::getProcessorId() const {
  return processorId;
}

std::vector<long long> VisionProcess::getRegisteredProcessorIds(const long long &typeId) {
  std::vector<long long> processorId_list;
  boost::unordered_map<long long, boost::unordered_set<VisionProcess::Ptr> >::const_iterator regProcs_itr = registeredProcessors.find(typeId);
  if (regProcs_itr != registeredProcessors.end()) {
    boost::unordered_set<VisionProcess::Ptr>::const_iterator regProcsByType_itr;
    for (regProcsByType_itr = regProcs_itr->second.begin(); regProcsByType_itr != regProcs_itr->second.end(); ++regProcsByType_itr) {
      processorId_list.push_back((*regProcsByType_itr)->processorId);
    }
  }
  return processorId_list;
}

void VisionProcess::turnDisplayOn(const std::string &windowName) {
  boost::unique_lock<boost::mutex> lock(display_mutex);
  ade::Display::destroyWindowIfItExists(displayName);
  displayFlag = true;
  displayName = windowName;
  ade::Display::createWindowIfDoesNotExist(displayName);
}

void VisionProcess::turnDisplayOff() {
  boost::unique_lock<boost::mutex> lock(display_mutex);
  displayFlag = false;
  ade::Display::destroyWindowIfItExists(displayName);
}

bool VisionProcess::getDisplayFlag() const {
  boost::unique_lock<boost::mutex> lock(display_mutex);
  return displayFlag;
}

std::string VisionProcess::getDisplayName() const {
  boost::unique_lock<boost::mutex> lock(display_mutex);
  return displayName;
}

void VisionProcess::loadConfig(const std::string &config) {
  LOG4CXX_WARN(logger, boost::format("[loadConfig] not implemented for derived class: %s.") % config);
}


void VisionProcess::saveConfig(const std::string &config) {
  LOG4CXX_WARN(logger, boost::format("[saveConfig] not implemented for derived class: %s.") % config);
}

bool VisionProcess::handleNotification(Notification::Ptr n) {
  if (n->type == Notification::CAPTURE) {
    CaptureNotification::Ptr cn = boost::dynamic_pointer_cast<CaptureNotification>(n);
    if (!cn) {
      LOG4CXX_ERROR(logger, "[handleNotification] bad cast.");
      return false;
    }
    handleCaptureNotification(cn);
  } else if (n->type == Notification::FRAME_COMPLETION) {
    FrameCompletionNotification::Ptr fcn = boost::dynamic_pointer_cast<FrameCompletionNotification>(n);
    if (!fcn) {
      LOG4CXX_ERROR(logger, "[handleNotification] bad cast.");
      return false;
    }
    handleFrameCompletionNotification(fcn);
  } else if (n->type == Notification::MEMORY_OBJECT) {
    MemoryObjectNotification::Ptr mon = boost::dynamic_pointer_cast<MemoryObjectNotification>(n);
    if (!mon) {
      LOG4CXX_ERROR(logger, "[handleNotification] bad cast.");
      return false;
    }
    // don't care about MemoryObjects of a typeId that this vision proc isn't processing
    //    DescriptorsByTypeConstPtr types = getTypes();
    //    if (mon->object && types->find(mon->object->getTypeId()) == types->end()) {
    //      LOG4CXX_DEBUG(logger, boost::format("[handleNotification] ignoring MEMORY_OBJECT notification of typeId: %lld.") % mon->object->getTypeId());
    //      return false;
    //    }
    handleMemoryObjectNotification(mon);
  } else if (n->type == Notification::SALIENCY) {
    SaliencyNotification::Ptr sn = boost::dynamic_pointer_cast<SaliencyNotification>(n);
    if (!sn) {
      LOG4CXX_ERROR(logger, "[handleNotification] bad cast.");
      return false;
    }
    handleSaliencyNotification(sn);
  } else if (n->type == Notification::PLANE) {
    PlaneNotification::Ptr pn = boost::dynamic_pointer_cast<PlaneNotification>(n);
    if (!pn) {
      LOG4CXX_ERROR(logger, "[handleNotification] bad cast.");
      return false;
    }
    handlePlaneNotification(pn);
  } else if (n->type == Notification::SIFT) {
    SiftNotification::Ptr sn = boost::dynamic_pointer_cast<SiftNotification>(n);
    if (!sn) {
      LOG4CXX_ERROR(logger, "[handleNotification] bad cast.");
      return false;
    }
    handleSiftNotification(sn);
  } else if (n->type == Notification::LEARNING) {
    LearningNotification::Ptr ln = boost::dynamic_pointer_cast<LearningNotification>(n);
    if (!ln) {
      LOG4CXX_ERROR(logger, "[handleNotification] bad cast.");
      return false;
    }
    handleLearningNotification(ln);
  } else if (n->type == Notification::MOTION) {
    MotionNotification::Ptr mn = boost::dynamic_pointer_cast<MotionNotification>(n);
    if (!mn) {
      LOG4CXX_ERROR(logger, "[handleNotification] bad cast.");
      return false;
    }
    handleMotionNotification(mn);
  } else {
    LOG4CXX_WARN(logger, boost::format("[handleNotification] can't handle notification of type: %d.") % n->type);
    return false;
  }

  return true;
}

void VisionProcess::handleCaptureNotification(CaptureNotification::ConstPtr notification) {
  LOG4CXX_WARN(logger, "[handleCaptureNotification] not handled.");
}

void VisionProcess::handleFrameCompletionNotification(FrameCompletionNotification::ConstPtr notification) {
  DescriptorsByType::const_iterator types_itr;
  DescriptorsByTypeConstPtr types = getTypes(); //using getter so we don't have to lock

  if (logger->isTraceEnabled()) {
    boost::unordered_set<long long>::const_iterator notification_types_itr;
    LOG4CXX_TRACE(logger, "[handleFrameCompletionNotification] typeIds:");
    for (notification_types_itr = notification->typeIds.begin();
         notification_types_itr != notification->typeIds.end(); ++notification_types_itr) {
      LOG4CXX_TRACE(logger, boost::format("[handleFrameCompletionNotification] typeId: %lld") % (*notification_types_itr));
    }
  }

  // if notification isn't for a particular search typeId (e.g., capture, sift image proc, etc)
  // TODO: figure this out!!
  //if (notification->typeId == -1) {
  if (notification->typeIds.find(-1) != notification->typeIds.end()) {
    LOG4CXX_TRACE(logger, "[handleFrameCompletionNotification] 1.");
    if (types->empty()) {
      LOG4CXX_TRACE(logger, "[handleFrameCompletionNotification] 2.");
      // send notification that's not for a particular search typeId
      FrameCompletionNotification::Ptr new_fcn(
              new FrameCompletionNotification(shared_from_this(), -1L, notification->frameNumber));
      sendNotifications(new_fcn);
    } else {
      LOG4CXX_TRACE(logger, "[handleFrameCompletionNotification] 3.");
      // send notifications for all relevant search typeIds
      for (types_itr = types->begin(); types_itr != types->end(); ++types_itr) {
        LOG4CXX_TRACE(logger, "[handleFrameCompletionNotification] 4.");
        FrameCompletionNotification::Ptr new_fcn(
                new FrameCompletionNotification(shared_from_this(), types_itr->first, notification->frameNumber));
        sendNotifications(new_fcn);
      }
    }
  } else {
    LOG4CXX_TRACE(logger, "[handleFrameCompletionNotification] 5.");
    // pass along notification for that particular search typeId (only if this VP is processing that typeId)
    // TODO: figure this out!!
    for (types_itr = types->begin(); types_itr != types->end(); ++types_itr) {
      //if (types->find(notification->typeId) != types->end()) {
      if (notification->typeIds.find(types_itr->first) != notification->typeIds.end()) {
        LOG4CXX_TRACE(logger, "[handleFrameCompletionNotification] 6.");
        FrameCompletionNotification::Ptr new_fcn(
                new FrameCompletionNotification(shared_from_this(), types_itr->first, notification->frameNumber));
        sendNotifications(new_fcn);
      }
    }
  }
}

void VisionProcess::handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification) {
  LOG4CXX_WARN(logger, "[handleMemoryObjectNotification] not handled.");
}

void VisionProcess::handleMemoryObjectsNotification(MemoryObjectsNotification::ConstPtr notification) {
  LOG4CXX_WARN(logger, "[handleMemoryObjectsNotification] not handled.");
}

void VisionProcess::handleSaliencyNotification(SaliencyNotification::ConstPtr notification) {
  LOG4CXX_WARN(logger, "[handleSaliencyNotification] not handled.");
}

void VisionProcess::handlePlaneNotification(PlaneNotification::ConstPtr notification) {
  LOG4CXX_WARN(logger, "[handlePlaneNotification] not handled.");
}

void VisionProcess::handleSiftNotification(SiftNotification::ConstPtr notification) {
  LOG4CXX_WARN(logger, "[handleSiftNotification] not handled.");
}

void VisionProcess::handleLearningNotification(LearningNotification::ConstPtr notification) {
  LOG4CXX_WARN(logger, "[handleLearningNotification] not handled.");
}

void VisionProcess::handleMotionNotification(MotionNotification::ConstPtr notification) {
  LOG4CXX_WARN(logger, "[handleMotionNotification] not handled.");
}

void VisionProcess::printRegistration() {
  boost::lock_guard<boost::mutex> lock(reg_procs_mutex);
  LOG4CXX_INFO(logger, "Printing Registered Processors...");
  boost::unordered_map<long long, boost::unordered_set<VisionProcess::Ptr> >::const_iterator typeId_itr;
  boost::unordered_set<VisionProcess::Ptr>::const_iterator visionProc_itr;
  for (typeId_itr = registeredProcessors.begin(); typeId_itr != registeredProcessors.end(); ++typeId_itr) {
    LOG4CXX_INFO(logger, boost::format("Registered Processor(s) for typeId: %lld") % typeId_itr->first);
    for (visionProc_itr = typeId_itr->second.begin(); visionProc_itr != typeId_itr->second.end(); ++visionProc_itr) {
      LOG4CXX_INFO(logger, boost::format("\t\tProcessor: %s") % (*visionProc_itr)->visionProcessName);
    }
  }
}

void VisionProcess::registerForNotification(VisionProcess::Ptr notifyTarget, const long long &typeId) {
  boost::lock_guard<boost::mutex> lock(reg_procs_mutex);

  // add processor to registeredProcessors
  boost::unordered_map<long long, boost::unordered_set<VisionProcess::Ptr> >::iterator visionProc_itr;
  visionProc_itr = registeredProcessors.find(typeId);
  if (visionProc_itr == registeredProcessors.end()) {
    boost::unordered_set<VisionProcess::Ptr> visionProcSet;
    visionProcSet.insert(notifyTarget);
    registeredProcessors[typeId] = visionProcSet;
  } else {
    visionProc_itr->second.insert(notifyTarget);
  }

  //notify target processor of successful registration
  notifyTarget->notifyOfRegistration(shared_from_this());
}

void VisionProcess::unregisterForNotification(VisionProcess::Ptr notifyTarget, const long long &typeId) {
  boost::lock_guard<boost::mutex> lock(reg_procs_mutex);

  boost::unordered_map<long long, boost::unordered_set<VisionProcess::Ptr> >::iterator visionProc_itr;
  visionProc_itr = registeredProcessors.find(typeId);
  if (visionProc_itr != registeredProcessors.end()) {
    visionProc_itr->second.erase(notifyTarget);

    //notify target processor of successful unregistration
    notifyTarget->notifyOfUnregistration(shared_from_this());

    // clean up empty container for typeId
    if (visionProc_itr->second.empty()) {
      registeredProcessors.erase(visionProc_itr);
    }
  }
}

void VisionProcess::registerForCaptureNotification() {
  ade::capture::Capture::registerForNotification(shared_from_this());
}

void VisionProcess::unregisterForCaptureNotification() {
  ade::capture::Capture::unregisterForNotification(shared_from_this());
}

void VisionProcess::notifyOfRegistration(VisionProcess::Ptr notifyingSource) {
  //default implementation does nothing
}

void VisionProcess::notifyOfUnregistration(VisionProcess::Ptr notifyingSource) {
  //default implementation does nothing
}

void VisionProcess::sendNotifications(Notification::Ptr n) {
  boost::lock_guard<boost::mutex> lock(reg_procs_mutex);

  // filter all registered processors based on typeIds
  boost::unordered_set<VisionProcess::Ptr> toNotify;
  boost::unordered_map<long long, boost::unordered_set<VisionProcess::Ptr> >::const_iterator visionProc_itr;
  boost::unordered_set<long long>::const_iterator typeId_itr;
  if (n->typeIds.empty() || n->typeIds.find(-1) != n->typeIds.end()) {
    // if no typeIds or contains -1, collect all registered procs
    for (visionProc_itr = registeredProcessors.begin();
         visionProc_itr != registeredProcessors.end(); ++visionProc_itr) {
      toNotify.insert(visionProc_itr->second.begin(), visionProc_itr->second.end());
    }
  } else {
    // collect only registered procs with matching typeIds
    for (typeId_itr = n->typeIds.begin(); typeId_itr != n->typeIds.end(); ++typeId_itr) {
      visionProc_itr = registeredProcessors.find(*typeId_itr);
      if (visionProc_itr != registeredProcessors.end()) {
        toNotify.insert(visionProc_itr->second.begin(), visionProc_itr->second.end());
      }
    }
  }

  LOG4CXX_TRACE(logger, boost::format("[sendNotifications] to %lu registeredProcessors.") % toNotify.size());

  int counter = 1;
  for (boost::unordered_set<VisionProcess::Ptr>::const_iterator itr = toNotify.begin(); itr != toNotify.end(); ++itr) {
    //    if (counter == 1) {
    LOG4CXX_TRACE(logger, boost::format("[sendNotifications] to %s (%lu of %lu).")
                          % (*itr)->visionProcessName % counter++ % registeredProcessors.size());
    (*itr)->notify(n);
    //    }
  }

  LOG4CXX_TRACE(logger, "[sendNotifications] done.");
}

void VisionProcess::notify(Notification::Ptr n) {
  //LOG4CXX_TRACE(logger, boost::format("[notify] notification type: %d.") % n->type);
  if (isRunning) {
    notifications.push(n);
  }
}

void VisionProcess::getPendingNotifications(std::list<Notification::Ptr> &pending) {
  pending.clear();
  Notification::Ptr n;
  // NOTE: Popping only the number of notifications that is currently in the queue
  // avoids being stuck in an endless loop in case new notifications come in at the
  // same rate as we are reading them off. (Admittedly a very unlikely case, but the
  // devil never sleeps ...)
  size_t s = notifications.size();
  for (size_t i = 0; i < s; i++) {
    if (notifications.trypop(n)) {
      pending.push_back(n);
    }
  }
}

void VisionProcess::waitPendingNotifications(std::list<Notification::Ptr> &pending) {
  pending.clear();
  Notification::Ptr n;
  notifications.wait();
  size_t s = notifications.size();
  for (size_t i = 0; i < s; i++) {
    if (notifications.trypop(n)) {
      pending.push_back(n);
    }
  }
}

void VisionProcess::waitPendingNotification(Notification::Ptr &pending) {
  pending = notifications.waitpop();
}

void VisionProcess::clearPendingNotifications() {
  Notification::Ptr n;
  size_t s = notifications.size();
  for (size_t i = 0; i < s; i++) {
    notifications.trypop(n);
  }
}

void VisionProcess::removeNotificationsOfType(std::list<Notification::Ptr> &pending, Notification::Type type) {
  for (std::list<Notification::Ptr>::iterator i = pending.begin(); i != pending.end();) {
    if ((*i)->type == type) {
      i = pending.erase(i);
    } else {
      ++i;
    }
  }
}

void VisionProcess::removeNotificationsOfType(std::list<Notification::Ptr> &pending,
                                              Notification::Type type, unsigned long int frameNum) {
  for (std::list<Notification::Ptr>::iterator i = pending.begin(); i != pending.end();) {
    if ((*i)->type == type && (*i)->frameNumber < frameNum) {
      i = pending.erase(i);
    } else {
      ++i;
    }
  }
}

void VisionProcess::interruptNotificationWait() {
  notifications.interrupt();
}

bool VisionProcess::addProcessingDescriptor(const std::string &descriptor, const long long &typeId) {
  LOG4CXX_DEBUG(logger, boost::format("[addProcessingDescriptor] descriptor: %s. typeId: %lld.") % descriptor % typeId);

  //lock
  boost::lock_guard<boost::mutex> lock(descriptor_mutex);

  PredicateHelper predicate(descriptor);

  // first container
  //deep copy, in case it's being used by another thread
  typesByDescriptor = boost::make_shared<TypesByDescriptor>(*typesByDescriptor);

  //add (potential) new entry
  if (typesByDescriptor->find(predicate) == typesByDescriptor->end()) {
    (*typesByDescriptor)[predicate] = std::tr1::unordered_set<long long>();
  }
  (*typesByDescriptor)[predicate].insert(typeId);

  //second container
  //deep copy, in case it's being used by another thread
  descriptorsByType = boost::make_shared<DescriptorsByType>(*descriptorsByType);

  //add (potential) new entry
  if (descriptorsByType->find(typeId) == descriptorsByType->end()) {
    (*descriptorsByType)[typeId] = std::tr1::unordered_set<PredicateHelper, std::tr1::hash<PredicateHelper>, predhelper_equal_to>();
  }
  (*descriptorsByType)[typeId].insert(predicate);
  //TODO: return true/false based on value being there already or not

  //log results
  if (logger->isDebugEnabled()) {
    LOG4CXX_DEBUG(logger, "Printing Processor descriptors...");
    TypesByDescriptor::iterator itr;
    for (itr = typesByDescriptor->begin(); itr != typesByDescriptor->end(); ++itr) {
      LOG4CXX_DEBUG(logger, boost::format("Processor descriptor: %s.") % itr->first.toString());
    }
  }
  return true;
}

bool VisionProcess::addProcessingDescriptor(const ade::common::fol::Predicate &predicate, const long long &typeId) {
  //  LOG4CXX_INFO(logger, predicate.toString());
  //  tmpPredicates.push_back(predicate);

  //  Symbol tmpSymbol("nativearg1");
  //  Variable tmpVariable("nativearg2", "nativearg2type");
  //  Symbol args[2] = {tmpSymbol, tmpVariable};
  //  Predicate tmp("nativepredicate", args, 2);
  //  tmpPredicates.push_back(tmp);
  return true;
}

void VisionProcess::printPredicate() {
  for (size_t i = 0; i < tmpPredicates.size(); ++i) {
    LOG4CXX_INFO(logger, tmpPredicates[i]->toString());
  }
}

bool VisionProcess::removeProcessingDescriptor(const std::string &descriptor, const long long &typeId) {
  //lock
  boost::lock_guard<boost::mutex> lock(descriptor_mutex);

  PredicateHelper predicate(descriptor);

  // first container
  //deep copy, in case it's being used by another thread
  //TODO: test this!
  typesByDescriptor = boost::make_shared<TypesByDescriptor>(*typesByDescriptor);

  // remove entry
  TypesByDescriptor::iterator typesByDescriptor_itr = typesByDescriptor->find(predicate);
  if (typesByDescriptor_itr != typesByDescriptor->end()) {
    typesByDescriptor_itr->second.erase(typeId);
    //remove completely if empty
    if (typesByDescriptor_itr->second.size() == 0) {
      typesByDescriptor->erase(typesByDescriptor_itr);
    }
  }

  //second container
  //deep copy, in case it's being used by another thread
  //TODO: test this!
  descriptorsByType = boost::make_shared<DescriptorsByType>(*descriptorsByType);

  // remove entry
  int erased = 0;
  DescriptorsByType::iterator descriptorsByType_itr = descriptorsByType->find(typeId);
  if (descriptorsByType_itr != descriptorsByType->end()) {
    erased = descriptorsByType_itr->second.erase(predicate);
    //remove completely if empty
    if (descriptorsByType_itr->second.size() == 0) {
      descriptorsByType->erase(descriptorsByType_itr);
    }
  }

  return erased;
}

VisionProcess::TypesByDescriptorConstPtr VisionProcess::getDescriptors() const {
  //lock
  boost::lock_guard<boost::mutex> lock(descriptor_mutex);

  return typesByDescriptor;
}

VisionProcess::DescriptorsByTypeConstPtr VisionProcess::getTypes() const {
  //lock
  boost::lock_guard<boost::mutex> lock(descriptor_mutex);

  return descriptorsByType;
}

bool VisionProcess::hasLearned() const {
  return false;
}

void VisionProcess::resetHasLearned() {
  // intentionally empty
}

void
VisionProcess::displayMemoryObject(ade::stm::MemoryObject::Ptr &mo, const std::string &variableName, bool binaryMask,
                                   bool combine) {
  MemoryObject::Vec moVec;
  if (mo->getVariableName().compare(variableName) == 0) {
    moVec.push_back(mo);
  } else {
    moVec = mo->getChildren(variableName);
  }

  std::vector<cv::Mat> imagesToDisplay;
  cv::Mat combinedImage;
  cv::Mat tmpImage;
  cv::Point txtOrig(2, 12);
  cv::Scalar color = cv::Scalar::all(255);

  MemoryObject::Vec::const_iterator moVec_itr;
  for (moVec_itr = moVec.begin(); moVec_itr != moVec.end(); ++moVec_itr) {
    // get binary image or colored image
    cv::Mat imageMask;
    if (binaryMask) {
      imageMask = (*moVec_itr)->getTrackingMask()->getImageMask();
    } else {
      imageMask = (*moVec_itr)->getTrackingMask()->getObjectImage();
    }
    if (combine) {
      if (moVec_itr == moVec.begin()) {
        // reset combined in case it was previously used, set it, and add to vector
        combinedImage.release();
        imageMask.copyTo(combinedImage);
        cv::putText(combinedImage, variableName, txtOrig, cv::FONT_HERSHEY_PLAIN, 1.0, color);
        imagesToDisplay.push_back(combinedImage);
      } else {
        // this also modifies the combinedImage that has been added to vector above
        cv::max(combinedImage, imageMask, combinedImage);
      }
    } else {
      tmpImage.release();
      imageMask.copyTo(tmpImage);
      cv::putText(tmpImage, variableName, txtOrig, cv::FONT_HERSHEY_PLAIN, 1.0, color);
      imagesToDisplay.push_back(tmpImage);
    }
  }

  LOG4CXX_DEBUG(logger, boost::format("[displayMemoryObject] num display images: %d.") % imagesToDisplay.size());
  if (imagesToDisplay.size() > 0) {
    cv::Mat displayFrame = ade::display::makeCanvas(imagesToDisplay);
    ade::Display::displayFrame(displayFrame, getDisplayName());
    //    ade::Display::displayFrame(imagesToDisplay[0], getDisplayName());
  }
}

void VisionProcess::displayMemoryObjectValidation(const ade::stm::MemoryObject::ConstPtr &mo, bool combine) {
  // various iterators and such
  DescriptorsByTypeConstPtr descriptorsByType = getTypes();
  DescriptorsByType::const_iterator descriptorsByType_itr;
  PredicateHelper::Set::const_iterator descriptor_itr;
  ValidationResult::Vec validations;
  ValidationResult::Vec::const_iterator validations_itr;

  std::vector<cv::Mat> imagesToDisplay;
  cv::Mat combinedImage;
  cv::Mat tmpImage;
  cv::Point txtOrig(2, 12);
  cv::Scalar color = cv::Scalar::all(255);

  descriptorsByType_itr = descriptorsByType->find(mo->getTypeId());

  if (descriptorsByType_itr != descriptorsByType->end()) {

    // iterate through set of descriptors
    for (descriptor_itr = descriptorsByType_itr->second.begin();
         descriptor_itr != descriptorsByType_itr->second.end(); ++descriptor_itr) {

      // iterate through validation results if any exist
      validations = mo->getValidationResults(*descriptor_itr);
      LOG4CXX_DEBUG(logger, boost::format("[displayMemoryObject] type: %lld. descriptor: %s. num validations: %d.")
                            % descriptorsByType_itr->first % descriptor_itr->toString() % validations.size());

      for (validations_itr = validations.begin(); validations_itr != validations.end(); ++validations_itr) {
        const cv::Mat &imageMask = (*validations_itr)->getObjectImage();
        //          const cv::Mat& imageMask = (*validations_itr)->getImageMask();
        if (combine) {
          if (validations_itr == validations.begin()) {
            // reset combined in case it was previously used, set it, and add to vector
            combinedImage.release();
            imageMask.copyTo(combinedImage);
            cv::putText(combinedImage, descriptor_itr->toString(), txtOrig, cv::FONT_HERSHEY_PLAIN, 1.0, color);
            imagesToDisplay.push_back(combinedImage);
          } else {
            // this also modifies the combinedImage that has been added to vector above
            cv::max(combinedImage, imageMask, combinedImage);
          }
        } else {
          tmpImage.release();
          imageMask.copyTo(tmpImage);
          cv::putText(tmpImage, descriptor_itr->toString(), txtOrig, cv::FONT_HERSHEY_PLAIN, 1.0, color);
          imagesToDisplay.push_back(tmpImage);
        }
      }
    }
  }

  LOG4CXX_DEBUG(logger, boost::format("[displayMemoryObject] num display images: %d.") % imagesToDisplay.size());
  if (imagesToDisplay.size() > 0) {
    cv::Mat displayFrame = ade::display::makeCanvas(imagesToDisplay);
    ade::Display::displayFrame(displayFrame, getDisplayName());
    //    ade::Display::displayFrame(imagesToDisplay[0], getDisplayName());
  }
}

void VisionProcess::displayMemoryObjectRelations(const ade::stm::MemoryObject::ConstPtr &mo) {
  LOG4CXX_DEBUG(logger, boost::format("[displayMemoryObjectRelations] mo: %s.") % mo->getVariableName());

  std::vector<cv::Mat> imagesToDisplay;
  cv::Mat tmpImage;
  cv::Mat redImage = cv::Mat(img_height, img_width, CV_8UC3, cv::Scalar(0, 0, 255));
  cv::Mat greenImage = cv::Mat(img_height, img_width, CV_8UC3, cv::Scalar(0, 255, 0));
  cv::Point txtOrig(2, 12);
  cv::Scalar color = cv::Scalar::all(255);

  // get current descriptors to process
  DescriptorsByTypeConstPtr descriptorsByType = getTypes();
  DescriptorsByType::const_iterator descriptorsByType_iter = descriptorsByType->find(mo->getTypeId());
  PredicateHelper::Set::const_iterator descriptors_itr;
  RelationValidationResult::Vec::const_iterator relations_itr;

  if (descriptorsByType_iter != descriptorsByType->end()) {
    // iterate through the (type's) descriptors
    for (descriptors_itr = descriptorsByType_iter->second.begin();
         descriptors_itr != descriptorsByType_iter->second.end(); ++descriptors_itr) {
      if (descriptors_itr->getNumArgs() != 2) {
        LOG4CXX_ERROR(logger, boost::format("%s does not have 2 args.") % descriptors_itr->toString());
        continue;
      }

      // parse descriptor
      std::string relation = descriptors_itr->getName();
      std::string arg0 = descriptors_itr->getArg(0);
      std::string arg1 = descriptors_itr->getArg(1);

      // get MOs matching first arg of relation
      MemoryObject::Vec arg0_MOs = mo->getChildren(arg0);
      MemoryObject::Vec::iterator arg0_MOs_iter;

      for (arg0_MOs_iter = arg0_MOs.begin(); arg0_MOs_iter != arg0_MOs.end(); ++arg0_MOs_iter) {
        const RelationValidationResult::Vec relations = (*arg0_MOs_iter)->getRelations(relation);
        // look through all of relations of arg1 MO
        for (relations_itr = relations.begin(); relations_itr != relations.end(); ++relations_itr) {
          MemoryObject::ConstPtr relatedObject = (*relations_itr)->getRelatedObject();
          if (relatedObject && relatedObject->getVariableName().compare(arg1) == 0) {

            const cv::Mat &arg0Mask = (*arg0_MOs_iter)->getTrackingMask()->getObjectImage();
            const cv::Mat &arg1Mask = relatedObject->getTrackingMask()->getObjectImage();
            tmpImage.release();
            greenImage.copyTo(tmpImage, arg1Mask);
            redImage.copyTo(tmpImage, arg0Mask);
            cv::putText(tmpImage, descriptors_itr->toString(), txtOrig, cv::FONT_HERSHEY_PLAIN, 1.0, color);
            imagesToDisplay.push_back(tmpImage);
          }
        }
      }
    }
  }


  LOG4CXX_DEBUG(logger,
                boost::format("[displayMemoryObjectRelations] num display images: %d.") % imagesToDisplay.size());
  if (imagesToDisplay.size() > 0) {
    cv::Mat displayFrame = ade::display::makeCanvas(imagesToDisplay);
    ade::Display::displayFrame(displayFrame, getDisplayName());
  }
}