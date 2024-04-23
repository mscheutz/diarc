/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "ObjectValidator.hpp"

#include <opencv2/opencv.hpp>
#include <vector>
#include <iterator>
#include <string>
#include <opencv2/core/core_c.h>
#include "common/notification/FrameCompletionNotification.hpp"
#include "display/Display.hpp"

using namespace ade::stm;

ObjectValidator::ObjectValidator(const long long& processorId, const unsigned int imgWidth,
        const unsigned int imgHeight, const bool isStereo)
: ImageProcessor(processorId, imgWidth, imgHeight, isStereo) {
  visionProcessName = "ObjectValidator";
  logger = log4cxx::Logger::getLogger("ade.imgproc.validator.ObjectValidator");
}

ObjectValidator::~ObjectValidator() {
}

void ObjectValidator::init() {

}

void ObjectValidator::cleanup() {

}

void ObjectValidator::removeNotifications(std::list<Notification::Ptr> &pending, Notification::Ptr n) {
  LOG4CXX_TRACE(logger, "[removeNotifications] method entered.");
  int init_count = pending.size();
  if (n->type == Notification::MEMORY_OBJECT) {
    MemoryObjectNotification::Ptr mon = boost::dynamic_pointer_cast<MemoryObjectNotification>(n);
    //bad cast
    if (!mon) {
      LOG4CXX_ERROR(logger, "[removeNotifications] bad cast.");
      return;
    }

    for (std::list<Notification::Ptr>::iterator i = pending.begin(); i != pending.end();) {
      if ((*i)->type == Notification::MEMORY_OBJECT) {

        MemoryObjectNotification::Ptr mon_i = boost::dynamic_pointer_cast<MemoryObjectNotification>(*i);
        //bad cast
        if (!mon_i) {
          LOG4CXX_ERROR(logger, "[removeNotifications] wrong type.");
          ++i;
        } else {
          if ((mon_i->object) == (mon->object)) {
            LOG4CXX_TRACE(logger, "[removeNotifications] object is the same, removing.");
            i = pending.erase(i);
          } else {
            ++i;
          }
        }
      } else {
        ++i;
      }
    }
  }
  LOG4CXX_TRACE(logger, boost::format("[removeNotifications] returning after removing %d notifications. remaining notifications: %d.")
          % (init_count - pending.size()) % pending.size());
}

void ObjectValidator::sendValidationNotifications(MemoryObject::VecPtr validatedObjects) {
  LOG4CXX_TRACE(logger, boost::format("[sendValidationNotifications] num validated objects: %d.") % validatedObjects->size());

  //  MemoryObject::MemoryObjectsNotification::Ptr n(new MemoryObject::MemoryObjectsNotification(validatedObjects,
  //          lastProcessedFrameNum));
  //  sendNotifications(n);

  if (validatedObjects->size() > 0) {
    for (size_t i = 0; i < validatedObjects->size(); i++) {
      MemoryObjectNotification::Ptr n(new MemoryObjectNotification(shared_from_this(),
                                                                   (*validatedObjects)[i]->getTypeId(),
                                                                   (*validatedObjects)[i]->getDetectionMask()->getFrameNumber(),
                                                                   (*validatedObjects)[i]));
      sendNotifications(n);
    }
  }
}

void ObjectValidator::sendValidationNotifications(MemoryObject::Ptr validatedObject) {
  LOG4CXX_TRACE(logger, "[sendValidationNotifications] single validated object.");
  if (validatedObject) {
    MemoryObjectNotification::Ptr n(new MemoryObjectNotification(shared_from_this(), validatedObject->getTypeId(), validatedObject->getDetectionMask()->getFrameNumber(), validatedObject));
    sendNotifications(n);
  }
}