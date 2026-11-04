/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef OBJECTVALIDATOR_HPP
#define OBJECTVALIDATOR_HPP

#include "common/notification/MemoryObjectNotification.hpp"
#include "imgproc/ImageProcessor.hpp"
#include "stm/MemoryObject.hpp"

class ObjectValidator : public ImageProcessor {
public:

  //NOTE: enum not currently used
  enum Type {
    COLOR,
    SHAPE,
    SIZE
  };

  typedef boost::shared_ptr<ObjectValidator> Ptr;
  typedef boost::shared_ptr<const ObjectValidator> ConstPtr;

  virtual ~ObjectValidator();

  /**
   * To override capture registration.
   */
  virtual void init();
  /**
   * To override capture un-registration.
   */
  virtual void cleanup();

protected:
  ObjectValidator(const long long& processorId, const unsigned int imgwidth,
          const unsigned int imgheight, const bool isStereo);

  void sendValidationNotifications(diarc::stm::MemoryObject::VecPtr validatedObjects);
  void sendValidationNotifications(diarc::stm::MemoryObject::Ptr validatedObject);
  void removeNotifications(std::list<Notification::Ptr> &pending, Notification::Ptr n);
};

#endif  //OBJECTVALIDATOR_HPP
