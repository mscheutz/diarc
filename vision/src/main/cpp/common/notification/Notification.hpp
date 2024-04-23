/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * Classes for the notification mechanism
 *
 * @author Michael Zillich (MZ)
 * @date Nov 2012
 */

#ifndef NOTIFICATION_HPP
#define NOTIFICATION_HPP

#include <boost/shared_ptr.hpp>
#include <boost/unordered_set.hpp>

/**
 * Notification base class
 */
class Notification {
public:
  // Notification types.
  // NOTE: we chose not to use the C++ real time type information for that purpose,
  // as it gets quite messy with smart pointers, iterators to lists of smart
  // pointers etc.

  enum Type {
    CAPTURE,
    FRAME_COMPLETION,
    SCENE_CHANGE,
    SALIENCY,
    MEMORY_OBJECT,
    MEMORY_OBJECTS,
    PLANE,
    SIFT,
    MOTION,
    LEARNING
  };

  typedef boost::shared_ptr<Notification> Ptr;
  typedef boost::shared_ptr<const Notification> ConstPtr;

  Notification(const Type& type_, const long long& typeId_, const unsigned long long& frameNumber_)
  : type(type_), typeIds{typeId_}, frameNumber(frameNumber_) {
  }

  Notification(const Type& type_, const boost::unordered_set<long long>& typeIds_, const unsigned long long& frameNumber_)
          : type(type_), typeIds(typeIds_), frameNumber(frameNumber_) {
  }

  virtual ~Notification() {
  }

  const Type type;
  const boost::unordered_set<long long> typeIds;
  const unsigned long long frameNumber;
};

#endif
