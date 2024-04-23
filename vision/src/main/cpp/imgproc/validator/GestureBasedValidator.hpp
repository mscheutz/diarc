/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   GestureBasedValidator.hpp
 * Author: Evan Krause
 *
 * Created on March 17, 2020,  11:53 AM
 */

#ifndef GESTUREBASEDVALIDATOR_HPP
#define  GESTUREBASEDVALIDATOR_HPP

#include "ObjectValidator.hpp"
#include "stm/TrackedObjects.hpp"
#include <boost/unordered_map.hpp>

class GestureBasedValidator : public ObjectValidator {
public:
  typedef boost::shared_ptr<GestureBasedValidator> Ptr;
  typedef boost::shared_ptr<const GestureBasedValidator> ConstPtr;

  GestureBasedValidator(const long long &processorId, const int imgWidth, const int imgHeight);

  ~GestureBasedValidator();

  virtual void loadConfig(const std::string &configFile);


protected:

  virtual void handleFrameCompletionNotification(FrameCompletionNotification::ConstPtr notification);

  virtual void handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification);

private:
  bool findReferencedObject(const PredicateHelper &relation,
                            const ade::stm::MemoryObject::Ptr &referent,
                            const ade::stm::MemoryObject::Ptr &relatum);

  bool calcTargetPoint(const ade::stm::MemoryObject::Ptr &personRoot,
                       const ade::stm::MemoryObject::Ptr &handRoot,
                       pcl::PointXYZ &targetPoint);

  //! last seen MemoryObject tokenIds, hashed by variable name
  boost::unordered_map<std::string, long long> lastSeenTokenIds;

  //! for keeping track of which frame numbers have been fully processed for a particular typeId
  boost::unordered_map<long long, unsigned long long> lastFrameCompletedByType;

  //! handle to tracked objects db
  ade::stm::TrackedObjects *trackedObjects;
};


#endif  /* GESTUREBASEDVALIDATOR_HPP */

