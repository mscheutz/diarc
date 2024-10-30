/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   SpatialRelationDetector.hpp
 * Author: Evan Krause
 *
 * Created on February 5, 2013, 6:48 PM
 */

#ifndef SPATIALRELATIONDETECTOR_HPP
#define  SPATIALRELATIONDETECTOR_HPP

#include "ObjectDetector.hpp"
#include "stm/TrackedObjects.hpp"
#include <boost/unordered_map.hpp>

class SpatialRelationDetector : public ObjectDetector {
public:
  typedef boost::shared_ptr<SpatialRelationDetector> Ptr;
  typedef boost::shared_ptr<const SpatialRelationDetector> ConstPtr;

  SpatialRelationDetector(const long long &processorId, const int imgWidth, const int imgHeight);

  ~SpatialRelationDetector();

  virtual void loadConfig(const std::string &configFile);

  /**
   * Override to disable automatic capture notification registration.
   */
  virtual void init();

  /**
   * Override to disable automatic capture notification unregistration.
   */
  virtual void cleanup();


protected:

  virtual void handleFrameCompletionNotification(FrameCompletionNotification::ConstPtr notification);

  virtual void handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification);

private:

  /**
   * Check two objects for specified relation.
   * @param relation relation to check
   * @param referent one of objects in relation
   * @param relatum other object in relation
   * @return if relation was found
   */
  bool checkForRelation(const PredicateHelper &relation,
                        const diarc::stm::MemoryObject::Ptr &referent,
                        const diarc::stm::MemoryObject::Ptr &relatum);

  bool checkForTouchingRelation(const PredicateHelper &relation,
                            const diarc::stm::MemoryObject::Ptr &referent,
                            const diarc::stm::MemoryObject::Ptr &relatum);

  bool checkForOnTopRelation(const PredicateHelper &relation,
                                                      const diarc::stm::MemoryObject::Ptr &referent,
                                                      const diarc::stm::MemoryObject::Ptr &relatum);

  //! last seen MemoryObject tokenIds, hashed by variable name
  boost::unordered_map<std::string, long long> lastSeenTokenIds;

  //! for keeping track of which frame numbers have been fully processed for a particular typeId
  boost::unordered_map<long long, unsigned long long> lastFrameCompletedByType;

  //! handle to tracked objects db
  diarc::stm::TrackedObjects *trackedObjects;
};


#endif  /* SPATIALRELATIONDETECTOR_HPP */

