/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   SpatialRelationValidator.hpp
 * Author: Evan Krause <evan.krause@tufts.edu>
 *
 * Created on March 23, 2016, 2:00 PM
 */

#ifndef SPATIALRELATIONVALIDATOR_HPP
#define	SPATIALRELATIONVALIDATOR_HPP

#include "ObjectValidator.hpp"
#include "stm/ValidationResult.hpp"

class SpatialRelationValidator : public ObjectValidator {
public:
  typedef boost::shared_ptr<SpatialRelationValidator> Ptr;
  typedef boost::shared_ptr<const SpatialRelationValidator> ConstPtr;

  SpatialRelationValidator(const long long& processorId, const unsigned int imgWidth,
          const unsigned int imgHeight, const bool isStereo);
  ~SpatialRelationValidator();

  virtual void loadConfig(const std::string& config);

protected:
  virtual void handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification);

private:
  bool calculateSpatialRelation(const PredicateHelper& descriptor,
          diarc::stm::MemoryObject::Ptr& mo0, diarc::stm::MemoryObject::Ptr& mo1);

  bool calculateNearRelation(const PredicateHelper& descriptor,
        diarc::stm::MemoryObject::Ptr& mo0, diarc::stm::MemoryObject::Ptr& mo1);
    
  bool calculateOnRelation(const PredicateHelper& descriptor,
        diarc::stm::MemoryObject::Ptr& mo0, diarc::stm::MemoryObject::Ptr& mo1);
};

#endif	/* SPATIALRELATIONVALIDATOR_HPP */

