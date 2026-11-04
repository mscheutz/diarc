/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   RelationValidationResult.hpp
 * Author: Evan Krause <evan.krause@tufts.edu>
 *
 * Created on May 10, 2016, 4:58 PM
 */

#ifndef RELATIONVALIDATIONRESULT_HPP
#define	RELATIONVALIDATIONRESULT_HPP

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include "common/fol/PredicateHelper.hpp"
#include "ConfidenceLevel.hpp"

namespace diarc {
  namespace stm {
    
    // forward declaration
    class MemoryObject;
    typedef boost::shared_ptr<MemoryObject> MemoryObjectPtr;
    typedef boost::shared_ptr<const MemoryObject> MemoryObjectConstPtr;

    class RelationValidationResult {
    public:
      typedef boost::shared_ptr<RelationValidationResult> Ptr;
      typedef boost::shared_ptr<const RelationValidationResult> ConstPtr;
      typedef std::vector<RelationValidationResult::Ptr> Vec;

      RelationValidationResult(const float& confidence, const PredicateHelper& descriptor, MemoryObjectPtr& relatedObject);
      ~RelationValidationResult();

      void setConfidence(const float &confidence);
      void decayConfidence();
      float getConfidence() const;
      const PredicateHelper& getDescriptor() const;
      MemoryObjectPtr getRelatedObject();
      MemoryObjectConstPtr getRelatedObject() const;

    private:
      //! detection or validation confidence [0 1]
      ConfidenceLevel confidence_;

      //! descriptor (e.g., on(X,Y), near(X,Z), etc)
      PredicateHelper descriptor_;
      
      //! other MemoryObject that relationship is between
      boost::weak_ptr<MemoryObject> relatedObject_;

      mutable boost::recursive_mutex data_mutex_;

    };

  } //namespace stm
} //namespace diarc
#endif	/* RELATIONVALIDATIONRESULT_HPP */

