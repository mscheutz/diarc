/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   Validation.hpp
 * Author: evan
 *
 * Created on November 5, 2013, 5:33 PM
 */

#ifndef VALIDATIONRESULTS_HPP
#define	VALIDATIONRESULTS_HPP

#include "ValidationResult.hpp"
#include "common/fol/PredicateHelper.hpp"
#include <boost/format.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/PointIndices.h>

namespace diarc {
  namespace stm {

    class ValidationResults {
    public:
      ValidationResults();
      virtual ~ValidationResults();

      /**
       * Add a new validation result.
       * @param validationResult
       */
      void addValidationResult(ValidationResult::ConstPtr validationResult);
      /**
       * Get combined confidence for all validations, using only the highest
       * confidence value for each descriptor.
       * @return 
       */
      float getConfidence() const;
      /**
       * Get highest confidence matching descriptor.
       * @param descriptor
       * @return 
       */
      float getConfidence(const PredicateHelper& descriptor) const;
      /**
       * Find if descriptor is contained in validation results.
       * @param descriptor
       * @return 
       */
      bool containsDescriptor(const PredicateHelper& descriptor) const;
      /**
       * Find if all descriptors are contained in validation results.
       * @param descriptors
       * @return 
       */
      bool containsAllDescriptors(const PredicateHelper::Set& descriptors) const;
      /**
       * Get all validation results.
       * @return 
       */
      const ValidationResult::VecByPredicate getResults() const;
      /**
       * Get vector of validation results matching descriptor.
       * @param descriptor
       * @return 
       */
      const ValidationResult::Vec getResults(const PredicateHelper& descriptor) const;
      /**
      * Get descriptors for all validation results.
      * @return
      */
      const PredicateHelper::Set getDescriptors() const;
      /**
       * Get validation results in string form. This includes the descriptors and confidence values.
       * @return 
       */
      std::string getResultsString() const;
      /**
       * Get descriptors of validation results in string form.
       * @return
       */
      std::string getDescriptorsString() const;

    private:
      //! all validation results hashed by Predicate
      ValidationResult::VecByPredicate validationResults_;

      //! flag if cached confidence needs to be updated
      mutable bool updated_;
      
      //! cached confidence value
      mutable float confidence_;
    };

  } //namespace stm
} //namespace diarc

#endif	/* VALIDATIONRESULTS_HPP */

