
/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "ValidationResults.hpp"
#include <boost/lexical_cast.hpp>

using namespace diarc::stm;

ValidationResults::ValidationResults()
: validationResults_(),
updated_(true),
confidence_(0.0) {
}

ValidationResults::~ValidationResults() {

}

void ValidationResults::addValidationResult(ValidationResult::ConstPtr validationResult) {

  // add new result to results map
  ValidationResult::VecByPredicate::iterator iter;
  const PredicateHelper& descriptor = validationResult->getDescriptor();
  iter = validationResults_.find(descriptor);
  if (iter == validationResults_.end()) {
    // create and add to new vector
    std::vector<ValidationResult::ConstPtr> validationVec;
    validationVec.push_back(validationResult);
    validationResults_.insert(ValidationResult::VecByPredicate::value_type(descriptor, validationVec));
  } else {
    // add to existing vector
    iter->second.push_back(validationResult);
  }

  updated_ = true;
}

float ValidationResults::getConfidence() const {
  // if validation results haven't been updated, use cached value
  if (!updated_) {
    return confidence_;
  }

  float tmpConfidence;
  if (validationResults_.empty()) {
    tmpConfidence = 0.0;
  } else {
    ValidationResult::VecByPredicate::const_iterator map_iter;
    std::vector<ValidationResult::ConstPtr>::const_iterator vec_iter;
    tmpConfidence = 1.0;
    for (map_iter = validationResults_.begin(); map_iter != validationResults_.end(); ++map_iter) {
      tmpConfidence *= getConfidence(map_iter->first);
    }
  }

  updated_ = false;
  confidence_ = tmpConfidence;
  return (confidence_);
}

float ValidationResults::getConfidence(const PredicateHelper& descriptor) const {
  float confidence = 0.0;
  ValidationResult::VecByPredicate::const_iterator validationResults_itr = validationResults_.find(descriptor);
  if (validationResults_itr != validationResults_.end() && !validationResults_itr->second.empty()) {
    std::vector<ValidationResult::ConstPtr>::const_iterator vec_iter;
    for (vec_iter = validationResults_itr->second.begin(); vec_iter != validationResults_itr->second.end(); ++vec_iter) {
      if ((*vec_iter)->getConfidence() > confidence) {
        confidence = (*vec_iter)->getConfidence();
      }
    }
  }

  return confidence;
}

bool ValidationResults::containsDescriptor(const PredicateHelper& descriptor) const {
  if (validationResults_.find(descriptor) == validationResults_.end()) {
    return false;
  }
  return true;
}

bool ValidationResults::containsAllDescriptors(const PredicateHelper::Set& descriptors) const {
  long unsigned int descriptorsMatched = 0;
  PredicateHelper::Set::const_iterator descriptors_iter;
  for (descriptors_iter = descriptors.begin(); descriptors_iter != descriptors.end(); ++descriptors_iter) {
    if (validationResults_.find(*descriptors_iter) != validationResults_.end()) {
      ++descriptorsMatched;
    }
  }

  if (descriptors.size() != descriptorsMatched) {
    return false;
  }

  return true;
}


const PredicateHelper::Set ValidationResults::getDescriptors() const {
  PredicateHelper::Set descriptors;

  ValidationResult::VecByPredicate::const_iterator map_iter;
  for (map_iter = validationResults_.begin(); map_iter != validationResults_.end(); ++map_iter) {
    descriptors.insert(PredicateHelper::Set::value_type(map_iter->first));
  }

  return descriptors;
}

const ValidationResult::VecByPredicate ValidationResults::getResults() const {
  return validationResults_;
}

const ValidationResult::Vec ValidationResults::getResults(const PredicateHelper& descriptor) const {
  ValidationResult::VecByPredicate::const_iterator itr = validationResults_.find(descriptor);
  if (itr != validationResults_.end()) {
    return itr->second;
  } else {
    return ValidationResult::Vec();
  }
}

std::string ValidationResults::getResultsString() const {
  std::string result;
  ValidationResult::VecByPredicate::const_iterator map_iter;
  for (map_iter = validationResults_.begin(); map_iter != validationResults_.end(); ++map_iter) {
    result.append(map_iter->first.toString());
    result.append(":\n");

    std::vector<ValidationResult::ConstPtr>::const_iterator vec_iter;
    for (vec_iter = map_iter->second.begin(); vec_iter != map_iter->second.end(); ++vec_iter) {
      result.append(boost::lexical_cast<std::string>((*vec_iter)->getConfidence()));
      result.append("\n");
    }
  }

  return result;
}

std::string ValidationResults::getDescriptorsString() const {
  std::string result;
  ValidationResult::VecByPredicate::const_iterator map_iter;
  for (map_iter = validationResults_.begin(); map_iter != validationResults_.end(); ++map_iter) {
    result.append(map_iter->first.toString()).append(" ");
  }

  return result;
}