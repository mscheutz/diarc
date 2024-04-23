/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   DefinitionValidator.hpp
 * Author: Evan Krause <evan.krause@tufts.edu>
 *
 * Created on November 18, 2016, 1:55 PM
 */

#ifndef DEFINITIONVALIDATOR_HPP
#define	DEFINITIONVALIDATOR_HPP

#include "ObjectValidator.hpp"
#include "common/fol/PredicateHelper.hpp"
#include <tr1/unordered_map>

class DefinitionValidator : public ObjectValidator {
public:
  typedef boost::shared_ptr<DefinitionValidator> Ptr;
  typedef boost::shared_ptr<const DefinitionValidator> ConstPtr;

  DefinitionValidator(const long long& processorId, const unsigned int imgWidth,
          const unsigned int imgHeight);
  virtual ~DefinitionValidator();

  virtual void loadConfig(const std::string& config);

protected:
  virtual void handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification);

private:
  std::tr1::unordered_map<PredicateHelper, std::vector<PredicateHelper>, std::tr1::hash<PredicateHelper>, predhelper_equal_to> definitions;
};


#endif	/* DEFINITIONVALIDATOR_HPP */

