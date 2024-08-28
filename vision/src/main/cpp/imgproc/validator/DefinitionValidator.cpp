/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "DefinitionValidator.hpp"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>

using namespace diarc::stm;

DefinitionValidator::DefinitionValidator(const long long& processorId, const unsigned int imgWidth,
        const unsigned int imgHeight)
: ObjectValidator(processorId, imgWidth, imgHeight, false) {

}

DefinitionValidator::~DefinitionValidator() {

}

void DefinitionValidator::loadConfig(const std::string& config) {
  LOG4CXX_TRACE(logger, "[loadConfig] method entered.");

  // get directory
  unsigned found = config.find_last_of("/\\");
  std::string dir = config.substr(0, found + 1);

  // populate tree structure pt
  using boost::property_tree::ptree;
  ptree pt;
  read_xml(config, pt);

  std::string descriptorName;
  std::string varName;
  std::vector<PredicateHelper> definition;

  // parse tree

  BOOST_FOREACH(ptree::value_type const& predicateNode, pt.get_child("processor")) {
    if (predicateNode.first.compare("predicate") == 0) {
      descriptorName = predicateNode.second.get<std::string> ("<xmlattr>.name", "unknown");
      LOG4CXX_DEBUG(logger, boost::format("[loadConfig] descriptorName: %s.") % descriptorName);

      BOOST_FOREACH(ptree::value_type const& dataNode, predicateNode.second) {
        if (dataNode.first.compare("varname") == 0) {
          varName = static_cast<std::string> (dataNode.second.data());
        } else if (dataNode.first.compare("definition") == 0) {

          BOOST_FOREACH(ptree::value_type const& definitionNode, dataNode.second) {
            PredicateHelper defTerm = PredicateHelper(static_cast<std::string> (definitionNode.second.data()));
            definition.push_back(defTerm);
          }
        }
      }
    }

    // add new definition
    PredicateHelper definiendum = PredicateHelper(descriptorName + "(" + varName + ")");
    definitions.insert(std::pair<PredicateHelper, std::vector<PredicateHelper> > (definiendum, definition));
  }

}

void DefinitionValidator::handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification) {
  LOG4CXX_TRACE(logger, "[handleMemoryObjectNotification] method entered.");
  MemoryObject::Ptr object = notification->object;

  // TODO: get objects from scene graph matching definitions

  // TODO: add label to objects matching definitions

  bool validationFound = false;

  // check if validator is actually looking for the detected object type
  // set confidence and descriptors for search typeids
  float confidence = 1.0f; //TODO: set this properly
  TypesByDescriptorConstPtr descriptors = getDescriptors();
  TypesByDescriptor::const_iterator descriptors_itr;
  for (descriptors_itr = descriptors->begin(); descriptors_itr != descriptors->end(); ++descriptors_itr) {
    definitions.find(descriptors_itr->first.getName());
    
//    if (definitions.[0].type_.compare(descriptors_itr->first.getName()) == 0) {
//      if (logger->isDebugEnabled()) {
//        LOG4CXX_DEBUG(logger, boost::format("Descriptor: %s.") % descriptors_itr->first.toString());
//        std::tr1::unordered_set<long long>::const_iterator typeId_itr;
//        for (typeId_itr = descriptors_itr->second.begin(); typeId_itr != descriptors_itr->second.end(); ++typeId_itr) {
//          LOG4CXX_DEBUG(logger, boost::format("TypeId: %ld.") % *(typeId_itr));
//        }
//      }
//      object->addValidationResult(confidence, descriptors_itr->first);
//      validationFound = true;
//    }
  }

//  if (validationFound) {
//    sendValidationNotifications(object);
//  } else {
//    LOG4CXX_DEBUG(logger, boost::format("%s detected but not looking for that category.") % matching_models[0].type_);
//  }
}
