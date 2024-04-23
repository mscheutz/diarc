/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   PredicateBuilder.hpp
 * Author: Evan Krause <evan.krause@tufts.edu>
 *
 * Created on February 4, 2016, 3:27 PM
 */

#ifndef PREDICATEBUILDER_HPP
#define	PREDICATEBUILDER_HPP

#include "Symbol.hpp"
#include "Predicate.hpp"
#include "Variable.hpp"

#include <string>
#include <boost/shared_ptr.hpp>
#include <log4cxx/logger.h>

namespace ade {
  namespace common {
    namespace fol {

      class PredicateBuilder {
      public:
        PredicateBuilder()
        : logger_(log4cxx::Logger::getLogger("ade.common.fol.PredicateBuilder")) {
        }

        void setName(const std::string& name) {
          name_ = name;
        }

        void addArgument(const Symbol& arg) {
          const ade::common::fol::Variable* var = dynamic_cast<const ade::common::fol::Variable*> (&arg);
          if (var != NULL) {
            LOG4CXX_DEBUG(logger_, "Adding variable argument: " + arg.toString());
            args_.push_back(boost::make_shared<ade::common::fol::Variable>(*var));
          } else {
            LOG4CXX_DEBUG(logger_, "Adding symbol argument: " + arg.toString());
            args_.push_back(boost::make_shared<ade::common::fol::Symbol>(arg));
          }
        }

        Predicate build() {
          Predicate newPredicate(name_, args_);
          return newPredicate;
        }
      private:
        std::string name_;
        std::vector<Symbol::ConstPtr> args_;
        log4cxx::LoggerPtr logger_;
      };

    } //namespace fol
  } //namespace common
} //namespace ade

#endif	/* PREDICATEBUILDER_HPP */

