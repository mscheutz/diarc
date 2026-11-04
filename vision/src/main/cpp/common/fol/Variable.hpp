/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   Variable.hpp
 * Author: Evan Krause <evan.krause@tufts.edu>
 *
 * Created on January 29, 2016, 5:37 PM
 */
#ifndef VARIABLE_HPP
#define	VARIABLE_HPP

#include "Symbol.hpp"

namespace diarc {
  namespace common {
    namespace fol {

      class Variable : public Symbol {
      public:
        typedef boost::shared_ptr<Variable> Ptr;
        typedef boost::shared_ptr<const Variable> ConstPtr;

        Variable(const std::string& name, const std::string& type)
        : Symbol(name),
        type_(type) {
          logger_ = log4cxx::Logger::getLogger("diarc.common.fol.Symbol");
        }

        virtual ~Variable() {
        };

        std::string getType() const {
          return type_;
        };

        virtual std::string toString() const {
          return (name_ + " - " + type_);
        };

      private:
        std::string type_;
      };

    } //namespace fol
  } //namespace common
} //namespace diarc

#endif	/* VARIABLE_HPP */

