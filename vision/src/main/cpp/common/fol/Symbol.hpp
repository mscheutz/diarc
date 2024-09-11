/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   Symbol.hpp
 * Author: Evan Krause <evan.krause@tufts.edu>
 *
 * Created on January 29, 2016, 5:38 PM
 */

#ifndef SYMBOL_HPP
#define	SYMBOL_HPP

#include <string>
#include <boost/shared_ptr.hpp>
#include <log4cxx/logger.h>

namespace diarc {
  namespace common {
    namespace fol {

      class Symbol {
      public:
        typedef boost::shared_ptr<Symbol> Ptr;
        typedef boost::shared_ptr<const Symbol> ConstPtr;

        Symbol(const std::string& name)
        : name_(name),
        logger_(log4cxx::Logger::getLogger("diarc.common.fol.Symbol")) {
        };

        virtual ~Symbol() {
        };

        std::string getName() const {
          return name_;
        };

        virtual std::string toString() const {
          return name_;
        };

      protected:
        std::string name_;
        log4cxx::LoggerPtr logger_;

      private:
      };

    } //namespace fol
  } //namespace common
} //namespace diarc

#endif	/* SYMBOL_HPP */

