/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   Predicate.hpp
 * Author: Evan Krause <evan.krause@tufts.edu>
 *
 * Created on January 29, 2016, 5:37 PM
 */

#ifndef PREDICATE_HPP
#define	PREDICATE_HPP

#include "Symbol.hpp"
#include "Variable.hpp"
#include <boost/make_shared.hpp>
#include <vector>

namespace diarc {
  namespace common {
    namespace fol {

      class Predicate : public Symbol {
      public:
        typedef boost::shared_ptr<Predicate> Ptr;
        typedef boost::shared_ptr<const Predicate> ConstPtr;

        Predicate(const std::string& name, const std::vector<Symbol::ConstPtr>& args)
        : Symbol(name),
        args_(),
        isNegated_(false) {
          logger_ = log4cxx::Logger::getLogger("diarc.common.fol.Predicate");
          args_.reserve(args.size());
          std::vector<Symbol::ConstPtr>::const_iterator args_itr;
          for (args_itr = args.begin(); args_itr != args.end(); ++args_itr) {
            LOG4CXX_DEBUG(logger_, (*args_itr)->toString());
            Variable::ConstPtr var = boost::dynamic_pointer_cast<const Variable>(*args_itr);
            if (var) {
              args_.push_back(boost::make_shared<Variable>(*var));
            } else {
              args_.push_back(boost::make_shared<Symbol>(*(*args_itr)));
            }
          }
        };

        virtual ~Predicate() {
        };

        long unsigned int getNumArgs() const {
          return args_.size();
        };

        Symbol::ConstPtr getArg(const long unsigned int i) const {
          return args_.at(i);
        };

        virtual std::string toString() const {
          std::string output = name_ + "(";
          std::vector<Symbol::Ptr>::const_iterator args_itr;
          for (args_itr = args_.begin(); args_itr != args_.end(); ++args_itr) {
            output += (*args_itr)->toString();
            if ((args_itr + 1) != args_.end()) {
              output += ", ";
            }
          }
          output += ")";
          return output;
        };

        //        std::vector<Symbol> getVariables() {
        //          std::string output = name_ + ":\n\n";
        //          bool isVariable = false;
        //          std::vector<Symbol> variables;
        //          std::vector<Symbol>::iterator args_itr;
        //          for (args_itr = args_.begin(); args_itr != args_.end(); ++args_itr) {
        //            output += args_itr->toString();
        //            isVariable = false;
        //
        //            try {
        //              Variable& var = dynamic_cast<Symbol&>((*args_itr));
        //              variables.push_back(*args_itr);
        //              isVariable = true;
        //            } catch (const std::bad_cast& e) {
        //              LOG4CXX_ERROR(logger_, "Bad Cast to Variable.");
        //            }
        //            
        //            if (isVariable) {
        //              output += args_itr->name_ + " is a Variable.";
        //            } else {
        //              output += args_itr->name_ + " is a Symbol.";
        //            }
        //            
        //          }
        //         
        //          LOG4CXX_ERROR(logger_, output);
        //          return variables;
        //        }

      private:
        std::vector<Symbol::Ptr> args_;
        bool isNegated_;
      };

    } //namespace fol
  } //namespace common
} //namespace diarc

#endif	/* PREDICATE_HPP */

