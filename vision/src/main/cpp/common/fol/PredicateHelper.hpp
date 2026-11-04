/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   PredicateHelper.hpp
 * Author: Evan Krause <evan.krause@tufts.edu>
 *
 * Created on March 29, 2016, 3:32 PM
 */

#ifndef PREDICATEHELPER_HPP
#define	PREDICATEHELPER_HPP

#include <boost/format.hpp>
#include <tr1/unordered_set>
#include <log4cxx/logger.h>
#include <string>
#include <vector>

struct predhelper_equal_to;

class PredicateHelper {
public:
  typedef boost::shared_ptr<PredicateHelper> Ptr;
  typedef boost::shared_ptr<const PredicateHelper> ConstPtr;
  typedef std::tr1::unordered_set<PredicateHelper, std::tr1::hash<PredicateHelper>, predhelper_equal_to> Set;

  PredicateHelper(const std::string& predicate)
  : predicate_(predicate),
  name_(),
  args_(),
  logger(log4cxx::Logger::getLogger("diarc.common.fol.PredicateHelper")) {
    size_t opar = predicate_.find_first_of("(");
    size_t cpar = predicate_.find_last_of(")");
    size_t comma;
    std::string tmpStr;

    if (opar != std::string::npos && cpar != std::string::npos) {
      name_ = predicate_.substr(0, opar);

      tmpStr = predicate_.substr(opar + 1, cpar - opar - 1);
      comma = tmpStr.find(",");
      while (!tmpStr.empty()) {
        if (comma != std::string::npos) {
          if (areParenBalanced(tmpStr.substr(0, comma))) {
            LOG4CXX_DEBUG(logger, boost::format("balanced paren: %s. comma: %d. next arg: %s.") % tmpStr % comma % tmpStr.substr(0, comma));
            args_.push_back(tmpStr.substr(0, comma));
            tmpStr = (tmpStr.length() >= comma + 1) ? tmpStr.substr(comma + 1) : "";
            comma = tmpStr.find(",");
          } else {
            //not balanced paren - grow string to next comma or end of string
            LOG4CXX_DEBUG(logger, boost::format("not balanced paren: %s. comma: %d.") % tmpStr % comma);
            comma = tmpStr.find(",", comma + 1);
          }
        } else {
          // no commas -- last arg
          LOG4CXX_DEBUG(logger, boost::format("no commas (last arg): %s.") % tmpStr);
          args_.push_back(tmpStr);
          tmpStr = ""; //to terminate loop
        }
      }
    } else {
      // no parenthesis -- incorrect form, just do this for now
      name_ = predicate_;
    }

    if (logger->isDebugEnabled()) {
      LOG4CXX_DEBUG(logger, boost::format("Predicate: %s.") % predicate_);
      LOG4CXX_DEBUG(logger, boost::format("Predicate name: %s.") % name_);
      for (uint i = 0; i < args_.size(); ++i) {
        LOG4CXX_DEBUG(logger, boost::format("Predicate arg(%d): %s.") % i % args_[i]);
      }
    }
  }

  ~PredicateHelper() {
  }

  std::string toString() const {
    return predicate_;
  }

  std::string getName() const {
    return name_;
  }

  std::string getArg(int i) const {
    if (args_.size() > (uint) i) {
      return args_.at(i);
    }
    return "";
  }

  int getNumArgs() const {
    return args_.size();
  }

private:
  std::string predicate_;
  std::string name_;
  std::vector<std::string> args_;
  log4cxx::LoggerPtr logger;

  bool areParenBalanced(const std::string& str) {
    // count open paren
    int opar_count = 0;
    size_t opar = str.find("(");
    while (opar != std::string::npos) {
      opar = str.find("(", opar + 1);
      ++opar_count;
    }

    // count close paren
    int cpar_count = 0;
    size_t cpar = str.find(")");
    while (cpar != std::string::npos) {
      cpar = str.find(")", cpar + 1);
      ++cpar_count;
    }

    if (opar_count == cpar_count) {
      return true;
    } else {
      return false;
    }
  }

};

namespace std {
  namespace tr1 {

    template<>
    struct hash<PredicateHelper> : public unary_function<PredicateHelper, size_t> {

      size_t operator()(const PredicateHelper& p) const {
        return hash<std::string>()(p.toString());
      }
    };
  }
}

struct predhelper_equal_to : std::binary_function<PredicateHelper, PredicateHelper, bool> {

  bool operator()(const PredicateHelper& p1, const PredicateHelper& p2) const {
    return (p1.toString().compare(p2.toString()) == 0);
  }
};


#endif	/* PREDICATEHELPER_HPP */

