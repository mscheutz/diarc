/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "Learner.hpp"
#include "SegmentedObjectLearner.hpp"


// NOTE: not named "diarc.learn.Learner" because there could be non-static
// loggers named "diarc.learn.Learner"
log4cxx::LoggerPtr Learner::factoryLogger = log4cxx::Logger::getLogger("diarc.learn.Learner.Factory");

//Factory method
Learner::Ptr Learner::get(const LearnerType type, const long long& processorId, const int imgWidth, const int imgHeight) {
  switch (type) {
    case INSTANCE:
      return SegmentedObjectLearner::Ptr(new SegmentedObjectLearner(processorId, imgWidth, imgHeight));
      break;
  }

  LOG4CXX_ERROR(factoryLogger, "Empty Learner::Ptr being returned.");
  return Learner::Ptr();
}

Learner::Learner(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight)
: VisionProcess(processorId, imgWidth, imgHeight) {
  logger = log4cxx::Logger::getLogger("diarc.learn.Learner");
}

Learner::~Learner() {
}
