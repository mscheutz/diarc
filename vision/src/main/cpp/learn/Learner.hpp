/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   Learner.hpp
 * Author: Evan Krause <evan.krause@tufts.edu>
 *
 * Created on November 14, 2016, 5:46 PM
 */

#ifndef LEARNER_HPP
#define	LEARNER_HPP

#include "stm/MemoryObject.hpp"
#include "visionproc/VisionProcess.hpp"

/**
 * This is the base class for the object Learner classes.
 *
 * @author Evan Krause
 */
class Learner : public VisionProcess {
public:
  typedef boost::shared_ptr<Learner> Ptr;
  typedef boost::shared_ptr<const Learner> ConstPtr;
  
  enum LearnerType {
    DEFINITION,
    INSTANCE
  };
  
  /**
   * Factory method to get any and all object Learners.
   * @param type LearnerType being requested
   * @param processorId VisionProcess ID
   * @param imgWidth image width
   * @param imgHeight image height
   * @return 
   */
  static Learner::Ptr get(const LearnerType type, const long long& processorId, const int imgWidth, const int imgHeight);

  virtual ~Learner();

protected:
  Learner(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight);

private:

  //! For logging in factory method only.
  static log4cxx::LoggerPtr factoryLogger;
};

#endif	/* LEARNER_HPP */

