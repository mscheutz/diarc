/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   SegmentedObjectLearner.hpp
 * Author: Evan Krause <evan.krause@tufts.edu>
 *
 * Created on November 14, 2016, 6:04 PM
 */

#ifndef SEGMENTEDOBJECTLEARNER_HPP
#define	SEGMENTEDOBJECTLEARNER_HPP

#include "Learner.hpp"

class SegmentedObjectLearner : public Learner {
public:
  typedef boost::shared_ptr<SegmentedObjectLearner> Ptr;
  typedef boost::shared_ptr<const SegmentedObjectLearner> ConstPtr;

  SegmentedObjectLearner(const long long& processorId, const int imgWidth, const int imgHeight);
  ~SegmentedObjectLearner();

protected:
  virtual void handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification);

private:

};

#endif	/* SEGMENTEDOBJECTLEARNER_HPP */

