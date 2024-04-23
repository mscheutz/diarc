/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   OrientationValidator.hpp
 * Author: evan
 *
 * Created on March 27, 2014, 5:23 PM
 */

#ifndef ORIENTATIONVALIDATOR_HPP
#define	ORIENTATIONVALIDATOR_HPP

#include "ObjectValidator.hpp"

class OrientationValidator : public ObjectValidator {
public:
  typedef boost::shared_ptr<OrientationValidator> Ptr;
  typedef boost::shared_ptr<const OrientationValidator> ConstPtr;

  OrientationValidator(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight, const bool isStereo);
  ~OrientationValidator();

protected:
  virtual void handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification);

private:
};



#endif	/* ORIENTATIONVALIDATOR_HPP */

