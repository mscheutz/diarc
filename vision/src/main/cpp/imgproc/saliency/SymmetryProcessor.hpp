/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

// class to calculate ikn saliency
// author: ep

#ifndef SYMMETRYPROCESSOR_HPP
#define SYMMETRYPROCESSOR_HPP

#include "SaliencyProcessor.hpp"
#include <v4r/AttentionModule/AttentionModule.hpp>
#include <v4r/EPUtils/EPUtils.hpp>

class SymmetryProcessor : public SaliencyProcessor {
public:
  typedef boost::shared_ptr<SymmetryProcessor> Ptr;
  typedef boost::shared_ptr<const SymmetryProcessor> ConstPtr;

  SymmetryProcessor(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight, const bool isStereo);
  ~SymmetryProcessor();

protected:
  virtual void handleCaptureNotification(CaptureNotification::ConstPtr notification);

private:
};


#endif  //SYMMETRYPROCESSOR_HPP
