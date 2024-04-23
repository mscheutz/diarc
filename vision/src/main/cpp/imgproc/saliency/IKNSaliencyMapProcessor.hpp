/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

// class to calculate ikn saliency
// author: ep

#ifndef IKNSALIENCYPROCESSOR_HPP
#define IKNSALIENCYPROCESSOR_HPP

#include "SaliencyProcessor.hpp"
#include <v4r/AttentionModule/AttentionModule.hpp>
#include <v4r/EPUtils/EPUtils.hpp>

class IKNSaliencyMapProcessor : public SaliencyProcessor {
public:
  typedef boost::shared_ptr<IKNSaliencyMapProcessor> Ptr;
  typedef boost::shared_ptr<const IKNSaliencyMapProcessor> ConstPtr;

  IKNSaliencyMapProcessor(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight, const bool isStereo);
  virtual ~IKNSaliencyMapProcessor();

protected:
  virtual void handleCaptureNotification(CaptureNotification::ConstPtr notification);

private:
};


#endif  //IKNSALIENCYPROCESSOR_HPP
