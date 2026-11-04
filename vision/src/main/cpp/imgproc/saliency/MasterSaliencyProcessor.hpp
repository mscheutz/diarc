/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * Image processor to fuse all saliencies into a single master saliency.
 *
 * @author Michael Zillich (MZ)
 * @date Oct 2012
 */

#ifndef MASTERSALIENCYPROCESSOR_HPP
#define MASTERSALIENCYPROCESSOR_HPP

#include "SaliencyProcessor.hpp"
#include <boost/unordered_map.hpp>

class MasterSaliencyProcessor : public SaliencyProcessor {
public:
  typedef boost::shared_ptr<MasterSaliencyProcessor> Ptr;
  typedef boost::shared_ptr<const MasterSaliencyProcessor> ConstPtr;

  MasterSaliencyProcessor(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight, const bool isStereo);

  /**
   * Overridden in order to not register for capture notifications.
   */
  virtual void init();
  /**
   * Overridden in order to not un-register for capture notifications.
   */
  virtual void cleanup();

protected:
  virtual void handleSaliencyNotification(SaliencyNotification::ConstPtr notification);
  virtual void notifyOfRegistration(VisionProcess::Ptr notifyingSource);
  virtual void notifyOfUnregistration(VisionProcess::Ptr notifyingSource);

private:
  boost::unordered_map<long long, cv::Mat> saliencyMaps;       //hashed by processorId
  mutable boost::mutex saliencyMaps_mutex;
};

#endif  // MASTERSALIENCYPROCESSOR_HPP
