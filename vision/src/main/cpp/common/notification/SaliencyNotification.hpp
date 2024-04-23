/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   SaliencyNotification.hpp
 * Author: Evan Krause
 *
 * Created on August 27, 2015, 4:34 PM
 */

#ifndef SALIENCYNOTIFICATION_HPP
#define	SALIENCYNOTIFICATION_HPP

#include "common/notification/ImageProcessorNotification.hpp"

/*
 * Notifies about a change of saliency in the scene, e.g. red become
 * salient.
 */
class SaliencyNotification : public ImageProcessorNotification {
public:
  typedef boost::shared_ptr<SaliencyNotification> Ptr;
  typedef boost::shared_ptr<const SaliencyNotification> ConstPtr;

  SaliencyNotification(const VisionProcessConstPtr& notifier_,
                       const unsigned long long& frameNumber_,
                       const CaptureData::ConstPtr& captureData_,
                       const cv::Mat& saliencyMap_)
          : ImageProcessorNotification(notifier_, SALIENCY, frameNumber_, captureData_),
            saliencyMap(saliencyMap_.clone()) {
  }

  //! 2D saliency map - must be explicitly copied (i.e., clone()) by user if being modified
  const cv::Mat saliencyMap;
};


#endif	/* SALIENCYNOTIFICATION_HPP */

