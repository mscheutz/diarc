/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   SaliencyProcessor.hpp
 * Author: Evan Krause
 *
 * Created on July 17, 2013, 6:19 PM
 */

#ifndef SALIENCYPROCESSOR_HPP
#define	SALIENCYPROCESSOR_HPP

#include "imgproc/ImageProcessor.hpp"
#include "common/notification/SaliencyNotification.hpp"

class SaliencyProcessor : public ImageProcessor {
public:
  typedef boost::shared_ptr<SaliencyProcessor> Ptr;
  typedef boost::shared_ptr<const SaliencyProcessor> ConstPtr;

  enum Type {
    COLOR,
    LOCATION,
    ORIENTATION,
    HEIGHT,
    SURFACE_CURVATURE,
    SURFACE_ORIENTATION,
    IKN,
    SYMMETRY,
    MASTER
  };

  virtual ~SaliencyProcessor();

protected:
  SaliencyProcessor(const long long& processorId, const unsigned int imgwidth,
          const unsigned int imgheight, const bool isStereo);

  //! override automatic forwarding of frame completion notifications
  virtual void handleFrameCompletionNotification(FrameCompletionNotification::ConstPtr notification);

private:

};

#endif	/* SALIENCYPROCESSOR_HPP */

