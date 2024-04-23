/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   SaliencyProcessor.cpp
 * Author: Evan Krause
 * 
 * Created on July 17, 2013, 6:19 PM
 */

#include "SaliencyProcessor.hpp"

SaliencyProcessor::SaliencyProcessor(const long long& processorId, const unsigned int imgwidth,
        const unsigned int imgheight, const bool isStereo)
: ImageProcessor(processorId, imgwidth, imgheight, isStereo) {

  logger = log4cxx::Logger::getLogger("ade.imgproc.saliency.SaliencyProcessor");
}

SaliencyProcessor::~SaliencyProcessor() {
}

void SaliencyProcessor::handleFrameCompletionNotification(FrameCompletionNotification::ConstPtr notification) {
  // intentionally left empty
}

