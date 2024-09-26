/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * Image processor to fuse all saliencies into a single master saliency.
 *
 * @author Michael Zillich (MZ)
 * @date Oct 2012
 */

#include <boost/foreach.hpp>
#include "display/Display.hpp"
#include "MasterSaliencyProcessor.hpp"

MasterSaliencyProcessor::MasterSaliencyProcessor(const long long& processorId,
        const unsigned int imgWidth, const unsigned int imgHeight, const bool isStereo)
: SaliencyProcessor(processorId, imgWidth, imgHeight, isStereo) {
  visionProcessName = "MasterSaliencyProcessor";
  logger = log4cxx::Logger::getLogger("diarc.imgproc.saliency.MasterSaliencyProcessor");
  ignoreOlderNotifications = false;
}

void MasterSaliencyProcessor::init() {
}

void MasterSaliencyProcessor::cleanup() {
}

void MasterSaliencyProcessor::handleSaliencyNotification(SaliencyNotification::ConstPtr notification) {
  //lock saliencyMaps
  boost::lock_guard<boost::mutex> salamp_lock(saliencyMaps_mutex);

  long long salprocId = notification->notifier->getProcessorId();
  boost::unordered_map<long long, cv::Mat>::iterator salmaps_iter = saliencyMaps.find(salprocId);
  if (salmaps_iter == saliencyMaps.end()) {
    LOG4CXX_WARN(logger, "[haveNewSaliencyMap] received notification from unregistered saliency processor.");
    return;
  }

  //copy new saliency map to local data structure
  notification->saliencyMap.copyTo(salmaps_iter->second);

  //build new combined saliency map
  cv::Mat resultImage = cv::Mat_<float>::ones(img_height, img_width);
  for (salmaps_iter = saliencyMaps.begin(); salmaps_iter != saliencyMaps.end(); ++salmaps_iter) {
    cv::Mat img = salmaps_iter->second;
    cv::multiply(resultImage, img, resultImage);
  }

  Notification::Ptr n(new SaliencyNotification(shared_from_this(), notification->frameNumber, notification->captureData, resultImage));
  sendNotifications(n);
  if (getDisplayFlag()) {
    diarc::Display::displayFrame(resultImage, getDisplayName());
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  //  // NOTE: It's ok to lock the result image here, because processing is really fast in our case
  //  // and we are thus not blocking the mutex for long
  //  boost::mutex::scoped_lock lock(result_mutex);
  //  //boost::mutex::scoped_lock lock2(result_vector_mutex);
  //  // if I get saliency for a _new_ frame, then reset the accumulated saliency to 1
  //  if (frameNum > frameNum) {
  //    resultImage = 1.;
  //    //resultImagesVector.clear();
  //  }
  //  cv::Mat img;
  //  unsigned long long tmp;
  //  if (saliencyProcessor->getLastResultImage(img, tmp)) {
  //    //if (saliencyProcessor->getResultImage(img, frameNum)) {
  //    frameNum = frameNum;
  //    // HACK: for now assume that saliency maps are always multiplied for fusion
  //    cv::multiply(resultImage, img, resultImage);
  //    //resultImagesVector.push_back(img);
  //    Notification::Ptr n(new SaliencyNotification(shared_from_this(), frameNum));
  //    sendNotifications(n);
  //    if (getDisplayFlag()) {
  //      IplImage ipl_img = resultImage;
  //      diarc::Display::displayFrame(&ipl_img, getDisplayName());
  //    }
  //  }
}

void MasterSaliencyProcessor::notifyOfRegistration(VisionProcess::Ptr notifyingSource) {
  //lock saliencyMaps
  boost::lock_guard<boost::mutex> lock(saliencyMaps_mutex);

  long long procId = notifyingSource->getProcessorId();
  boost::unordered_map<long long, cv::Mat>::iterator salmaps_iter = saliencyMaps.find(procId);
  if (salmaps_iter != saliencyMaps.end()) {
    LOG4CXX_WARN(logger, "[notifyOfRegistration] has already been notified of registration.");
    return;
  }

  cv::Mat newSalMap(img_height, img_width, CV_32FC1);
  newSalMap = 1.0;
  saliencyMaps[procId] = newSalMap;
}

void MasterSaliencyProcessor::notifyOfUnregistration(VisionProcess::Ptr notifyingSource) {
  //lock saliencyMaps
  boost::lock_guard<boost::mutex> lock(saliencyMaps_mutex);

  long long procId = notifyingSource->getProcessorId();
  boost::unordered_map<long long, cv::Mat>::iterator salmaps_iter = saliencyMaps.find(procId);
  if (salmaps_iter == saliencyMaps.end()) {
    LOG4CXX_WARN(logger, "[notifyOfUnregistration] was never notified of registration or has already unregistered.");
    return;
  }

  //remove it
  saliencyMaps.erase(salmaps_iter);
}

