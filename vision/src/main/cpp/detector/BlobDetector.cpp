/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "BlobDetector.hpp"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <fstream>

#include "capture/util/CaptureUtilities.hpp"
#include "display/Display.hpp"

using namespace ade::stm;

BlobDetector::BlobDetector(const long long &processorId, const int imgWidth, const int imgHeight)
        : ObjectDetector(processorId, imgWidth, imgHeight),
          colorDetector(new FastBlobDetector()),
          USE_FAST_BLOB_DETECTION_BLUR(false),
          USE_FAST_BLOB_DETECTION_BLUR_AMOUNT(0) {
  colorDetector->initBlobDetection(img_width, img_height);
  visionProcessName = "BlobDetector";
  logger = log4cxx::Logger::getLogger("ade.detector.BlobDetector");
}

BlobDetector::~BlobDetector() {

}

void BlobDetector::loadConfig(const std::string &configFile) {

  //get directory
  unsigned found = configFile.find_last_of("/\\");
  std::string dir = configFile.substr(0, found + 1);

  // populate tree structure pt
  using boost::property_tree::ptree;
  ptree pt;
  read_xml(configFile, pt);

  // parse xml file

  BOOST_FOREACH(ptree::value_type const& predicateNode, pt.get_child("processor")) {
          if (predicateNode.first.compare("predicate") == 0) {
            std::string descriptorName = predicateNode.second.get<std::string>("<xmlattr>.name", "unknown");
            LOG4CXX_DEBUG(logger, boost::format("[loadConfig] descriptorName: %s.") % descriptorName);

            std::string colorFilename =
                    dir + predicateNode.second.get_child("config").get<std::string>("<xmlattr>.filename", "unknown");
            LOG4CXX_DEBUG(logger, boost::format("[loadConfig] colorFilename: %s.") % colorFilename);

            std::ifstream file(colorFilename.c_str());
            if (!file.good()) {
              LOG4CXX_ERROR(logger, boost::format("[loadConfig] Could not open file: %s.") % colorFilename);
              continue;
            }

            std::string line;
            int rl, rh, gl, gh, bl, bh, min, max;
            while (getline(file, line)) {
              int retNum = sscanf(line.c_str(), "(%d,%d,%d %*s %d,%d,%d) [%d %*s %d]", &rl, &gl, &bl, &rh, &gh, &bh,
                                  &min, &max);
              LOG4CXX_DEBUG(logger, boost::format("load colors return num: %d") % retNum);
              LOG4CXX_DEBUG(logger,
                            boost::format("colors (%d,%d,%d to %d,%d,%d) [%d to %d]") % rl % gl % bl % rh % gh % bh %
                            min % max);
              if (8 == retNum) {
                colorDetector->addColor(descriptorName, rl, rh, gl, gh, bl, bh, min, max);
              } else {
                LOG4CXX_ERROR(logger, boost::format("[loadConfig] Problem loading color config: %s.") % colorFilename);
              }
            }
            file.close();
          }
        }
}

void BlobDetector::handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification) {
  LOG4CXX_DEBUG(logger, "[handleMemoryObjectNotification] method entered.");
  MemoryObject::VecPtr newBlobs = detectBlobs(notification->object->getDetectionMask()->getObjectImage(),
                                              notification->object->getCaptureData());

  MemoryObject::Vec::const_iterator blobs_itr;
  int childrenAdded = 0;
  for (blobs_itr = newBlobs->begin(); blobs_itr != newBlobs->end(); ++blobs_itr) {
    if ((*blobs_itr)->getTypeId() == notification->object->getTypeId()) {
      notification->object->addChild(*blobs_itr);
      ++childrenAdded;
    }
  }

  // if children added, send along object that had children added to it
  LOG4CXX_DEBUG(logger, boost::format("[handleMemoryObjectNotification] num children added: %d.") % childrenAdded);
  if (childrenAdded > 0) {
    sendDetectionNotifications(notification->object);
  }
}

void BlobDetector::handleCaptureNotification(CaptureNotification::ConstPtr notification) {
  LOG4CXX_DEBUG(logger, "[handleCaptureNotification] method entered.");
  MemoryObject::VecPtr newBlobs = detectBlobs(notification->captureData->frame, notification->captureData);

  // send newly detected objects notifications
  sendDetectionNotifications(newBlobs);
}

MemoryObject::VecPtr BlobDetector::detectBlobs(const cv::Mat &currFrame, CaptureData::ConstPtr capture) {
  // make a copy (in case do blur to it, or whatever else...)
  currFrame.copyTo(displayFrame);

  // blur (or otherwise post-process) based on initialized flags:
  performFastBlobDetectionImagePreProcessing(displayFrame);

  // This done, DETECT BLOBS:
  std::vector<BlobInfo> nblobs = colorDetector->getBlobs(displayFrame, 0, 0, displayFrame.cols,
                                                         displayFrame.rows);
  LOG4CXX_DEBUG(logger, boost::format("[detectBlobs] num blobs: %d.") % nblobs.size());

  MemoryObject::VecPtr newBlobs(new MemoryObject::Vec());

  // get current descriptors to process
  TypesByDescriptorConstPtr descriptors = getDescriptors();
  TypesByDescriptor::const_iterator descriptor_iter;
  std::tr1::unordered_set<long long>::const_iterator typeIds_itr;

  //add detected blobs
  for (int i = 0; i < nblobs.size(); i++) {
    if (logger->isDebugEnabled()) {
      LOG4CXX_DEBUG(logger, boost::format(
              "[handleCaptureNotification] blobs[%d] color: %s %d %d area: %f UL: (%d,%d) LR: (%d,%d)")
                            % i % nblobs[i].colorlabel % nblobs[i].color % nblobs[i].colorID
                            % nblobs[i].area % nblobs[i].UL_x % nblobs[i].UL_y % nblobs[i].LR_x % nblobs[i].LR_y);
    }

    //check if color has been requested by any searches
    for (descriptor_iter = descriptors->begin(); descriptor_iter != descriptors->end(); ++descriptor_iter) {
      LOG4CXX_DEBUG(logger, boost::format("[handleCaptureNotification] blobs[%d] desired: %s. found: %s.") % i %
                            descriptor_iter->first.getName() % nblobs[i].colorlabel);
      if (descriptor_iter->first.getName().compare(nblobs[i].colorlabel) == 0) {
        LOG4CXX_DEBUG(logger, boost::format("[handleCaptureNotification] blobs[%d] label found: %s.") % i %
                              nblobs[i].colorlabel);
        LOG4CXX_DEBUG(logger,
                      boost::format("[handleCaptureNotification] blobs[%d] bbox:(%d,%d,%d,%d).") % i % nblobs[i].UL_x %
                      nblobs[i].UL_y % (nblobs[i].LR_x - nblobs[i].UL_x) % (nblobs[i].LR_y - nblobs[i].UL_y));

        // EAK: for some unknown reason, there are occasionally blobs of the entire image
        // this weeds those out
        // EAK: this should be fixed now
        //if (nblobs[i].UL_x == 0 && nblobs[i].UL_y == 0
        //    && ((nblobs[i].LR_x - nblobs[i].UL_x) == (currFrame.cols-1))
        //    && ((nblobs[i].LR_y - nblobs[i].UL_y) == (currFrame.rows-1))) {
        //  LOG4CXX_ERROR(logger, "Blob size matches frame size...ignoring.");
        //  continue;
        //}

        // create MemoryObject for each relevant typeId
        for (typeIds_itr = descriptor_iter->second.begin();
             typeIds_itr != descriptor_iter->second.end(); ++typeIds_itr) {
          cv::Rect bb(nblobs[i].UL_x, nblobs[i].UL_y, (nblobs[i].LR_x - nblobs[i].UL_x),
                      (nblobs[i].LR_y - nblobs[i].UL_y));
          MemoryObject::Ptr newBlob(new MemoryObject(*typeIds_itr, descriptor_iter->first.getArg(0), capture, bb));

          //TODO: calculate actual confidence value
          newBlob->addValidationResult(0.9, descriptor_iter->first);

          //add blob to stack
          newBlobs->push_back(newBlob);
        }
      }
    }
  }

  //draw on blob boxes if displaying blob info
  if (getDisplayFlag()) {
    MemoryObject::Vec::iterator newBlobIter;
    int lineThickness = 2;

    for (newBlobIter = newBlobs->begin(); newBlobIter != newBlobs->end(); ++newBlobIter) {
      const cv::Rect &rect = (*newBlobIter)->getDetectionMask()->getBoundingBox();

      //add black and white box to make box easier to see
      cv::rectangle(displayFrame, cv::Point(rect.x + lineThickness, rect.y + lineThickness),
                  cv::Point(rect.x + rect.width - lineThickness, rect.y + rect.height - lineThickness),
                  CV_RGB(0, 0, 0),
                  lineThickness, 8, 0);
      cv::rectangle(displayFrame, cv::Point(rect.x, rect.y),
                  cv::Point(rect.x + rect.width, rect.y + rect.height),
                  CV_RGB(255, 255, 255),
                  lineThickness, 8, 0);
    }

    ade::Display::displayFrame(displayFrame, getDisplayName());
  }

  return newBlobs;
}

void BlobDetector::setColors(const int size, const char *const *names, const int redLow[], const int redHigh[],
                             const int greenLow[], const int greenHigh[], const int blueLow[], const int blueHigh[]) {
  int i = 0;
  std::string names_str[size];
  while (names[i]) {
    //printf("names[%d] = %s\n", i, names[i]);
    names_str[i] = names[i];
    i++;
  }
  colorDetector->setColors(size, names_str, redLow, redHigh, greenLow, greenHigh, blueLow, blueHigh);
}

void BlobDetector::setSizeMinsMaxs(const int size, const int mins[], const int maxs[]) {
  colorDetector->setMinsMaxs(size, mins, maxs);
}

void BlobDetector::setBlur(const bool blurFlag, const int blurAmount) {
  USE_FAST_BLOB_DETECTION_BLUR = blurFlag;
  USE_FAST_BLOB_DETECTION_BLUR_AMOUNT = blurAmount;
}

void BlobDetector::performFastBlobDetectionImagePreProcessing(cv::Mat theFrame) {
  if (USE_FAST_BLOB_DETECTION_BLUR)
    cv::GaussianBlur(theFrame, theFrame,
             cv::Size(USE_FAST_BLOB_DETECTION_BLUR_AMOUNT, USE_FAST_BLOB_DETECTION_BLUR_AMOUNT),
             0);
}

const int BlobDetector::getNumColors() const {
  return colorDetector->getNumColors();
}

void
BlobDetector::getColors(const int size, int redLow[], int redHigh[], int greenLow[], int greenHigh[], int blueLow[],
                        int blueHigh[]) const {
  colorDetector->getColors(size, redLow, redHigh, greenLow, greenHigh, blueLow, blueHigh);
}

std::string BlobDetector::getColorLabel(const int index) const {
  return colorDetector->getColorLabel(index);
}

void BlobDetector::getSizeMinsMaxs(const int size, int mins[], int maxs[]) const {
  colorDetector->getSizeMinsMaxs(size, mins, maxs);
}

void BlobDetector::getBlur(bool &blurFlag, int &blurAmount) const {
  blurFlag = USE_FAST_BLOB_DETECTION_BLUR;
  blurAmount = USE_FAST_BLOB_DETECTION_BLUR_AMOUNT;
}
