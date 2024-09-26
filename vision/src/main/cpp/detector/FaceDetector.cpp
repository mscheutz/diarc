/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "FaceDetector.hpp"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>

#include "display/Display.hpp"
#include "capture/util/CaptureUtilities.hpp"
#include "common/notification/CaptureNotification.hpp"

using namespace diarc::stm;

FaceDetector::FaceDetector(const long long &processorId, const int imgWidth, const int imgHeight)
        : ObjectDetector(processorId, imgWidth, imgHeight),
          cascade_names(),
          cascades(),
          frame_scaled(imgWidth / 2, imgHeight / 2, CV_8UC3) // used for haar (face analysis)
{
  visionProcessName = "FaceDetector";
  logger = log4cxx::Logger::getLogger("diarc.detector.FaceDetector");
}

FaceDetector::~FaceDetector() {
}

void FaceDetector::loadConfig(const std::string &configFile) {
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
            std::string functorName = predicateNode.second.get<std::string>("<xmlattr>.name", "unknown");
            LOG4CXX_DEBUG(logger, boost::format("[loadConfig] functorName: %s.") % functorName);

            std::string cascadeFilename = predicateNode.second.get_child("config").get<std::string>(
                    "<xmlattr>.filename", "unknown");
            cascade_names[functorName] = dir + cascadeFilename;
            LOG4CXX_DEBUG(logger, boost::format("[loadConfig] cascadeFilename: %s.") % cascade_names[functorName]);
          }
        }
}

void FaceDetector::handleCaptureNotification(CaptureNotification::ConstPtr notification) {

  //get captured frames info
  const cv::Mat currFrame = notification->captureData->frame;

  //Perform Haar Detection
  cv::pyrDown(currFrame, frame_scaled);

  //while not currently done, this class could be generalized to use haar 
  //wavelets for more than just face detection
  // get current descriptors to process
  TypesByDescriptorConstPtr descriptors = getDescriptors();
  TypesByDescriptor::const_iterator descriptor_iter;
  std::tr1::unordered_set<long long>::const_iterator typeIds_itr;
  MemoryObject::VecPtr newFaces(new MemoryObject::Vec());
  for (descriptor_iter = descriptors->begin(); descriptor_iter != descriptors->end(); ++descriptor_iter) {

    //initialize cascade if not already 
    boost::shared_ptr<cv::CascadeClassifier> cascade = cascades[descriptor_iter->first.getName()];
    if (!cascade) {
      std::string filename = cascade_names[descriptor_iter->first.getName()];
      cascade = boost::shared_ptr<cv::CascadeClassifier>(new cv::CascadeClassifier(filename));
      if (cascade->empty()) {
        LOG4CXX_ERROR(logger, boost::format("Could not load classifier: %s.") % filename);
        cascade_names.erase(descriptor_iter->first.getName());
        continue;
      }
    }

    //cv::equalizeHist(frame_scaled_mat, frame_scaled_mat);
    std::vector<cv::Rect> detectedObjects;
    cascade->detectMultiScale(frame_scaled, detectedObjects, 1.1, 2, 0, cv::Size(30, 30));

    for (int i = 0; i < detectedObjects.size(); ++i) {
      cv::Rect r = detectedObjects[i];

      //The next line keeps the two types of Haar face detections from "fighting" by filling in any detectedObjects found by the first face cascade.
      if (r.width > 0 && r.height > 0 && r.width < img_width && r.height < img_height) {

        r.x *= 2;
        r.y *= 2;
        r.width *= 2;
        r.height *= 2;

        // create MemoryObject for each relevant typeId
        for (typeIds_itr = descriptor_iter->second.begin();
             typeIds_itr != descriptor_iter->second.end(); ++typeIds_itr) {
          MemoryObject::Ptr newFace(
                  new MemoryObject(*typeIds_itr, descriptor_iter->first.getArg(0), notification->captureData, r));
          newFace->addValidationResult(0.9, descriptor_iter->first);
          newFaces->push_back(newFace);
        }
      }
    }
  }

  sendDetectionNotifications(newFaces);

  // draw face bounding boxes
  if (getDisplayFlag()) {
    currFrame.copyTo(displayFrame);

    MemoryObject::Vec::const_iterator newFaceItr;
    for (newFaceItr = newFaces->begin(); newFaceItr != newFaces->end(); ++newFaceItr) {
      const cv::Rect &faceRect = (*newFaceItr)->getDetectionMask()->getBoundingBox();
      cv::rectangle(displayFrame, cv::Point(faceRect.x, faceRect.y),
                    cv::Point(faceRect.x + faceRect.width, faceRect.y + faceRect.height),
                    CV_RGB(255, 0, 0), 2, 8, 0);
    }

    diarc::Display::displayFrame(displayFrame, getDisplayName());
  }
}
