/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "HoughDetector.hpp"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>

#include "display/Display.hpp"
#include "capture/util/CaptureUtilities.hpp"

using namespace diarc::stm;

HoughDetector::HoughDetector(const long long& processorId, const int imgWidth, const int imgHeight)
: ObjectDetector(processorId, imgWidth, imgHeight),
genHoughTrnfMap() {
  visionProcessName = "HoughDetector";
  logger = log4cxx::Logger::getLogger("diarc.detector.HoughDetector");
}

HoughDetector::~HoughDetector() {
}

void HoughDetector::loadConfig(const std::string& configFile) {
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
      std::string functorName = predicateNode.second.get<std::string> ("<xmlattr>.name", "unknown");
      LOG4CXX_DEBUG(logger, boost::format("[loadConfig] functorName: %s.") % functorName);

      //get all image templates for descriptor

      BOOST_FOREACH(ptree::value_type const& imageNode, predicateNode.second) {
        if (imageNode.first.compare("image") == 0) {
          std::string image = static_cast<std::string> (imageNode.second.data());
          LOG4CXX_DEBUG(logger, boost::format("[loadConfig] dir: %s. image: %s") % dir % image);
          std::string imageFilename = dir + image;

          //TODO: allow single descriptor to have multiple templates/transforms
          boost::shared_ptr<GenHoughTrnf> genHoughTrnf(new GenHoughTrnf);
          genHoughTrnf->createRtable(imageFilename);
          genHoughTrnfMap[functorName] = genHoughTrnf;
        }
      }
    }
  }
}

void HoughDetector::handleCaptureNotification(CaptureNotification::ConstPtr notification) {
  LOG4CXX_TRACE(logger, "[houghDetection] method entered.");

  // get captured frames info
  const cv::Mat currFrame = notification->captureData->frame;

  // to know if display frame should be reset
  bool firstDisplayIteration = true;

  // perform hough detection (for all descriptors)
  TypesByDescriptorConstPtr descriptors = getDescriptors();
  TypesByDescriptor::const_iterator descriptor_iter;
  boost::unordered_map < std::string, boost::shared_ptr < GenHoughTrnf > > ::iterator hough_itr;
  for (descriptor_iter = descriptors->begin(); descriptor_iter != descriptors->end(); ++descriptor_iter) {
    std::string label = descriptor_iter->first.getName();
    std::string variableName = descriptor_iter->first.getArg(0);
    hough_itr = genHoughTrnfMap.find(label);

    if (hough_itr != genHoughTrnfMap.end()) {
      LOG4CXX_TRACE(logger, "[houghDetection] detecting...");
      std::vector<cv::Rect> rects = hough_itr->second->detect(currFrame);
      LOG4CXX_TRACE(logger, "[houghDetection] done.");
      MemoryObject::VecPtr newObjects = createMemoryObjects(variableName, descriptor_iter->second, rects, notification);

      // send newly detected objects notifications
      sendDetectionNotifications(newObjects);

      // draw object bounding boxes
      if (getDisplayFlag()) {
        if (firstDisplayIteration) {
          firstDisplayIteration = false;
          currFrame.copyTo(displayFrame);
        }

        MemoryObject::Vec::const_iterator newObjectItr;
        for (newObjectItr = newObjects->begin(); newObjectItr != newObjects->end(); ++newObjectItr) {
          const cv::Rect& faceRect = (*newObjectItr)->getDetectionMask()->getBoundingBox();
          cv::rectangle(displayFrame, cv::Point(faceRect.x, faceRect.y),
                  cv::Point(faceRect.x + faceRect.width, faceRect.y + faceRect.height),
                  CV_RGB(255, 0, 0), 2, 8, 0);
        }

        diarc::Display::displayFrame(displayFrame, getDisplayName());
      }
    }
  }
}

MemoryObject::VecPtr HoughDetector::createMemoryObjects(const std::string& variableName,
        const std::tr1::unordered_set<long long>& typeIds,
        const std::vector<cv::Rect>& rects,
        CaptureNotification::ConstPtr capture) {
  LOG4CXX_TRACE(logger, "[createMemoryObjects] method entered.");

  MemoryObject::VecPtr newObjects(new MemoryObject::Vec());
  std::vector<cv::Rect>::const_iterator rect_itr;
  std::tr1::unordered_set<long long>::const_iterator typeIds_itr;
  for (rect_itr = rects.begin(); rect_itr != rects.end(); ++rect_itr) {
    // construct MO for each typeId
    for (typeIds_itr = typeIds.begin(); typeIds_itr != typeIds.end(); ++typeIds_itr) {
      MemoryObject::Ptr newObject(new MemoryObject(*typeIds_itr, variableName, capture->captureData, *rect_itr));
      newObjects->push_back(newObject);
    }
  }

  return newObjects;
}
