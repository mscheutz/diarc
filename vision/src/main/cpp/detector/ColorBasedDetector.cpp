/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "ColorBasedDetector.hpp"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <fstream>

#include "capture/util/CaptureUtilities.hpp"
#include "display/Display.hpp"

using namespace ade::stm;

ColorBasedDetector::ColorBasedDetector(const long long &processorId, const int imgWidth, const int imgHeight)
        : ObjectDetector(processorId, imgWidth, imgHeight) {
  visionProcessName = "ColorBasedDetector";
  logger = log4cxx::Logger::getLogger("ade.detector.ColorBasedDetector");

  colorSpaceMap["HLS"] = cv::COLOR_BGR2HLS;
  colorSpaceMap["HSV"] = cv::COLOR_BGR2HSV;
  colorSpaceMap["Lab"] = cv::COLOR_BGR2Lab;
  colorSpaceMap["RGB"] = cv::COLOR_BGR2RGB;
  colorSpaceMap["YUV"] = cv::COLOR_BGR2YUV;
}

ColorBasedDetector::~ColorBasedDetector() {

}

void ColorBasedDetector::loadConfig(const std::string &configFile) {

  //get directory
  unsigned found = configFile.find_last_of("/\\");
  std::string dir = configFile.substr(0, found + 1);

  // populate tree structure pt
  using boost::property_tree::ptree;
  ptree pt;
  read_xml(configFile, pt);

  // parse xml file

  ColorRange colorRange;
  int x,y,z;
  BOOST_FOREACH(ptree::value_type const& predicateNode, pt.get_child("processor")) {
          if (predicateNode.first.compare("predicate") == 0) {
            std::string descriptorName = predicateNode.second.get<std::string>("<xmlattr>.name", "unknown");
            LOG4CXX_DEBUG(logger, boost::format("[loadConfig] descriptorName: %s.") % descriptorName);
            bool supertype = predicateNode.second.get<bool>("<xmlattr>.supertype", false);
            LOG4CXX_DEBUG(logger, boost::format("[loadConfig] supertype: %s.") % (supertype ? "true" : "false"));
            if (!supertype) {
              std::string colorspace = predicateNode.second.get<std::string>("colorspace");
              colorRange.colorSpace = colorSpaceMap[colorspace];

              std::string start_line = predicateNode.second.get<std::string>("start");
              sscanf(start_line.c_str(), "%d %d %d", &x, &y, &z);
              colorRange.start = cv::Scalar(x,y,z);

              std::string end_line = predicateNode.second.get<std::string>("end");
              sscanf(end_line.c_str(), "%d %d %d", &x, &y, &z);
              colorRange.end = cv::Scalar(x,y,z);

              LOG4CXX_DEBUG(logger, boost::format("[loadConfig] colorspace: %s. Range %s to %s.") % colorspace % start_line % end_line);

              std::vector<ColorRange> &colors = colorMap[descriptorName];
              colors.push_back(colorRange);
            } else {
              superTypes.insert(descriptorName);
            }
          }
        }
}

void ColorBasedDetector::handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification) {
  LOG4CXX_DEBUG(logger, "[handleMemoryObjectNotification] method entered.");
  MemoryObject::VecPtr newObjects = detectObjects(notification->object->getDetectionMask()->getObjectImage(),
                                              notification->object->getCaptureData());

  int childrenAdded = 0;
  for (auto& newObject : (*newObjects)) {
    if (newObject->getTypeId() == notification->object->getTypeId()) {
      notification->object->addChild(newObject);
      ++childrenAdded;
    }
  }

  // if children added, send along object that had children added to it
  LOG4CXX_DEBUG(logger, boost::format("[handleMemoryObjectNotification] num children added: %d.") % childrenAdded);
  if (childrenAdded > 0) {
    sendDetectionNotifications(notification->object);
  }
}

void ColorBasedDetector::handleCaptureNotification(CaptureNotification::ConstPtr notification) {
  LOG4CXX_DEBUG(logger, "[handleCaptureNotification] method entered.");
  MemoryObject::VecPtr newObjects = detectObjects(notification->captureData->frame, notification->captureData);

  // send newly detected objects notifications
  sendDetectionNotifications(newObjects);
}

MemoryObject::VecPtr ColorBasedDetector::detectObjects(const cv::Mat &currFrame, CaptureData::ConstPtr capture) {

  MemoryObject::VecPtr newObjects(new MemoryObject::Vec());

  // get current descriptors to process
  TypesByDescriptorConstPtr descriptors = getDescriptors();
  for (const auto& descriptor : (*descriptors)) {

    // superType check
    std::string targetColor = descriptor.first.getName();
    if (superTypes.find(targetColor) != superTypes.end()) {
      // looking for a supertype -- check all colors
      for (const auto& colorEntry : colorMap) {
        std::vector<MemoryObject::Ptr> mos = getMemoryObjects(currFrame, capture, descriptor.first, descriptor.second, colorEntry.second);
        // add color descriptor with matching variable to supertype
        for (auto& mo : mos) {
          PredicateHelper colorDescriptor(colorEntry.first + "(" + descriptor.first.getArg(0) + ")");
          mo->addValidationResult(0.9f, colorDescriptor);
        }

        newObjects->insert(newObjects->end(), mos.begin(), mos.end());
      }
    } else {
      const auto& color = colorMap.find(targetColor);
      if (color != colorMap.end()) {
        std::vector<MemoryObject::Ptr> mos = getMemoryObjects(currFrame, capture, descriptor.first, descriptor.second, color->second);
        newObjects->insert(newObjects->end(), mos.begin(), mos.end());
      } else {
        LOG4CXX_ERROR(logger, "Color not in color map: " + targetColor);
        continue;
      }

    }

  }

  LOG4CXX_DEBUG(logger, boost::format("Detected %lu objects.") % newObjects->size());

  //draw on blob boxes if displaying blob info
  if (getDisplayFlag()) {
    int lineThickness = 2;
    currFrame.copyTo(displayFrame);

    for (const auto& mo : (*newObjects)) {
      const cv::Rect &rect = mo->getDetectionMask()->getBoundingBox();

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

  return newObjects;
}

std::vector<MemoryObject::Ptr> ColorBasedDetector::getMemoryObjects(const cv::Mat &currFrame,
                                                               CaptureData::ConstPtr capture,
                                                               const PredicateHelper &descriptor,
                                                               const std::tr1::unordered_set<long long> &typeIds,
                                                               const std::vector<ColorRange> &colorRanges) {

  // get color mask for entire frame
  cv::Mat colorMask = getColorMask(currFrame, colorRanges);

//  ade::Display::createWindowIfDoesNotExist("colorMask");
//  ade::Display::displayFrame(colorMask, "colorMask");

  // perform morphological operation to minimize holes and number of clusters
  int kernel_size = (img_width/320.0) * 5; // scale to image size;
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Point(kernel_size,kernel_size));
  cv::morphologyEx(colorMask, colorMask, cv::MORPH_CLOSE, kernel);

//  ade::Display::createWindowIfDoesNotExist("colorMask_morph");
//  ade::Display::displayFrame(colorMask, "colorMask_morph");

  // segment mask frame into clusters
  cv::Mat labelImage(currFrame.size(), CV_32S);
  cv::Mat stat, centroid;
  int nLabels = cv::connectedComponentsWithStats(colorMask, labelImage, stat, centroid, 8);

  std::vector<MemoryObject::Ptr> newObjects;
  cv::Mat componentMask;
  for (int i = 0; i < nLabels; i++) {
    cv::Rect bb(cv::Point(stat.at<int>(i, cv::CC_STAT_LEFT), stat.at<int>(i, cv::CC_STAT_TOP)),
                cv::Size(stat.at<int>(i, cv::CC_STAT_WIDTH), stat.at<int>(i, cv::CC_STAT_HEIGHT)));

    // for some reason, there are blobs of the entire image. this weeds those out
    if (bb.x == 0 && bb.y == 0
        && (bb.width == currFrame.cols)
        && (bb.height == currFrame.rows)) {
      LOG4CXX_DEBUG(logger, "Blob size matches frame size...ignoring.");
      continue;
    }

    // ignore small bounding boxes
    if ((bb.width < 10) || (bb.height < 10)) {
      LOG4CXX_DEBUG(logger, "Blob size less than 10x10...ignoring.");
      continue;
    }

    // set mask for connected component
    componentMask = cv::Mat::zeros(currFrame.size(), CV_8U);
    colorMask(bb).copyTo(componentMask(bb));

    if (logger->isDebugEnabled()) {
      LOG4CXX_DEBUG(logger, boost::format("bounding box: %d, %d, %d, %d.") % bb.x % bb.y % bb.width % bb.height);
      LOG4CXX_DEBUG(logger, boost::format("num non-zero pixels, %d.") % cv::countNonZero(componentMask));
    }

    //ade::Display::createWindowIfDoesNotExist("colorMask");
    //ade::Display::displayFrame(colorMask, "colorMask");

    // create MemoryObject for each relevant typeId
    for (const auto& typeId : typeIds) {
      MemoryObject::Ptr newObject(new MemoryObject(typeId, descriptor.getArg(0), capture, MemoryObjectMask::Ptr(new MemoryObjectMask(capture, componentMask))));

      //TODO: calculate actual confidence value
      newObject->addValidationResult(0.9, descriptor);

      //add object to stack
      newObjects.push_back(newObject);
    }
  }

  return newObjects;
}

cv::Mat ColorBasedDetector::getColorMask(const cv::Mat &currFrame, const std::vector<ColorRange> &colorRanges) {

  cv::Mat finalMask = cv::Mat::zeros(currFrame.size(), CV_8U);
  cv::Mat tmpMask;
  cv::Mat convertedFrame;

  for (const auto& range: colorRanges) {
    //Converting image from BGR to target color space.
    cv::cvtColor(currFrame, convertedFrame, range.colorSpace);

    // find color mask for range
    cv::inRange(convertedFrame, range.start, range.end, tmpMask);

    // overlay all masks
    finalMask += tmpMask;
  }

  return finalMask;
}
