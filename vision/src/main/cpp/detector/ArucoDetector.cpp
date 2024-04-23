/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "ArucoDetector.hpp"

#include "display/Display.hpp"
#include "capture/util/CaptureUtilities.hpp"
#include "common/notification/CaptureNotification.hpp"

#include <jsoncpp/json/reader.h>

using namespace ade::stm;

// pre-load string to enum map for loading config
#if (CV_MAJOR_VERSION >= 4 && CV_MINOR_VERSION >= 7)
  std::unordered_map<std::string,cv::aruco::PredefinedDictionaryType> const ArucoDetector::dictionaryTable = {
#else
std::unordered_map<std::string,cv::aruco::PREDEFINED_DICTIONARY_NAME> const ArucoDetector::dictionaryTable = {
#endif
    {"DICT_4X4_50", cv::aruco::DICT_4X4_50}, {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
    {"DICT_4X4_250", cv::aruco::DICT_4X4_250}, {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
    {"DICT_5X5_50", cv::aruco::DICT_5X5_50}, {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
    {"DICT_5X5_250", cv::aruco::DICT_5X5_250}, {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
    {"DICT_6X6_50", cv::aruco::DICT_6X6_50}, {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
    {"DICT_6X6_250", cv::aruco::DICT_6X6_250}, {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
    {"DICT_7X7_50", cv::aruco::DICT_7X7_50}, {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
    {"DICT_7X7_250", cv::aruco::DICT_7X7_250}, {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
    {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL}
};


ArucoDetector::ArucoDetector(const long long &processorId, const int imgWidth, const int imgHeight)
    : ObjectDetector(processorId, imgWidth, imgHeight) {
  visionProcessName = "ArucoDetector";
  logger = log4cxx::Logger::getLogger("ade.detector.ArucoDetector");
}

ArucoDetector::~ArucoDetector() {
}

void ArucoDetector::loadConfig(const std::string &configFile) {
  //get directory
  unsigned found = configFile.find_last_of("/\\"); // check if config file is filename or path
  std::string dir = configFile.substr(0, found + 1);
  std::string model_file;
  std::string backEnd;

  Json::CharReaderBuilder builder;
  Json::Value root;
  JSONCPP_STRING errs;
  std::ifstream fileStream(configFile, std::ifstream::binary);
  if (!Json::parseFromStream(builder, fileStream, &root, &errs)) {
    LOG4CXX_ERROR(logger, boost::format("Error parsing json file: %s") % configFile.c_str());
    return;
  }

  // predicate descriptor options
  std::string parentType = root["processor"]["type"].asString();
  int arity = root["processor"]["arity"].asInt();
  for (auto predicateValue: root["predicates"]) {
    std::string name = predicateValue["name"].asString();
    int id = predicateValue["id"].asInt();
    classToId.insert({name, id});
    idToClass.insert({id, name});
  }

  // configuration options
#if (CV_MAJOR_VERSION >= 4 && CV_MINOR_VERSION >= 7)
  parameters = cv::aruco::DetectorParameters();
#else
  parameters = cv::aruco::DetectorParameters::create();
#endif

  Json::Value config = root["config"];
  std::string dictionaryType = config["dictionary"].asString();
  auto it = dictionaryTable.find(dictionaryType);
  if (it != dictionaryTable.end()) {
    LOG4CXX_DEBUG(logger, "Using Aruco dictionary type: " + dictionaryType);
    dictionary = cv::aruco::getPredefinedDictionary(it->second);
  } else {
    LOG4CXX_ERROR(logger, "Invalid Aruco dictionary type: " + dictionaryType);
  }

#if (CV_MAJOR_VERSION >= 4 && CV_MINOR_VERSION >= 7)
  detector = cv::aruco::ArucoDetector(dictionary, parameters);
#endif
}

void ArucoDetector::handleCaptureNotification(CaptureNotification::ConstPtr notification) {

  //get captured frames info
  const cv::Mat currFrame = notification->captureData->frame;

  // call aruco detector
  std::vector< int > markerIds;
  std::vector< std::vector<cv::Point2f> > markerCorners;

#if (CV_MAJOR_VERSION >= 4 && CV_MINOR_VERSION >= 7)
  detector.detectMarkers(currFrame, markerCorners, markerIds);
#else
  cv::aruco::detectMarkers(currFrame, dictionary, markerCorners, markerIds, parameters);
#endif

  LOG4CXX_DEBUG(logger, boost::format("Detected Aruco Tag(s). Num ids: %ld. Num corners: %ld.") % markerIds.size() % markerCorners.size());

  // fill memory objects with detection results
  MemoryObject::VecPtr newObjects(new MemoryObject::Vec());
  TypesByDescriptorConstPtr descriptors = getDescriptors();
  for (int i = 0; i < markerIds.size(); ++i) {
    // check that the detected id is one we care about (i.e., one in the aruco.json config)
    int detectedId = markerIds.at(i);
    LOG4CXX_DEBUG(logger, boost::format("Detected Aruco Id: %ld.") % detectedId);
    auto label_itr = idToClass.find(detectedId);
    if (label_itr == idToClass.end()) {
      continue;
    }

    for (auto descriptor_iter = descriptors->begin(); descriptor_iter != descriptors->end(); ++descriptor_iter) {
      std::string label = label_itr->second;
      if (descriptor_iter->first.getName().compare(label) == 0) {
        // create MemoryObject for each relevant typeId
        for (auto typeIds_itr = descriptor_iter->second.begin();
             typeIds_itr != descriptor_iter->second.end(); ++typeIds_itr) {
          cv::Mat maskImg = cv::Mat::zeros(currFrame.size(), CV_32F);

          // convert corners to vector of cv::Point
          std::vector<cv::Point> corners;
          for (auto pt : markerCorners.at(i)) {
            corners.push_back(pt);
          }

          cv::fillConvexPoly(maskImg, corners, 1.0f);
          MemoryObjectMask::Ptr objectMask(new MemoryObjectMask(notification->captureData, maskImg));
          MemoryObject::Ptr newObject(
              new MemoryObject(*typeIds_itr, descriptor_iter->first.getArg(0), notification->captureData, objectMask));
          newObject->addValidationResult(0.9, descriptor_iter->first);
          newObjects->push_back(newObject);
        }
      }
    }
  }

  sendDetectionNotifications(newObjects);

  // Display location
  if (getDisplayFlag()) {
    currFrame.copyTo(displayFrame);
    cv::aruco::drawDetectedMarkers(displayFrame, markerCorners, markerIds);
    ade::Display::displayFrame(displayFrame, getDisplayName());
  }
}
