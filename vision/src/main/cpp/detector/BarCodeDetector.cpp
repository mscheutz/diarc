/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

//
// Created by evan on 5/8/18.
//
#include "BarCodeDetector.hpp"
#include "display/Display.hpp"

using namespace diarc::stm;

BarCodeDetector::BarCodeDetector(const long long &processorId, const int imgWidth, const int imgHeight)
: ObjectDetector(processorId, imgWidth, imgHeight) {
  visionProcessName = "BarCodeDetector";
  logger = log4cxx::Logger::getLogger("diarc.detector.BarCodeDetector");

  // Configure scanner
  scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
}

BarCodeDetector::~BarCodeDetector() {}

void BarCodeDetector::loadConfig(const std::string &config) {

}

void BarCodeDetector::handleCaptureNotification(CaptureNotification::ConstPtr notification) {
  // Read image
  const cv::Mat im = notification->captureData->frame;

  // Variable for decoded objects
  std::vector<DecodedObject> decodedObjects;

  // Find and decode barcodes and QR codes
  decode(im, decodedObjects);

  MemoryObject::VecPtr newObjects(new MemoryObject::Vec());

  TypesByDescriptorConstPtr descriptors = getDescriptors();
  for (DecodedObject decodedObject : decodedObjects) {
    for (auto descriptor_iter = descriptors->begin(); descriptor_iter != descriptors->end(); ++descriptor_iter) {
      if (descriptor_iter->first.getName().compare(decodedObject.data) == 0) {
        // create MemoryObject for each relevant typeId
        for (auto typeIds_itr = descriptor_iter->second.begin();
             typeIds_itr != descriptor_iter->second.end(); ++typeIds_itr) {
          cv::Mat maskImg = cv::Mat::zeros(im.size(), CV_32F);
          cv::fillConvexPoly(maskImg, decodedObject.location, 1.0f);
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
    display(im, decodedObjects);
  }
}

void BarCodeDetector::handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification) {

}

// Find and decode barcodes and QR codes
void BarCodeDetector::decode(const cv::Mat &im, std::vector<DecodedObject> &decodedObjects) {

  // Convert image to grayscale
  cv::Mat imGray;


  // opencv version 3 -> 4  changes tweaks some namespaces
#if (CV_MAJOR_VERSION < 4)
      cv::cvtColor(im, imGray, CV_BGR2GRAY);
#else
      cv::cvtColor(im, imGray, cv::COLOR_BGR2GRAY);
#endif

  // Wrap image data in a zbar image
  zbar::Image image(im.cols, im.rows, "Y800", (uchar *) imGray.data, im.cols * im.rows);

  // Scan the image for barcodes and QRCodes
  int n = scanner.scan(image);

  // Print results
  for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
    DecodedObject obj;

    obj.type = symbol->get_type_name();
    obj.data = symbol->get_data().substr(7); // http://<embedded-data> --> get embedded data

    // Print type and data
    LOG4CXX_DEBUG(logger, boost::format("Type: %s.") % obj.type);
    LOG4CXX_DEBUG(logger, boost::format("Data: %s.") % obj.data);

    // Obtain location
    for (int i = 0; i < symbol->get_location_size(); i++) {
      obj.location.push_back(cv::Point(symbol->get_location_x(i), symbol->get_location_y(i)));
    }

    decodedObjects.push_back(obj);
  }
}

// Display barcode and QR code location
void BarCodeDetector::display(const cv::Mat &im, const std::vector<DecodedObject> &decodedObjects) {
  im.copyTo(displayFrame);

  // Loop over all decoded objects
  for (int i = 0; i < decodedObjects.size(); i++) {
    std::vector<cv::Point> points = decodedObjects[i].location;
    std::vector<cv::Point> hull;

    // If the points do not form a quad, find convex hull
    if (points.size() > 4) {
      cv::convexHull(points, hull);
    } else {
      hull = points;
    }

    // Number of points in the convex hull
    int n = hull.size();

    for (int j = 0; j < n; j++) {
      line(displayFrame, hull[j], hull[(j + 1) % n], cv::Scalar(255, 0, 0), 3);
    }

  }

  // Display results
  diarc::Display::displayFrame(displayFrame, getDisplayName());

}
