/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

//
// Created by evan on 5/8/18.
//

#ifndef BARCODEDETECTOR_HPP
#define BARCODEDETECTOR_HPP

#include "ObjectDetector.hpp"

extern "C" {
#include <zbar.h>
}

#include <vector>

class BarCodeDetector : public ObjectDetector {
public:
  typedef boost::shared_ptr<BarCodeDetector> Ptr;
  typedef boost::shared_ptr<const BarCodeDetector> ConstPtr;

  BarCodeDetector(const long long &processorId, const int imgWidth, const int imgHeight);

  ~BarCodeDetector();

  virtual void loadConfig(const std::string &config);

  typedef struct {
    std::string type;
    std::string data;
    std::vector<cv::Point> location;
  } DecodedObject;

protected:
  virtual void handleCaptureNotification(CaptureNotification::ConstPtr notification);

  virtual void handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification);

private:

  /**
  * Find and decode barcodes and QR codes.
  * @param im
  * @param decodedObjects
  */
  void decode(const cv::Mat &im, std::vector<DecodedObject> &decodedObjects);

  /**
  * Display barcode and QR code location.
  * @param im
  * @param decodedObjects
  */
  void display(const cv::Mat &im, const std::vector<DecodedObject> &decodedObjects);

  //! zbar scanner
  zbar::ImageScanner scanner;

};

#endif //BARCODEDETECTOR_HPP
