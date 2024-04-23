/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef BUTTONDETECTOR_HPP
#define BUTTONDETECTOR_HPP

#include "NeuralDetector.hpp"

class ButtonDetector : public NeuralDetector {
  public:
    ButtonDetector(const long long &processorId, const int imgWidth, const int imgHeight);
    ~ButtonDetector();
  protected:
    virtual void handleCaptureNotification(CaptureNotification::ConstPtr notification);
  private:
      double rows;
      double cols;
      struct center {
        cv::Point point;
        int index;
      };
    void floorButtonWithEmptyButtons(std::vector<DetectedObject> &objects, std::string desiredButtonName);
    void reclassifyUpDownButtons(std::vector<DetectedObject> &objects, std::string desiredButtonName, bool emptyButtonsDetected);
};
#endif