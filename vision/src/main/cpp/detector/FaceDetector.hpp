/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef FACEDETECTOR_HPP
#define FACEDETECTOR_HPP

#include "ObjectDetector.hpp"

#include <unordered_map>

class FaceDetector : public ObjectDetector {
public:
  typedef boost::shared_ptr<FaceDetector> Ptr;
  typedef boost::shared_ptr<const FaceDetector> ConstPtr;

  FaceDetector(const long long& processorId, const int imgWidth, const int imgHeight);
  ~FaceDetector();

  //set which haar profile to load on initHaarCascades(). needs to be full path.
  virtual void loadConfig(const std::string& configFile);
  
protected:
  virtual void handleCaptureNotification(CaptureNotification::ConstPtr notification);

private:

  boost::unordered_map<std::string, std::string> cascade_names;
  boost::unordered_map<std::string, boost::shared_ptr<cv::CascadeClassifier> > cascades;

  cv::Mat frame_scaled;
};

#endif  //FACEDETECTOR_HPP
