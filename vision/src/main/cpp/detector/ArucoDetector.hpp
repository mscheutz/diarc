/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef ARUCODETECTOR_HPP
#define ARUCODETECTOR_HPP

#include "ObjectDetector.hpp"

#if (CV_MAJOR_VERSION >= 4 && CV_MINOR_VERSION >= 7)
  #include <opencv2/objdetect/aruco_dictionary.hpp>
#else
  #include <opencv2/aruco.hpp>
#endif

class ArucoDetector : public ObjectDetector {
public:
  typedef boost::shared_ptr<ArucoDetector> Ptr;
  typedef boost::shared_ptr<const ArucoDetector> ConstPtr;

  ArucoDetector(const long long& processorId, const int imgWidth, const int imgHeight);
  ~ArucoDetector();

  virtual void loadConfig(const std::string& configFile);
  
protected:
  virtual void handleCaptureNotification(CaptureNotification::ConstPtr notification);

private:
#if (CV_MAJOR_VERSION >= 4 && CV_MINOR_VERSION >= 7)
  static std::unordered_map<std::string,cv::aruco::PredefinedDictionaryType> const dictionaryTable;
  cv::aruco::DetectorParameters parameters;
  cv::aruco::Dictionary dictionary;
  cv::aruco::ArucoDetector detector;
#else
  static std::unordered_map<std::string,cv::aruco::PREDEFINED_DICTIONARY_NAME> const dictionaryTable;
  cv::Ptr<cv::aruco::DetectorParameters> parameters;
  cv::Ptr<cv::aruco::Dictionary> dictionary;
#endif

  std::unordered_map<std::string, int> classToId;
  std::unordered_map<int, std::string> idToClass;
};

#endif  //ARUCODETECTOR_HPP
