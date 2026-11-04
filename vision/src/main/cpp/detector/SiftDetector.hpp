/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef SIFTDETECTOR_HPP
#define SIFTDETECTOR_HPP

#include "ObjectDetector.hpp"

class SiftDetector : public ObjectDetector {
public:
  typedef boost::shared_ptr<SiftDetector> Ptr;
  typedef boost::shared_ptr<const SiftDetector> ConstPtr;

  SiftDetector(const long long& processorId, const int imgWidth, const int imgHeight);
  ~SiftDetector();
  
  //! override because this detector never wants capture notifications (uses sift notifications)
  virtual void registerForCaptureNotification();
  
  //! override because this detector never wants capture notifications (uses sift notifications)
  virtual void unregisterForCaptureNotification();

  //!load sift features
  virtual void loadConfig(const std::string& config);

  virtual void cleanup();
  
protected:
  virtual void handleSiftNotification(SiftNotification::ConstPtr notification);
  
private:
  SiftFeatureVectPtr loadKeyFile(const std::string& filenamePath);
  void writeKeyFile(const std::string & filenamePath, SiftFeatureVectPtr) const;
  bool findSiftObject(const SiftFeatureVectPtr& targetSifts,
        SiftFeatures::ConstPtr& imgSifts,
        CaptureData::ConstPtr captureData, cv::Rect & boundingBox);

  bool saveSiftsToFile;

  //! Sift features that belong to target object(s).
  boost::unordered_map<std::string, std::string> sift_filenames;
  boost::unordered_map<std::string, SiftFeatureVectPtr> target_sifts;
  
  //! Number of matches to be considered detected object.
  int SIFT_MATCH_THRESH;
};

#endif  //SIFTDETECTOR_HPP