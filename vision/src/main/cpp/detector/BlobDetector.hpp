/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef BLOBDETECTOR_HPP
#define BLOBDETECTOR_HPP

#include "ObjectDetector.hpp"
#include "fastblobdetect/FastBlobDetector.hpp"

class BlobDetector : public ObjectDetector {
public:
  typedef boost::shared_ptr<BlobDetector> Ptr;
  typedef boost::shared_ptr<const BlobDetector> ConstPtr;

  BlobDetector(const long long& processorId, const int imgWidth, const int imgHeight);
  ~BlobDetector();

  virtual void loadConfig(const std::string& config);

  void setColors(const int size, const char* const* names, const int redLow[], const int redHigh[], const int greenLow[], const int greenHigh[], const int blueLow[], const int blueHigh[]);
  void setSizeMinsMaxs(const int size, const int mins[], const int maxs[]);
  void setBlur(const bool blurFlag, const int blurAmount);

  const int getNumColors() const;
  void getColors(const int size, int redLow[], int redHigh[], int greenLow[], int greenHigh[], int blueLow[], int blueHigh[]) const;
  std::string getColorLabel(const int index) const;
  void getSizeMinsMaxs(const int size, int mins[], int maxs[]) const;
  void getBlur(bool& blurFlag, int& blurAmount) const;

protected:
  virtual void handleCaptureNotification(CaptureNotification::ConstPtr notification);
  virtual void handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification);

  ade::stm::MemoryObject::VecPtr detectBlobs(const cv::Mat& currFrame, CaptureData::ConstPtr capture);

private:
  void performFastBlobDetectionImagePreProcessing(cv::Mat theFrame);
  FastBlobDetector::Ptr colorDetector;

  bool USE_FAST_BLOB_DETECTION_BLUR;
  int USE_FAST_BLOB_DETECTION_BLUR_AMOUNT;
};


#endif  //BLOBDETECTOR_HPP