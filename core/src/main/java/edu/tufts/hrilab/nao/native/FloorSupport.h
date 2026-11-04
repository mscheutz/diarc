#ifndef FLOORSUPPORT_H
#define FLOORSUPPORT_H
#include <alproxies/alvideodeviceproxy.h>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <opencv2/core/core.hpp>
#include <thread>
#include <iostream>
#include <mutex>

class FloorSupport {
public:
  FloorSupport(std::string naoAddr);
  ~FloorSupport();
  
  bool detectFloorSupport();
  
private:
  void displayFrames();
  bool grabFrame();
  bool checkColorFloorSupport();
  bool checkEdgeFloorSupport();
  
  std::string videoClientName;
  boost::shared_ptr<AL::ALVideoDeviceProxy> cameraProxy;
  
  int imageWidth, imageHeight;
  cv::Mat src, temp;
  cv::Mat colorFrame, edgeFrame, displayFrame;
  cv::Rect floorRect, checkRect;
  int hRangeVal;
  int sRangeVal;
  int vRangeVal;
  
  bool displayThreadRunning;
  std::thread displayThread;
  std::mutex mutex;
};

#endif //FLOORSUPPORT_H
