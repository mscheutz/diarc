#include <alvision/alvisiondefinitions.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "FloorSupport.h"

FloorSupport::FloorSupport(std::string naoAddr)
: imageWidth(320),
imageHeight(240),
floorRect(imageWidth / 2 - 10, imageHeight - 21, 20, 20),
checkRect(imageWidth / 2 - 10, (imageHeight / 2) + 30, 20, 20),
hRangeVal(200),
sRangeVal(100),
vRangeVal(50),
displayFrame(imageHeight, imageWidth * 3, CV_8UC3),
displayThreadRunning(false),
displayThread(),
mutex() {
  std::string ip = "192.168.0.190"; //default
  int port = 9559; //default
  if (!naoAddr.empty()) {
    size_t pos = naoAddr.find_first_of(":", 0);
    if (pos != std::string::npos) {
      ip = naoAddr.substr(0, pos);
      port = boost::lexical_cast<int>(naoAddr.substr(pos + 1));
    } else {
      std::cerr << "Incorrect Nao IP address format. Should be x.x.x.x:port. Attempting to use default." << std::endl;
    }
  } else {
    std::cerr << "No Nao IP address was provided. Attempting to use default." << std::endl;
  }
  videoClientName = "FloorSupport";
  cameraProxy = boost::shared_ptr<AL::ALVideoDeviceProxy>(new AL::ALVideoDeviceProxy(ip, port));
  // Allocate our Image header.
  temp = cv::Mat(cv::Size(imageWidth, imageHeight), CV_8UC3);

  int resolution = AL::getResolutionFromSize(imageWidth, imageHeight);
  videoClientName = cameraProxy->subscribeCamera(videoClientName, 1, resolution, 13, 5);

  // Grab a few frames to get an image (I don't know why the first 2-3 frames are empty)
  for (int i = 0; i < 5; i++) {
    grabFrame();
  }
}

FloorSupport::~FloorSupport() {
  displayThreadRunning = false;
  cv::destroyAllWindows();
  if (cameraProxy) {
    cameraProxy->unsubscribe(videoClientName);
  }
};

void FloorSupport::displayFrames() {
  // create track bars
  std::string windowName = "Floor Support Check";
  cv::namedWindow(windowName);
  cv::createTrackbar("H-range", windowName, &hRangeVal, 359);
  cv::createTrackbar("S-range", windowName, &sRangeVal, 244);
  cv::createTrackbar("V-range", windowName, &vRangeVal, 244);

  cv::Rect roi;
  while (displayThreadRunning) {
    mutex.lock();

    roi = cv::Rect(0, 0, imageWidth, imageHeight);
    src.copyTo(displayFrame(roi));

    if (!colorFrame.empty()) {
      roi = cv::Rect(imageWidth, 0, imageWidth, imageHeight);
      colorFrame.copyTo(displayFrame(roi));
    }

    if (!edgeFrame.empty()) {
      roi = cv::Rect(imageWidth * 2, 0, imageWidth, imageHeight);
      edgeFrame.copyTo(displayFrame(roi));
    }

    mutex.unlock();

    cv::imshow(windowName, displayFrame);
    cv::waitKey(30);
  }

  // clean up on the way out
  cv::destroyAllWindows();
}

bool FloorSupport::grabFrame() {
  //  std::cerr << "Grabbing frame." << std::endl;
  AL::ALValue results = cameraProxy->getImageRemote(videoClientName);
  if (results.getType() != AL::ALValue::TypeArray && results.getSize() != 7) {
    std::cerr << "Invalid image returned." << std::endl;
    return false;
  }

  // Set the buffer we received to our cv::Mat header.
  temp.data = (uchar*) (results[6].GetBinary());
  src = temp.clone();

  cameraProxy->releaseImage(videoClientName);
  return true;
}

bool FloorSupport::detectFloorSupport() {
  std::lock_guard<std::mutex> lock(mutex);

  if (grabFrame()) {
    if (!displayThreadRunning) {
      displayThreadRunning = true;
      displayThread = std::thread(&FloorSupport::displayFrames, this);
    }

    bool colorSupport = checkColorFloorSupport();
    bool edgeSupport = checkEdgeFloorSupport();

    if (colorSupport && edgeSupport) { //maybe OR here??
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

bool FloorSupport::checkColorFloorSupport() {
  // Blur slightly
  cv::Mat frame;
  cv::blur(src, frame, cv::Size(3, 3));

  // Convert to hsv
  cv::Mat hsv;
  cv::cvtColor(frame, hsv, CV_BGR2HSV);

  // Sample floor color
  cv::Scalar floorColor = cv::mean(hsv(floorRect));

  // Extract areas that look like our floor color
  std::vector<cv::Mat> channels;
  cv::split(hsv, channels);
  cv::inRange(channels[0], floorColor.val[0] - hRangeVal, floorColor.val[0] + hRangeVal, channels[0]);
  cv::inRange(channels[1], floorColor.val[1] - sRangeVal, floorColor.val[1] + sRangeVal, channels[1]);
  cv::inRange(channels[2], floorColor.val[2] - vRangeVal, floorColor.val[2] + vRangeVal, channels[2]);

  cv::Mat result;
  cv::bitwise_and(channels[0], channels[1], result);
  cv::bitwise_and(channels[2], result, result);

  // Check for floor
  cv::Scalar checkColor = cv::mean(result(checkRect));

  // Debug: draw results
  cv::cvtColor(result, colorFrame, CV_GRAY2BGR);
  cv::rectangle(colorFrame, floorRect, cv::Scalar(0, 255, 0));
  cv::rectangle(colorFrame, checkRect, cv::Scalar(0, 255, 0));

  if (checkColor.val[0] > checkRect.area()*0.4) {
    return true;
  } else {
    return false;
  }
}

/**
 * Checks if there are edge pixels between the floorRect and checkRect boxes. 
 * Returns true if there less than some threshold number of edge pixels
 * (i.e., no edge detected), and returns false if more edge pixels above the
 * threshold (i.e., edge detected).
 * @return 
 */
bool FloorSupport::checkEdgeFloorSupport() {
  //blur
  cv::Mat frame;
  cv::blur(src, frame, cv::Size(3, 3));

  int lowThreshold = 100;
  int ratio = 3;
  int kernel_size = 3;

  cv::Mat edges;
  cv::Canny(frame, edges, lowThreshold, lowThreshold*ratio, kernel_size);

  // check for edge pixels in area connecting floorRect and checkRect
  int xStart = checkRect.x;
  int xEnd = checkRect.x + checkRect.width;
  int yStart = checkRect.y + checkRect.height / 2;
  int yEnd = floorRect.y;

  int edgeCount = 0; //edge pixels encountered in checked region
  for (int x = xStart; x < xEnd; ++x) {
    for (int y = yStart; y < yEnd; ++y) {
      if (edges.at<uchar>(y, x) > 0) {
        ++edgeCount;
      }
    }
  }

  // Using Canny's output as a mask, we display our result
  edgeFrame = cv::Scalar::all(0);
  frame.copyTo(edgeFrame, edges);

  if (edgeCount > 3) {
    return false;
  } else {
    return true;
  }
}
