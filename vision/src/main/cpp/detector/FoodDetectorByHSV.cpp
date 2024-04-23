/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @cst <Chris Thierauf, chris@cthierauf.com>
 */

#include "FoodDetectorByHSV.hpp"
#include "display/Display.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Some debug flags (defined means the debug will occur) (this will break inside of vision)
#define DEBUGMODE_bgshow
#define DEBUGMODE_showboundingrect
#define DEBUGMODE_showboundingrect_good
#define DEBUGMODE_showmasks
#define DEBUGMODE_showlabled

using std::string;
using std::vector;
using cv::Point;
using cv::Mat;
using namespace ade::stm;

// If the area of a bounding rectangle is smaller than this percentage of the frame dimensions,
// it's too small to be anything we care about and will be ignored
// #define BOUNDING_RECT_MIN_PERC 0.075
#define BOUNDING_RECT_MIN_PERC 0.004

// If the bounding rectangle is greater than this percentage of the frame's dimensions, it's too big
// to be useful and will be ignored (it's probably the entire image frame)
#define BOUNDING_RECT_MAX_PERC  0.80

#define OPENCV_HSV_SIZE 180
#define IMAGE_AREA (SELFIMG.frame.cols * SELFIMG.frame.rows)

// HSV's of foods is a simple hashmap
string FOODMAP[OPENCV_HSV_SIZE];

struct __SELFIMGS {
  Mat frame;
  Mat mask;
};
struct __SELFIMGS SELFIMG;

FoodDetectorByHSV::FoodDetectorByHSV(const long long &processorId, const int imgWidth, const int imgHeight)
        : ObjectDetector(processorId, imgWidth, imgHeight) {

  visionProcessName = "FoodDetectorByHSV";
  logger = log4cxx::Logger::getLogger("ade.detector.FoodDetectorByHSV");

  // Some foods to get started
  FoodType foodlist[] = {
          {"carrot",   7,  1},
          {"cucumber", 70, 1},
          {"pear",     26, 1},
          {"tomato",   3,  1},
  };

  for (FoodType f : foodlist) {
    insert_into_foodmap(f.name, f.hsv);
  }
}

FoodDetectorByHSV::~FoodDetectorByHSV() {

}

void FoodDetectorByHSV::insert_into_foodmap(string food, int H) {
  if (H < 0 || H > OPENCV_HSV_SIZE) {
    LOG4CXX_ERROR(logger, "[insert_into_foodmap] HSV value must be integer from 0 to 180.");
    return;
  }

  FOODMAP[H] = food;
  LOG4CXX_DEBUG(logger, boost::format("[insert_into_foodmap] %d = %s.") % H % FOODMAP[H].c_str());
}

Mat FoodDetectorByHSV::mask_out_background() {
  // Original, not sure where from
  // cv::Scalar low = cv::Scalar(103, 83, 17);
  // cv::Scalar high = cv::Scalar(141, 118, 118);

  // Eric Tablecover day + lights
  // cv::Scalar low = cv::Scalar(64, 43, 12);
  // cv::Scalar high = cv::Scalar(90, 67, 35);
  cv::Scalar low = cv::Scalar(33, 20, 0);
  cv::Scalar high = cv::Scalar(84, 59, 33);

  SELFIMG.mask = SELFIMG.frame;
  cv::inRange(SELFIMG.mask, low, high, SELFIMG.mask);

  return SELFIMG.mask;
}

vector <FoodDetectorByHSV::Food> FoodDetectorByHSV::get_foods_in_view() {
  mask_out_background();
  #ifdef DEBUGMODE_bgshow
  ade::Display::createWindowIfDoesNotExist("bg");
  ade::Display::displayFrame(SELFIMG.mask, "bg");
  #endif
  vector <Food> foods = divide_image();
  for (uint n = 0; n < foods.size(); ++n) {
    foods[n].type = get_masked_food(foods[n]);
  }

  return foods;
}

vector <FoodDetectorByHSV::Food> FoodDetectorByHSV::detect_foods(cv::Mat currFrame) {
  currFrame.copyTo(SELFIMG.frame);
  vector <FoodDetectorByHSV::Food> foods = get_foods_in_view();
  return foods;
}

void FoodDetectorByHSV::loadConfig(const std::string &config) {
  std::cout << config << std::endl;
}

int food_countup(int hsv) {
  int n;
  for (n = 0; FOODMAP[hsv + n].length() < 1; ++n)
    if (hsv + n == OPENCV_HSV_SIZE)
      hsv = 0;

  return n;
}

int food_countdown(int hsv) {
  int n;
  for (n = 0; FOODMAP[hsv + n].length() < 1; --n)
    if (hsv + n == 0)
      hsv = OPENCV_HSV_SIZE;

  return n;
}

FoodDetectorByHSV::FoodType FoodDetectorByHSV::get_nearest_food(int hsv) {
  int up = food_countup(hsv);
  int down = food_countdown(hsv);

  int closer = (abs(up) > abs(down)) ? hsv + down : hsv + up;
  if (closer > OPENCV_HSV_SIZE) closer -= OPENCV_HSV_SIZE;
  if (closer < 0) closer += OPENCV_HSV_SIZE;

  FoodType f;
  f.name = FOODMAP[closer];
  f.hsv = hsv;
  f.confidence = (1 - ((abs(closer - hsv) * 1.0) / OPENCV_HSV_SIZE)) * 100;

  return f;
}

int FoodDetectorByHSV::max_within_mask(Mat mask, Mat image) {
  int quickhist[OPENCV_HSV_SIZE] = {0};
  // LOG4CXX_ERROR(logger, boost::format("image row,cols: %d,%d.") % image.rows % image.cols);
  for (int y = 0; y < image.rows; ++y) {
    for (int x = 0; x < image.cols; ++x) {
      if (mask.at<uchar>(y, x) > 0) {     // This will accept int32, char8, etc, but they all give weird results
      // if (mask.at<int>(y, x) > 0) {     // This will accept int32, char8, etc, but they all give weird results
        quickhist[int(image.at<cv::Vec3b>(y, x)[0])]++;
      }
    }
  }

  int indexof_max = 0;
  for (int n = 0; n < OPENCV_HSV_SIZE; ++n) {
    if (quickhist[n] > quickhist[indexof_max]) indexof_max = n;
  }
  return indexof_max;
}

FoodDetectorByHSV::FoodType FoodDetectorByHSV::get_masked_food(Food f) {
  Mat hsv_mat;
  cv::cvtColor(SELFIMG.frame, hsv_mat, cv::COLOR_BGR2HSV);

  int hsv_int = max_within_mask(f.maskarea, hsv_mat);
  FoodType returnme = get_nearest_food(hsv_int);
  #ifdef DEBUGMODE_showlabled
  //std::string printme = returnme.name;
  //printme.append(" (H:").append(std::to_string(returnme.hsv)).append(", C:").append(std::to_string(returnme.confidence)).append(")");
  //cv::putText(SELFIMG.frame, printme, f.centroid, CV_FONT_HERSHEY_PLAIN, 1.0, CV_RGB(200, 200, 200), 2.0);        cv::circle(SELFIMG.frame, f.centroid, 20, CV_RGB(0, 255, 0), -1);

  ade::Display::createWindowIfDoesNotExist("showlabeled");
  ade::Display::displayFrame(SELFIMG.frame, "showlabeled");
  #endif
  return returnme;
}

vector <FoodDetectorByHSV::Food> FoodDetectorByHSV::divide_image() {
  // Get a mask and a copy ready to manipulate
  // Gaussian blur scales relative to frame size, values computed off of experimental data
  long smaller_dimension = (SELFIMG.frame.cols < SELFIMG.frame.rows)? SELFIMG.frame.cols : SELFIMG.frame.rows;
  int digits = floor(log10(smaller_dimension))+1;
  int blurval = pow(10, digits-2);
  Mat kernel = cv::getGaussianKernel(blurval, 1/(blurval*blurval));
  Mat copy = Mat::zeros(SELFIMG.mask.size(), SELFIMG.mask.type());
  SELFIMG.mask.copyTo(copy);
  cv::filter2D(copy, copy, -1, kernel);

  vector <vector<Point> > contours;
  vector <vector<Point> > good_contours;
  cv::findContours(copy, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  #ifdef DEBUGMODE_showboundingrect
  Mat bounds;
  SELFIMG.frame.copyTo(bounds);
  #endif

  LOG4CXX_DEBUG(logger, boost::format("[divide_image] number of contours: %lu.") % contours.size());
  int i = 0;
  for (vector <Point> p : contours) {
    cv::Rect rect;
    rect = cv::boundingRect(p);
    #ifdef DEBUGMODE_showboundingrect
    cv::drawContours(copy, contours, i++, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
    cv::rectangle(bounds, rect.tl(), rect.br(), cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
    #endif

    int rectarea = (rect.tl().x - rect.br().x) * (rect.tl().y - rect.br().y);
    if (rectarea > (IMAGE_AREA * BOUNDING_RECT_MIN_PERC) &&
        rectarea < (IMAGE_AREA * BOUNDING_RECT_MAX_PERC)) {
      good_contours.push_back(p);
    }
  }
  #ifdef DEBUGMODE_showboundingrect
  ade::Display::createWindowIfDoesNotExist("contours");
  ade::Display::displayFrame(copy, "contours");
  ade::Display::createWindowIfDoesNotExist("bounds");
  ade::Display::displayFrame(bounds, "bounds");
  #endif


  #ifdef DEBUGMODE_showboundingrect_good
  Mat gbounds;
  SELFIMG.frame.copyTo(gbounds);

  for(vector<Point> p : good_contours) {
    cv::Rect rect = cv::boundingRect(p);
    cv::rectangle(gbounds, rect.tl(), rect.br(), cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
  }
  ade::Display::createWindowIfDoesNotExist("gbounds");
  ade::Display::displayFrame(gbounds, "gbounds");
  #endif


  vector <Food> returnfood;
  Mat all_masks = Mat::zeros(SELFIMG.mask.size(), SELFIMG.mask.type());
  for (uint n = 0; n < good_contours.size(); ++n) {
    cv::Rect rect;
    rect = cv::boundingRect(good_contours[n]);
    Food f;
    Mat foodmask = Mat::zeros(SELFIMG.mask.size(), SELFIMG.mask.type());
    // for (int y = 0; y < foodmask.rows; ++y) {
    //   for (int x = 0; x < foodmask.cols; ++x) {
    //     if (x > rect.br().x && x < rect.tl().x && y > rect.br().y && y < rect.tl().y) {
    //       foodmask.at<int>(y, x) = 1;
    //       // LOG4CXX_DEBUG(logger, boost::format("mask at %d,%d = %d.") % y % x % int(foodmask.at<uchar>(y, x)));
    //     }
        
    //     // if (foodmask.at<uchar>(y, x) > 0) {     // This will accept int32, char8, etc, but they all give weird results
    //       // LOG4CXX_DEBUG(logger, boost::format("%d,%d.") % y % x);    
    //     // }
    //   }
    // }
    // cv::rectangle(foodmask, rect.tl(), rect.br(), cv::Scalar(255, 0, 0), cv::FILLED, CV_AA);
    // for (int y = 0; y < foodmask.rows; ++y) {
    //   for (int x = 0; x < foodmask.cols; ++x) {
    //     // LOG4CXX_ERROR(logger, boost::format("mask at %d,%d = %d.") % y % x % int(mask.at<uchar>(y, x)));
    //     if (foodmask.at<uchar>(y, x) > 0) {     // This will accept int32, char8, etc, but they all give weird results
    //       LOG4CXX_ERROR(logger, boost::format("%d,%d.") % y % x);    
    //     }
    //   }
    // }
    // ade::Display::createWindowIfDoesNotExist("foodmask");
    // ade::Display::displayFrame(foodmask, "foodmask");
    cv::drawContours(foodmask, good_contours, n, cv::Scalar(255, 255, 255), -1);
    cv::drawContours(all_masks, good_contours, n, cv::Scalar(255, 255, 255), -1);
    f.maskarea = foodmask;
    f.centroid = Point(rect.br().x - (rect.width / 2), rect.br().y - (rect.height / 2));
    returnfood.push_back(f);
  }

  #ifdef DEBUGMODE_showmasks
    ade::Display::createWindowIfDoesNotExist("showmasks");
    ade::Display::displayFrame(all_masks, "showmasks");
  #endif

  SELFIMG.mask = copy;
  return returnfood;
}

void FoodDetectorByHSV::handleCaptureNotification(CaptureNotification::ConstPtr notification) {

  vector<FoodDetectorByHSV::Food> foods;
  foods = detect_foods(notification->captureData->frame);
  LOG4CXX_DEBUG(logger, boost::format("[handleCaptureNotification] food items detected: %lu.") % foods.size());

  TypesByDescriptorConstPtr descriptors = getDescriptors();
  TypesByDescriptor::const_iterator descriptor_iter;
  std::tr1::unordered_set<long long>::const_iterator typeIds_itr;
  MemoryObject::VecPtr foodMemoryObjects(new MemoryObject::Vec());

  vector<FoodDetectorByHSV::Food>::iterator foods_itr;
  for (foods_itr = foods.begin(); foods_itr != foods.end(); ++foods_itr) {

    LOG4CXX_DEBUG(logger, boost::format("[handleCaptureNotification] food item type: %s.") % foods_itr->type.name.c_str());
    for (descriptor_iter = descriptors->begin(); descriptor_iter != descriptors->end(); ++descriptor_iter) {
      if (descriptor_iter->first.getName().compare(foods_itr->type.name) == 0 ||
          descriptor_iter->first.getName().compare("anyfood") == 0) {

        LOG4CXX_DEBUG(logger, boost::format("[handleCaptureNotification] creating MO: %s.") %
                              descriptor_iter->first.toString().c_str());
        //cv::Mat_<float> binaryMask;
        //cv::threshold(foods_itr->maskarea, binaryMask, 0.0f, 1.0f, CV_THRESH_BINARY);
        //ade::Display::createWindowIfDoesNotExist("binaryMask");
        //ade::Display::displayFrame(binaryMask, "binaryMask");

        // create MemoryObject for each relevant typeId
        for (typeIds_itr = descriptor_iter->second.begin();
             typeIds_itr != descriptor_iter->second.end(); ++typeIds_itr) {

          MemoryObjectMask::Ptr mask(new MemoryObjectMask(notification->captureData, foods_itr->maskarea));
          MemoryObject::Ptr foodMO(
                  new MemoryObject(*typeIds_itr, descriptor_iter->first.getArg(0), notification->captureData, mask));
          foodMO->addValidationResult(0.9, descriptor_iter->first);
          if (descriptor_iter->first.getName().compare("anyfood") == 0)
            foodMO->addValidationResult(0.9, PredicateHelper(foods_itr->type.name + "(" + descriptor_iter->first.getArg(0) + ")"));
          foodMemoryObjects->push_back(foodMO);
        }
      }
    }
  }

  ObjectDetector::sendDetectionNotifications(foodMemoryObjects);

  // draw food bounding boxes
  if (getDisplayFlag()) {
    notification->captureData->frame.copyTo(displayFrame);

    MemoryObject::Vec::iterator food_iter;
    for (food_iter = foodMemoryObjects->begin(); food_iter != foodMemoryObjects->end(); ++food_iter) {
      cv::rectangle(displayFrame, (*food_iter)->getDetectionMask()->getBoundingBox(), cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
    }
    ade::Display::displayFrame(displayFrame, getDisplayName());
  }
}
