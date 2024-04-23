/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef __HSVFOODDETECT
#define __HSVFOODDETECT

#include <string>
#include <opencv2/opencv.hpp>
#include "ObjectDetector.hpp"

class FoodDetectorByHSV : public ObjectDetector {
public:
  typedef boost::shared_ptr <FoodDetectorByHSV> Ptr;
  typedef boost::shared_ptr<const FoodDetectorByHSV> ConstPtr;

  FoodDetectorByHSV(const long long &processorId, const int imgWidth, const int imgHeight);

  ~FoodDetectorByHSV();

  void insert_into_foodmap(std::string food, int H);

  typedef struct FOODTYPE {
    std::string name;
    int hsv;
    int confidence;
  } FoodType;

  typedef struct FOOD {
    FoodType type;
    cv::Point grasp1;
    cv::Point grasp2;
    cv::Point centroid;
    float orientation;
    cv::Mat_<float> maskarea;
  } Food;

  void loadConfig(const std::string &config);

  void handleCaptureNotification(CaptureNotification::ConstPtr notification);

  std::vector <FoodDetectorByHSV::Food> detect_foods(cv::Mat currFrame);

private:
  std::vector <Food> get_foods_in_view();

  FoodType get_nearest_food(int hsv);

  FoodType get_masked_food(Food f);

  int max_within_mask(cv::Mat mask, cv::Mat image);

  std::vector <Food> divide_image();

  cv::Mat mask_out_background();

};

#endif // __HSVFOODDETECT
