/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

//
// Created by evan on 6/24/20.
//

#ifndef ADE_VISION_DETECTOR_COLORBASED_COLORRANGE_HPP
#define ADE_VISION_DETECTOR_COLORBASED_COLORRANGE_HPP

class ColorRange {
 public:
  cv::ColorConversionCodes colorSpace;
  cv::Scalar start;
  cv::Scalar end;
 private:

};

#endif //ADE_VISION_DETECTOR_COLORBASED_COLORRANGE_HPP
