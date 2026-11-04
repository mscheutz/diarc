/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   DisplayUtil.hpp
 * Author: Evan Krause <evan.krause@tufts.edu>
 *
 * Created on March 21, 2016, 12:29 PM
 */

#ifndef DISPLAYUTILITIES_HPP
#define	DISPLAYUTILITIES_HPP

#include <opencv2/opencv.hpp>
#include <log4cxx/logger.h>

namespace diarc {
  namespace display {

    static log4cxx::LoggerPtr logger(log4cxx::Logger::getLogger("diarc.display.DisplayUtilities"));

    /**
     * @brief makeCanvas Makes composite image from the given images
     * @param vecMat Vector of Images. Must be non-empty and of same type and size.
     * @return new composite image.
     */
    cv::Mat makeCanvas(std::vector<cv::Mat>& vecMat) {
      int n = vecMat.size();
      int imageHeight = vecMat[0].rows;
      int imageWidth = vecMat[0].cols;
      int type = vecMat[0].type();
      int nRows = ceil(sqrt(n));
      int imagesPerRow = ceil(double(n) / nRows);
      int windowHeight = nRows * vecMat[0].rows;
      int windowWidth = imagesPerRow * vecMat[0].cols;

      cv::Mat canvasImage(windowHeight, windowWidth, CV_8UC3, cv::Scalar(0, 0, 0));

      int k = 0;
      for (int i = 0; i < nRows; ++i) {
        int y = i * imageHeight;
        int x = 0;
        for (int j = 0; j < imagesPerRow && k < n; ++k, ++j) {
          cv::Rect roi(x, y, imageWidth, imageHeight);
          cv::Size s = canvasImage(roi).size();

          // copy and convert size (if necessary)
          cv::Mat target_ROI;
          cv::resize(vecMat[k], target_ROI, s);

          // convert color/channels (if necessary)
          if (target_ROI.channels() != canvasImage.channels() && target_ROI.channels() == 1) {
            cv::cvtColor(target_ROI, target_ROI, cv::COLOR_GRAY2BGR);
          }

          // convert type (if necessary)
          if (target_ROI.type() != CV_8UC3) {
            double alpha = 1.0;
            if (target_ROI.type() == CV_32FC3) {
              alpha = 255.0;
            } else if (target_ROI.type() == CV_16UC3 || target_ROI.type() == CV_32SC3) {
              alpha = 1 / 256.0;
            }
            target_ROI.convertTo(target_ROI, canvasImage.type(), alpha);
          }

          // copy to canvas image
          target_ROI.copyTo(canvasImage(roi));
          x += imageWidth;
        }
      }
      return canvasImage;
    }


  } //namespace diarc
} //namespace display

#endif	/* DISPLAYUTILITIES_HPP */

