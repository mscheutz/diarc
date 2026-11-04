/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   GHT.hpp
 * Author: evan
 *
 * Created on March 31, 2014, 3:39 PM
 */

#include <opencv2/opencv.hpp>
#include <log4cxx/logger.h>

#ifndef GHT_HPP
#define	GHT_HPP

struct Rpoint {
  int dx;
  int dy;
  float phi;
};

struct Rpoint2 {
  float x;
  float y;
  int phiindex;
};

class GenHoughTrnf {
public:

  GenHoughTrnf();

  /**
   * Set Canny edge detection thresholds.
   * @param t1
   * @param t2
   */
  void setTresholds(int t1, int t2);

  void setLinearPars(int w1, int w2, int rS, int rXY);

  void setAngularPars(int p1, int p2, int ints);

  void createRtable(const std::string& imgFilename);

  std::vector<cv::Rect> detect(const cv::Mat& input_img);

private:
  // create binary contour template from original image
  cv::Mat createTemplate(const cv::Mat& img);

  // fill accumulator matrix
  void accumulate(const cv::Mat& input_img);

  // show the best candidate detected on image
  bool bestCandidate(cv::Rect& boundingBox);

  // finds local minima above a certain threshold
  void localMaxima();

  inline int roundToInt(float num);
  inline short at4D(cv::Mat &mt, int i0, int i1, int i2, int i3);
  inline short* ptrat4D(cv::Mat &mt, int i0, int i1, int i2, int i3);

  // accumulator matrix
  cv::Mat accum;
  // accumulator matrix
  cv::Mat showimage;
  // reference point (inside contour)
  cv::Vec2i refPoint;
  // R-table of template object:
  std::vector<std::vector<cv::Vec2i> > Rtable;
  // number of intervals for angles of R-table:
  int intervals;
  // threasholds of canny edge detector
  int thr1;
  int thr2;
  // width of template contour
  int wtemplate;
  // minimum and maximum width of scaled contour
  int wmin;
  int wmax;
  // minimum and maximum rotation allowed for template
  float phimin;
  float phimax;
  // dimension in pixels of squares in image
  int rangeXY;
  // interval to increase scale
  int rangeS;
  
  log4cxx::LoggerPtr logger;
};


#endif	/* GHT_HPP */

