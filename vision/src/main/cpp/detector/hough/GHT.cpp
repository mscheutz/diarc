/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "GHT.hpp"
#include <cmath>
#include "display/Display.hpp"
#include "boost/format.hpp"

using namespace cv;

GenHoughTrnf::GenHoughTrnf() {
  // default values

  // canny threasholds
  thr1 = 50;
  thr2 = 150;

  // minimun and maximum width of the searched template
  wmin = 50;
  wmax = 200;
  // increasing step in pixels of the width
  rangeS = 5;
  // side of the squares in which the image is divided
  rangeXY = 6;

  // min value allowed is -M_PI
  phimin = -M_PI;
  // max value allowed is +M_PI
  phimax = +M_PI;
  // number of slices (angles) in R-table
  intervals = 16;

  logger = log4cxx::Logger::getLogger("diarc.detector.hough.GHT");
}

void GenHoughTrnf::setTresholds(int t1, int t2) {
  thr1 = t1;
  thr2 = t2;
}

void GenHoughTrnf::setLinearPars(int w1, int w2, int rS, int rXY) {
  wmin = w1;
  wmax = w2;
  rangeS = rS;
  rangeXY = rXY;
}

void GenHoughTrnf::setAngularPars(int p1, int p2, int ints) {
  if (p1 < p2) {
    if (p1>-M_PI) {
      phimin = p1;
    }
    if (p2<+M_PI) {
      phimax = p2;
    }
  }
  intervals = ints;
}

void GenHoughTrnf::createRtable(const std::string& imgFilename) {
  Mat original_img = imread(imgFilename);
  Mat template_img = createTemplate(original_img);

  // calc reference point of contour image and save it in variable refPoint
  int nl = template_img.rows;
  int nc = template_img.cols;
  float x_ref = 0, y_ref = 0;
  int ref_count = 0;
  for (int j = 0; j < nl; ++j) {
    uchar* data = (uchar*) (template_img.data + template_img.step.p[0] * j);
    for (int i = 0; i < nc; ++i) {
      if (data[i] == 255) {
        //calc updated mean
        ++ref_count;
        x_ref = x_ref + (i - x_ref) / ref_count;
        y_ref = y_ref + (j - y_ref) / ref_count;
      }
    }
  }
  refPoint = Vec2i(static_cast<int> (x_ref), static_cast<int> (y_ref));
  LOG4CXX_DEBUG(logger, boost::format("[createRtable] count: %d. ref point: (%d, %d).")
          % ref_count % refPoint(0) % refPoint(1));

  // get Scharr matrices from original template image to obtain contour gradients
  Mat input_img_gray;
  input_img_gray.create(Size(original_img.cols, original_img.rows), CV_8UC1);
  cvtColor(original_img, input_img_gray, cv::COLOR_BGR2GRAY);
  Mat dx;
  dx.create(Size(original_img.cols, original_img.rows), CV_16SC1);
  Sobel(input_img_gray, dx, CV_16S, 1, 0, -1); // -1 == CV_SCHARR == cv::FILTER_SCHARR:  for opencv3 and 4 compatability
  Mat dy;
  dy.create(Size(original_img.cols, original_img.rows), CV_16SC1);
  Sobel(input_img_gray, dy, CV_16S, 0, 1, -1);  // -1 == CV_SCHARR == cv::FILTER_SCHARR: for opencv3 and 4 compatability

  // create contour points and build Rtable from those points
  Rtable.clear();
  Rtable.resize(intervals);
  Rpoint rpt;
  int mindx = INT_MAX;
  int maxdx = INT_MIN;
  int angleindex = -1;
  float range = M_PI / intervals;
  for (int j = 0; j < nl; ++j) {
    uchar* data = (uchar*) (template_img.data + template_img.step.p[0] * j);
    for (int i = 0; i < nc; ++i) {
      if (data[i] == 255) {
        data[i] = 0; //recolor for debugging
        //      if (data[i] == Vec3b(255, 255, 255)) {
        //        data[i] = Vec3b(0, 0, 0); //recolor for debugging
        short vx = dx.at<short>(j, i);
        short vy = dy.at<short>(j, i);
        //float mag = std::sqrt( float(vx*vx+vy*vy) );
        rpt.dx = refPoint(0) - i;
        rpt.dy = refPoint(1) - j;
        float a = atan2((float) vy, (float) vx); //radians
        rpt.phi = ((a > 0) ? a - M_PI / 2 : a + M_PI / 2);
        //float a = atan2((float)vy, (float)vx) * 180/3.14159265358979f; //degrees
        //rpt.phi = ((a > 0) ? a-90 : a+90);
        // update further right and left dx
        if (rpt.dx < mindx) mindx = rpt.dx;
        if (rpt.dx > maxdx) maxdx = rpt.dx;

        angleindex = (int) ((rpt.phi + M_PI / 2.0) / range);
        if (angleindex == intervals) angleindex = intervals - 1;
        Rtable[angleindex].push_back(Vec2i(rpt.dx, rpt.dy));
      }
    }
  }
  // maximum width of the contour
  wtemplate = maxdx - mindx + 1;

  diarc::Display::createWindowIfDoesNotExist("rtable template");
  diarc::Display::displayFrame(template_img, "rtable template");
}

Mat GenHoughTrnf::createTemplate(const Mat& img) {
  Mat src_gray;
  Mat detected_edges;
  src_gray.create(Size(img.cols, img.rows), CV_8UC1);
  cvtColor(img, src_gray, cv::COLOR_BGR2GRAY);
  blur(src_gray, detected_edges, Size(3, 3));
  Canny(detected_edges, detected_edges, 1, 100, 3);

  diarc::Display::createWindowIfDoesNotExist("template");
  diarc::Display::displayFrame(detected_edges, "template");
  return detected_edges;
}

std::vector<cv::Rect> GenHoughTrnf::detect(const cv::Mat& input_img) {

  accumulate(input_img);

  Rect bb;
  std::vector<cv::Rect> detectionResults;
  int counter = 0, maxCount = 10;
  while (counter < maxCount) {
    if (bestCandidate(bb)) {
      detectionResults.push_back(bb);
    }
    ++counter;
  }

  return detectionResults;
}

void GenHoughTrnf::accumulate(const cv::Mat& input_img) {
  showimage = input_img.clone();
  // transform image to grayscale:
  Mat src_gray;
  src_gray.create(Size(input_img.cols, input_img.rows), CV_8UC1);
  cvtColor(input_img, src_gray, cv::COLOR_BGR2GRAY);
  // reduce noise with a kernel 3x3 and get cannyedge image:
  Mat detected_edges;
  blur(src_gray, detected_edges, Size(3, 3));
  Canny(detected_edges, detected_edges, thr1, thr2, 3);
  diarc::Display::createWindowIfDoesNotExist("edges");
  diarc::Display::displayFrame(detected_edges, "edges");

  // get Scharr matrices from image to obtain contour gradients
  Mat dx;
  dx.create(Size(input_img.cols, input_img.rows), CV_16SC1);
  Sobel(src_gray, dx, CV_16S, 1, 0, -1); // -1 == CV_SCHARR == cv::FILTER_SCHARR:  for opencv3 and 4 compatability
  Mat dy;
  dy.create(Size(input_img.cols, input_img.rows), CV_16SC1);
  Sobel(src_gray, dy, CV_16S, 0, 1, -1); // -1 == CV_SCHARR == cv::FILTER_SCHARR:  for opencv3 and 4 compatability

  // load all points from image all image contours on vector pts2
  int nl = detected_edges.rows;
  int nc = detected_edges.cols;
  float deltaphi = M_PI / intervals;
  float inv_deltaphi = (float) intervals / M_PI;
  float inv_rangeXY = (float) 1 / rangeXY;
  float pi_half = M_PI * 0.5f;
  std::vector<Rpoint2> pts2;
  for (int j = 0; j < nl; ++j) {
    uchar* data = (uchar*) (detected_edges.data + detected_edges.step.p[0] * j);
    for (int i = 0; i < nc; ++i) { // consider only white points (contour)
      if (data[i] == 255) {
        short vx = dx.at<short>(j, i);
        short vy = dy.at<short>(j, i);
        Rpoint2 rpt;
        rpt.x = i*inv_rangeXY;
        rpt.y = j*inv_rangeXY;
        float a = atan2((float) vy, (float) vx); //	gradient angle in radians
        float phi = ((a > 0) ? a - pi_half : a + pi_half); // contour angle with respect to x axis
        int angleindex = (int) ((phi + M_PI * 0.5f) * inv_deltaphi); // index associated with angle (0 index = -90 degrees)
        if (angleindex == intervals) angleindex = intervals - 1; // -90 angle and +90 has same effect
        rpt.phiindex = angleindex;
        pts2.push_back(rpt);
      }
    }
  }

  // OpenCv 4-dimensional matrix definition and in general a useful way for defining multidimensional arrays and vectors in c++
  // create accumulator matrix
  int X = ceil((float) nc / rangeXY);
  int Y = ceil((float) nl / rangeXY);
  int S = ceil((float) (wmax - wmin) / rangeS + 1.0f);
  int R = ceil(phimax / deltaphi) - floor(phimin / deltaphi);
  if (phimax == M_PI && phimin == -M_PI) R--;
  int r0 = -floor(phimin / deltaphi);
  int matSizep_S[] = {X, Y, S, R};
  accum.create(4, matSizep_S, CV_16S);
  accum = Scalar::all(0);
  // icrease accum cells with hits corresponding with slope in Rtable vector rotatated and scaled
  float inv_wtemplate_rangeXY = (float) 1 / (wtemplate * rangeXY);
  // rotate RTable from minimum to maximum angle
  for (int r = 0; r < R; ++r) { // rotation
    int reff = r - r0;
    std::vector < std::vector < Vec2f > > Rtablerotated(intervals);
    // cos and sin are computed in the outer loop to reach computational efficiency
    float cs = cos(reff * deltaphi);
    float sn = sin(reff * deltaphi);
    for (std::vector < std::vector < Vec2i > > ::size_type ii = 0; ii < Rtable.size(); ++ii) {
      for (std::vector<Vec2i>::size_type jj = 0; jj < Rtable[ii].size(); ++jj) {
        int iimod = (ii + reff) % intervals;
        Rtablerotated[iimod].push_back(Vec2f(cs * Rtable[ii][jj][0] - sn * Rtable[ii][jj][1], sn * Rtable[ii][jj][0] + cs * Rtable[ii][jj][1]));
      }
    }
    // scale the rotated RTable from minimum to maximum scale
    for (int s = 0; s < S; ++s) { // scale
      std::vector < std::vector < Vec2f > > Rtablescaled(intervals);
      int w = wmin + s*rangeS;
      float wratio = (float) w*inv_wtemplate_rangeXY;
      for (std::vector < std::vector < Vec2f > > ::size_type ii = 0; ii < Rtablerotated.size(); ++ii) {
        for (std::vector<Vec2f>::size_type jj = 0; jj < Rtablerotated[ii].size(); ++jj) {
          Rtablescaled[ii].push_back(Vec2f(wratio * Rtablerotated[ii][jj][0], wratio * Rtablerotated[ii][jj][1]));
        }
      }
      // iterate through each point of edges and hit corresponding cells from rotated and scaled Rtable
      for (std::vector<Rpoint2>::size_type t = 0; t < pts2.size(); ++t) { // XY plane
        int angleindex = pts2[t].phiindex;
        for (std::vector<Vec2f>::size_type index = 0; index < Rtablescaled[angleindex].size(); ++index) {
          float deltax = Rtablescaled[angleindex][index][0];
          float deltay = Rtablescaled[angleindex][index][1];
          int xcell = (int) (pts2[t].x + deltax);
          int ycell = (int) (pts2[t].y + deltay);
          if ((xcell < X)&&(ycell < Y)&&(xcell>-1)&&(ycell>-1)) {
            //(*( (short*)(accum.data + xcell*accum.step.p[0] + ycell*accum.step.p[1] + s*accum.step.p[2]+ r*accum.step.p[3])))++;
            (*ptrat4D(accum, xcell, ycell, s, r))++;
          }
        }
      }
    }
  }
}

bool GenHoughTrnf::bestCandidate(Rect& boundingBox) {
  double minval;
  double maxval;
  int id_min[4] = {0, 0, 0, 0};
  int id_max[4] = {0, 0, 0, 0};
  minMaxIdx(accum, &minval, &maxval, id_min, id_max);

  short* maxValPtr = ptrat4D(accum, id_max[0], id_max[1], id_max[2], id_max[3]);
  LOG4CXX_DEBUG(logger, boost::format("[bestCandidate] count: %d. idx: (%d,%d,%d,%d)")
          % (*maxValPtr) % id_max[0] % id_max[1] % id_max[2] % id_max[3]);
  *maxValPtr = 0;

  //suppress current best region
  for (int s = 0; s < accum.size[2]; ++s) {
    for (int r = 0; r < accum.size[3]; ++r) {
      *(ptrat4D(accum, id_max[0], id_max[1], s, r)) = 0;
    }
  }

  int nl = showimage.rows;
  int nc = showimage.cols;

  Vec2i referenceP = Vec2i(id_max[0] * rangeXY + (rangeXY + 1) / 2, id_max[1] * rangeXY + (rangeXY + 1) / 2);

  // rotate and scale points all at once. Then impress them on image
  int xmin_rect = INT_MAX, xmax_rect = INT_MIN, ymin_rect = INT_MAX, ymax_rect = INT_MIN;
  std::vector < std::vector < Vec2i > > Rtablerotatedscaled(intervals);
  float deltaphi = M_PI / intervals;
  int r0 = -floor(phimin / deltaphi);
  int reff = id_max[3] - r0;
  float cs = cos(reff * deltaphi);
  float sn = sin(reff * deltaphi);
  int w = wmin + id_max[2] * rangeS;
  float wratio = (float) w / (wtemplate);
  for (std::vector < std::vector < Vec2i > > ::size_type ii = 0; ii < Rtable.size(); ++ii) {
    for (std::vector<Vec2i>::size_type jj = 0; jj < Rtable[ii].size(); ++jj) {
      int iimod = (ii + reff) % intervals;
      int dx = roundToInt(wratio * (cs * Rtable[ii][jj][0] - sn * Rtable[ii][jj][1]));
      int dy = roundToInt(wratio * (sn * Rtable[ii][jj][0] + cs * Rtable[ii][jj][1]));
      int x = referenceP[0] - dx;
      int y = referenceP[1] - dy;
      //Rtablerotatedscaled[ii].push_back(Vec2i( dx, dy));
      if ((x < nc)&&(y < nl)&&(x>-1)&&(y>-1)) {
        showimage.at<Vec3b>(y, x) = Vec3b(0, 255, 255);
        if (x < xmin_rect) xmin_rect = x;
        if (x > xmax_rect) xmax_rect = x;
        if (y < ymin_rect) ymin_rect = y;
        if (y > ymax_rect) ymax_rect = y;
      }
    }
  }

  boundingBox = Rect(xmin_rect, ymin_rect, xmax_rect - xmin_rect, ymax_rect - ymin_rect);

  // show result
  diarc::Display::createWindowIfDoesNotExist("hough");
  diarc::Display::displayFrame(showimage, "hough");

  return true;
}

void GenHoughTrnf::localMaxima() {
  // to bve implemented ...
}

inline int GenHoughTrnf::roundToInt(float num) {
  return (num > 0.0) ? (int) (num + 0.5f) : (int) (num - 0.5f);
}

inline short GenHoughTrnf::at4D(Mat &mt, int i0, int i1, int i2, int i3) {
  //(short)(mt.data + i0*mt.step.p[0] + i1*mt.step.p[1] + i2*mt.step.p[2] + i3*mt.step.p[3]);	
  return *((short*) (mt.data + i0 * mt.step.p[0] + i1 * mt.step.p[1] + i2 * mt.step.p[2] + i3 * mt.step.p[3]));
}

inline short* GenHoughTrnf::ptrat4D(Mat &mt, int i0, int i1, int i2, int i3) {
  return (short*) (mt.data + i0 * mt.step.p[0] + i1 * mt.step.p[1] + i2 * mt.step.p[2] + i3 * mt.step.p[3]);
}

//void runGHT(char c) {
//  //c = 't'; // create template
//  //c = 'r'; // run algorithm
//  if (c == 't') {
//    GenHoughTrnf ght;
//    ght.createTemplate();
//  } else if (c == 'r') {
//    GenHoughTrnf ght;
//    //ght.setTresholds(180, 250);
//    ght.createRtable();
//    //Mat detect_img = imread("files\\Img_01.png", 1);
//    Mat detect_img = imread("files\\Img_02.png", 1);
//    //Mat detect_img = imread("files\\Img_03.png", 1);
//    ght.accumulate(detect_img);
//    ght.bestCandidate();
//  }
//}