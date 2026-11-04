/**
 * Test Yefeng Zheng's (University of Maryland, College Park) shape context.
 * @author Michael Zillich
 * @date December 2013
 */

#include <stdio.h>
#include <list>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <third_party/point_match/PointMatch.h>
#include <third_party/point_match/Tools.h>
#include <limits>

class ShapeContext {
public:
  int nbins_r; ///< Number of bins in distance
  int nbins_theta; ///< Number of bins in angle
  cv::Mat hist; ///< histogram vector: 1 x nbins_r*nbins_theta
  cv::Mat image;

  ShapeContext(int nr = 5, int ntheta = 4) {
    nbins_r = nr;
    nbins_theta = ntheta;
    hist.create(1, nbins_r*nbins_theta, CV_32F);
    hist.setTo(0.);
  }

  ShapeContext(const ShapeContext &sc) {
    nbins_r = sc.nbins_r;
    nbins_theta = sc.nbins_theta;
    sc.hist.copyTo(hist);
  }

  ShapeContext& operator=(const ShapeContext &sc) {
    nbins_r = sc.nbins_r;
    nbins_theta = sc.nbins_theta;
    sc.hist.copyTo(hist);
    return *this;
  }

  void resize(int nr, int ntheta) {
    nbins_r = nr;
    nbins_theta = ntheta;
    hist.resize(1, nbins_r * nbins_theta);
  }

  float distance(ShapeContext &sc) {
    if (nbins_r == sc.nbins_r && nbins_theta == sc.nbins_theta) {
      int nbins = nbins_r*nbins_theta;
      float nsum = 0.;
      for (int k = 0; k < nbins; k++)
        nsum += powf(hist.at<float>(0, k) - sc.hist.at<float>(0, k), 2.) / (hist.at<float>(0, k) + sc.hist.at<float>(0, k) + std::numeric_limits<double>::epsilon());
      return nsum / 2.;
    } else
      return -1.;
  }

  void print() {
    for (int r = 0; r < nbins_r; r++) {
      for (int t = 0; t < nbins_theta; t++)
        printf("%6.3lf ", hist.at<float>(0, r * nbins_theta + t));
      printf("\n");
    }
  }
};

/**
 * Represent a shape as cluster of training exemplars.
 * A query shape matches if its distance is within stddev.
 */
class ShapeModel {
public:
  ShapeContext mean; ///< mean shape context
  double stddev; ///< standard deviation of distance from cluster mean
  std::list<ShapeContext> exemplars; ///< individual training exemplars

public:

  ShapeModel() : stddev(0.) {
  }
  /**
   * Add a training exemplar, update stddev
   */
  void train(ShapeContext &sc);
  bool match(ShapeContext& sc, float &score, float &conf, ShapeContext& match);
  void print();
  void printDetailed();
  void write(const std::string &filename);
  void read(const std::string &filename);
};


void binaryEdgeImageToPoints(const cv::Mat &edgeImg, MYPOINT *&points, int &nPoints);
void calculateShapeContext(const cv::Mat &image, ShapeContext &sc);
void calculateShapeContextBGR(const cv::Mat &image, ShapeContext &sc, bool randomWarp = false);
void calculateShapeContextBGR(const cv::Mat &image, ShapeContext &sc, cv::Mat& crop, cv::Mat& gray, cv::Mat& edge, bool randomWarp = false);
void drawResults(MYPOINT *modelPoints, int nModelPoints, cv::Mat &image);
void drawResults(MYPOINT *modelPoints, int nModelPoints, MYPOINT *testPoints, int nTestPoints,
        const cv::Mat &modelImage, const cv::Mat &testImage, cv::Mat &dbgImage);
void printSC(int nbins_theta, int nbins_r, double *SC);
double matchEdgeImages(const cv::Mat &model, const cv::Mat &test);
cv::Mat matchAndAlignEdgeImages(const cv::Mat &model, const cv::Mat &test);
void trainModel(ShapeModel &shape, const cv::Mat &modelImage, int numWarps = 10);
void trainModel(ShapeModel &shape, const std::string &file, int numWarps = 10);
void trainModel(ShapeModel &shape, const std::vector<std::string> &files, int numWarps = 10);
void cropImage(const cv::Mat &image, cv::Mat &crop);
void addBorder(cv::Mat &image, int borderWidth = 60);
void removeBorderEdges(cv::Mat &image, cv::Mat &edge);
void warpPerspectiveRand(const cv::Mat& src, cv::Mat& dst, cv::RNG& rng);
void colorBackgroundBlack(cv::Mat &image);
