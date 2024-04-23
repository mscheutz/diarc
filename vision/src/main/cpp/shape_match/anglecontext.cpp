/**
 * anglecontext
 * test shape context like descriptor base on angle ratio
 *
 * @author: Michael Zillich
 * @date Dec. 2013
 */

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>

using namespace cv;
using namespace std;

class Histogram
{
private:
  vector<float> bins;
  float binSize;

public:
  Histogram(int _numBins = 100, float _binSize = 0.1)
  : bins(_numBins), binSize(_binSize)
  {
  }
  void clear()
  {
    bins.assign(bins.size(), 0.);
  }
  void insert(float f)
  {
    size_t i = f < 0 ? 0 : f/binSize;
    if(i >= bins.size())
      i = bins.size() - 1;
    bins[size_t(i)] += 1.;
  }
  void normalize()
  {
    float total = 0.;
    for(size_t i = 0; i < bins.size(); i++)
      total += bins[i];
    for(size_t i = 0; i < bins.size(); i++)
      bins[i] /= total;
  }
  void asciiDisplay(int numCols = 80)
  {
    for(int c = 0; c < numCols; c++)
      printf("-");
    printf("\n");
    for(size_t i = 0; i < bins.size(); i++)
    {
      printf("%6.3f ", i*binSize);
      int c = (int)(bins[i]*(float)numCols);
      for(; c > 0; c--)
        printf("X");
      printf("\n");
    }
    for(int c = 0; c < numCols; c++)
      printf("-");
    printf("\n");
  }
};

int edgeThresh = 1;
Mat image, gray, edge, cedge;
Mat debug;

static bool isZero(float a)
{
  return fpclassify(a) == FP_ZERO;
}

static bool isZero(double a)
{
  return fpclassify(a) == FP_ZERO;
}

static void binaryEdgeImageToEdgelVector(Mat &edgeImg, vector<Vec2f> &edgels)
{
  edgels.clear();
  for(int i = 0; i < edgeImg.rows; i++)
    for(int j = 0; j < edgeImg.cols; j++)
    {
       if(edgeImg.at<unsigned char>(i, j) > 0)
         edgels.push_back(Vec2f((float)j, (float)i));
    }
}

static float calculateDistance(Vec2f &a, Vec2f &b)
{
  Vec2f ab = b - a;
  return norm(ab);  
}

/**
 * Returns the angle ratio of angles abc and acd, or 0 for degenerate cases.
 */
static float calculateAngleRatio(Vec2f &a, Vec2f &b, Vec2f &c, Vec2f &d)
{
  float tiny = 1e-9;
  Vec2f ab = b - a;
  Vec2f ac = c - a;
  Vec2f ad = d - a;
  float l;
  if((l = norm(ab)) > tiny)
    ab /= l;
  else
    return 0.;
  if((l = norm(ac)) > tiny)
    ac /= l;
  else
    return 0.;
  if((l = norm(ad)) > tiny)
    ad /= l;
  else
    return 0.;

  float alpha = acos(ab.dot(ac));
  float beta = acos(ac.dot(ad));
  if(isZero(beta))
    return 0.;

  CvScalar col = cvScalar(rand()%255, rand()%255, rand()%255);
  line(debug, cvPoint(a[0], a[1]), cvPoint(b[0], b[1]), col);
  line(debug, cvPoint(a[0], a[1]), cvPoint(c[0], c[1]), col);
  line(debug, cvPoint(a[0], a[1]), cvPoint(d[0], d[1]), col);

  return alpha/beta;
}

static void calculateARHistogram(vector<Vec2f> &edgels, Histogram &hist)
{
  const int NUM_SAMPLES = 1000;

  if(edgels.size() < 4)
    return;

  hist.clear();
  for(int n = 0; n < NUM_SAMPLES; n++)
  {
    int i = rand()%(int)edgels.size();
    int j = rand()%(int)edgels.size();
    int k = rand()%(int)edgels.size();
    int l = rand()%(int)edgels.size();

    float angleRatio = calculateAngleRatio(edgels[i], edgels[j], edgels[k], edgels[l]);
    if(!isZero(angleRatio))
    {
      hist.insert(angleRatio);
      // printf("%d %d %d %d  %.3f\n", i, j, k, l, angleRatio);  // HACK
    }
    //hist.insert(calculateDistance(edgels[i], edgels[j]));  // HACK
  }
  hist.normalize();
}

// define a trackbar callback
static void onTrackbar(int, void*)
{
    blur(gray, edge, Size(3,3));

    // Run the edge detector on grayscale
    Canny(edge, edge, edgeThresh, edgeThresh*3, 3);

    image.copyTo(debug);
    vector<Vec2f> edgels;
    Histogram hist(20, .5);
    binaryEdgeImageToEdgelVector(edge, edgels);
    calculateARHistogram(edgels, hist);
    hist.asciiDisplay();

    cedge = Scalar::all(0);
    image.copyTo(cedge, edge);
    imshow("Edge map", cedge);
    imshow("debug", debug);
}

static void help()
{
    printf("\nThis sample demonstrates Canny edge detection\n"
           "Call:\n"
           "    /.edge [image_name -- Default is fruits.jpg]\n\n");
}

const char* keys =
{
    "{1| |fruits.jpg|input image name}"
};

int main( int argc, const char** argv )
{
    help();

    CommandLineParser parser(argc, argv, keys);
    string filename = parser.get<string>("1");

    image = imread(filename, 1);
    if(image.empty())
    {
        printf("Cannot read image file: %s\n", filename.c_str());
        help();
        return -1;
    }
    cedge.create(image.size(), image.type());
    cvtColor(image, gray, CV_BGR2GRAY);

    // Create a window
    namedWindow("Edge map", 1);

    // create a toolbar
    createTrackbar("Canny threshold", "Edge map", &edgeThresh, 100, onTrackbar);

    // Show the image
    onTrackbar(0, 0);

    // Wait for a key stroke; the same function arranges events processing
    waitKey(0);

    return 0;
}
