/**
 * Evaluate Yefeng Zheng's (University of Maryland, College Park) shape context.
 * @author Michael Zillich
 * @date December 2013
 *
 * TODO:
 *  - evaluate: warping interpolate linear, border edge removal with 3 pixel erosion, edge detect
 *              (linear interpolation smears the border, thus requiring more erosion)
 *          vs. warping interpolate nearest, border edge removal with 2 pixel erosion, edge detect
 *              (gives more blocky warped image, with worse edges)
 *          vs. edge detect, border edge removal with 2 pixel erosion, warping
 *              (warping smears the edges, image is no longer binary)
 *  - evaluate: number of warps for training
 *  - evaluate: stddev tolerance
 */

#include <stdlib.h>
#include <stdio.h>
#include <shapecontext.h>

using namespace cv;
using namespace std;

//#define DEBUG

int edgeThresh = 90;
int numTrainingWarps = 100;
Vec3b backgroundColor(0, 0, 192), black(0, 0, 0);
RNG rng;

/**
 * Background in ADE is marked with (0,0,192). This is changed to (0,0,0).
 */
static void colorBackgroundBlack(Mat &image)
{
  for(int i = 0; i < image.rows; i++)
    for(int j = 0; j < image.cols; j++)
      if(image.at<Vec3b>(i, j) == backgroundColor)
        image.at<Vec3b>(i, j) = black;
}

/**
 * Crop image to fit to the object, plus a border of some pixels to have room for warping.
 */
static void cropImage(const Mat &image, Mat &crop)
{
  int x1 = image.cols;
  int y1 = image.rows;
  int x2 = 0;
  int y2 = 0;
  int enlarge = 20;
  for(int i = 0; i < image.rows; i++)
    for(int j = 0; j < image.cols; j++)
      if(image.at<Vec3b>(i, j) != backgroundColor && image.at<Vec3b>(i, j) != black)
      {
        x1 = min(x1, j);
        y1 = min(y1, i);
        x2 = max(x2, j);
        y2 = max(y2, i);
      }
  // enlarge by 20 pixels to allow room for warping
  x1 = max(x1 - enlarge, 0);
  y1 = max(y1 - enlarge, 0);
  x2 = min(x2 + enlarge, image.cols - 1);
  y2 = min(y2 + enlarge, image.rows - 1);
  Mat roi(image, Rect(x1, y1, x2 - x1 + 1, y2 - y1 + 1));
  roi.copyTo(crop);
}

/**
 * This is for testing images of indivvidual segmented objecgs,
 * saved from ADE
 * Remove all edges caused by object borders, so only object internal
 * edges remain.
 * @param image 3 channel 8 bit unsigned char BGR images
 * @param edge single channel 8 bit unsigned char edge image, to be filtered
 */
static void removeBorderEdges(Mat &image, Mat &edge)
{
  if(image.rows != edge.rows && image.cols != edge.cols)
    return;
  Mat mask(image.rows, image.cols, CV_8UC1);
  mask.setTo(255);
  for(int i = 0; i < image.rows; i++)
    for(int j = 0; j < image.cols; j++)
      if(image.at<Vec3b>(i, j) == black)
        mask.at<unsigned char>(i, j) = 0;
  Mat element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
  erode(mask, mask, element, Point(-1,-1), 3);
  Mat tmp;
  edge.copyTo(tmp, mask);
  tmp.copyTo(edge);
}

static void warpPerspectiveRand( const Mat& src, Mat& dst, RNG& rng )
{
  Mat H(3, 3, CV_32FC1);
  H.at<float>(0,0) = rng.uniform( 0.8f, 1.2f);
  H.at<float>(0,1) = rng.uniform(-0.1f, 0.1f);
  H.at<float>(0,2) = rng.uniform(-0.1f, 0.1f)*src.cols;
  H.at<float>(1,0) = rng.uniform(-0.1f, 0.1f);
  H.at<float>(1,1) = rng.uniform( 0.8f, 1.2f);
  H.at<float>(1,2) = rng.uniform(-0.1f, 0.1f)*src.rows;
  H.at<float>(2,0) = rng.uniform( -1e-4f, 1e-4f);
  H.at<float>(2,1) = rng.uniform( -1e-4f, 1e-4f);
  H.at<float>(2,2) = rng.uniform( 0.8f, 1.2f);

  warpPerspective( src, dst, H, src.size() ); //INTER_NEAREST );
}

/**
 * Calculate shape context from BGR color image.
 * Runs edge detection and calculates shape context from binary edge image.
 */
static void calculateShapeContextBGR(const Mat &image, ShapeContext &sc,  bool randomWarp = false)
{
  Mat crop, tmp, gray, edge;

  cropImage(image, crop);
  colorBackgroundBlack(crop);
  if(randomWarp)
    warpPerspectiveRand(crop, tmp, rng);
  else
    tmp = crop;
  cvtColor(tmp, gray, CV_BGR2GRAY);
  blur(gray, edge, Size(3,3));
  Canny(edge, edge, edgeThresh, edgeThresh*3, 3);
  removeBorderEdges(tmp, edge);
  calculateShapeContext(edge, sc);

#ifdef DEBUG
  imshow("BGR", tmp);
  imshow("edge", edge);
  waitKey(0);
#endif
}

/**
 * @param modelImage 3 channel 8bit unsigned char BGR image of model
 * @param testImage 3 channel 8bit unsigned char BGR image to test against model
 * @param randomWarp if true, perform a random perspective warp on the test
 *                   image before matching
 */
static double doMatch(Mat &modelImage, Mat &testImage, bool randomWarp = false)
{
  ShapeContext sc1, sc2;
  calculateShapeContextBGR(modelImage, sc1, false);
  calculateShapeContextBGR(testImage, sc2, randomWarp);

  double match = sc1.distance(sc2);

#ifdef DEBUG
  //imshow("model", modelEdge);
  //imshow("test", testEdge);
  //waitKey(0);
#endif

  return match;
}

static void trainModel(ShapeModel &shape, Mat &modelImage, int numWarps = 10)
{
  ShapeContext sc;

  // first exemplar is the original
  calculateShapeContextBGR(modelImage, sc, false);
  shape.train(sc);

  // now create a few warped exemplars
  for(int i = 0; i < numWarps; i++)
  {
    calculateShapeContextBGR(modelImage, sc, true);
    shape.train(sc);
  }
}

static void trainModel(ShapeModel &shape, string &file, int numWarps = 10)
{
  Mat modelImage = imread(file.c_str(), 1);
  if(modelImage.empty())
  {
    printf("Cannot read image file: %s\n", file.c_str());
    exit(EXIT_FAILURE);
  }
  trainModel(shape, modelImage, numWarps);
}

static void trainModel(ShapeModel &shape, vector<string> &files, int numWarps = 10)
{
  for(size_t i = 0; i < files.size(); i++)
    trainModel(shape, files[i], numWarps);
}

static void evalModel(ShapeModel &shape, vector<string> &files)
{
  float scores[files.size()];
  bool isMatch[files.size()];

  for(size_t i = 0; i < files.size(); i++)
  {
    Mat testImage = imread(files[i].c_str(), 1);
    if(testImage.empty())
    {
      printf("Cannot read image file: %s\n", files[i].c_str());
      exit(EXIT_FAILURE);
    }
    ShapeContext sc;
    calculateShapeContextBGR(testImage, sc);
    isMatch[i] = shape.match(sc, scores[i]);
  }

  for(size_t i = 0; i < files.size(); i++)
    printf("%s  %.6f %s\n", files[i].c_str(), scores[i], isMatch[i] ? "MATCH" : "");
}

/**
 * Test an image against warped versions of itself, print matching scores and stddev.
 */
static void selfAgainstWarps(string &file, int numWarps = 10)
{
  double scores[numWarps];
  double stddev = 0.;

  Mat modelImage = imread(file.c_str(), 1);
  if(modelImage.empty())
  {
    printf("Cannot read image file: %s\n", file.c_str());
    exit(EXIT_FAILURE);
  }
  for(int i = 0; i < numWarps; i++)
  {
    Mat testImage;
    modelImage.copyTo(testImage);
    scores[i] = doMatch(modelImage, testImage, true);
    printf("random warp %lf\n", scores[i]);
  }
  for(int i = 0; i < numWarps; i++)
    stddev += scores[i]*scores[i];
  stddev /= (double)numWarps;
  stddev = sqrt(stddev);
  printf("stddev: %lf\n", stddev);
}

static void selfAgainstWarps(vector<string> &files, int numWarps = 10)
{
  for(size_t i = 0; i < files.size(); i++)
    selfAgainstWarps(files[i], numWarps);
}

/**
 * Test each image agaist each other, print the confusion matrix.
 */
static void allAgainstAll(vector<string> &files)
{
  double confusion[files.size()][files.size()];

  for(size_t i = 0; i < files.size(); i++)
  {
    Mat modelImage = imread(files[i].c_str(), 1);
    if(modelImage.empty())
    {
      printf("Cannot read image file: %s\n", files[i].c_str());
      exit(EXIT_FAILURE);
    }

    for(size_t j = 0; j < files.size(); j++)
    {
      Mat testImage = imread(files[j].c_str(), 1);
      if(testImage.empty())
      {
        printf("Cannot read image file: %s\n", files[j].c_str());
        exit(EXIT_FAILURE);
      }
      confusion[i][j] = doMatch(modelImage, testImage);
    }
  }

  for(size_t i = 0; i < files.size(); i++)
  {
    for(size_t j = 0; j < files.size(); j++)
      printf("%.6f ", confusion[i][j]);
    printf("\n");
  }
}

/**
 * Test the first image against all the others, print matching scores.
 */
static void firstAgainstRest(vector<string> &files)
{
  double scores[files.size() - 1];

  Mat modelImage = imread(files[0].c_str(), 1);
  if(modelImage.empty())
  {
    printf("Cannot read image file: %s\n", files[0].c_str());
    exit(EXIT_FAILURE);
  }

  for(size_t i = 1; i < files.size(); i++)
  {
    Mat testImage = imread(files[i].c_str(), 1);
    if(testImage.empty())
    {
      printf("Cannot read image file: %s\n", files[i].c_str());
      exit(EXIT_FAILURE);
    }
    scores[i-1] = doMatch(modelImage, testImage);
  }

  printf("%s against:\n", files[0].c_str());
  for(size_t i = 0; i < files.size() - 1; i++)
    printf("%s: %.6f\n", files[i+1].c_str(), scores[i]);
}

/**
 * Build a shape model from the first image, using a few warps for training.
 * Then test against all the others, print matching scores.
 */
static void firstShapeModelAgainstRest(vector<string> &files)
{
  ShapeModel shape;
  float scores[files.size() - 1];
  bool isMatch[files.size() - 1];

  Mat modelImage = imread(files[0].c_str(), 1);
  if(modelImage.empty())
  {
    printf("Cannot read image file: %s\n", files[0].c_str());
    exit(EXIT_FAILURE);
  }
  trainModel(shape, modelImage, numTrainingWarps);

  printf("trained model:\n");
  shape.print();
#ifdef DEBUG
  shape.printDetailed();
#endif

  // HACK
  shape.write("shape.xml");
  ShapeModel shipe;
  shipe.read("shape.xml");
  shipe.print();
#ifdef DEBUG
  shipe.printDetailed();
  shipe.write("shipe.xml");
#endif

  for(size_t i = 1; i < files.size(); i++)
  {
    Mat testImage = imread(files[i].c_str(), 1);
    if(testImage.empty())
    {
      printf("Cannot read image file: %s\n", files[i].c_str());
      exit(EXIT_FAILURE);
    }
    ShapeContext sc;
    calculateShapeContextBGR(testImage, sc);
    isMatch[i-1] = shape.match(sc, scores[i-1]);
  }

  printf("%s against:\n", files[0].c_str());
  for(size_t i = 0; i < files.size() - 1; i++)
    printf("%s  %.6f %s\n", files[i+1].c_str(), scores[i], isMatch[i] ? "MATCH" : "");
}

int main( int argc, char** argv )
{
  enum Mode {TRAIN, EVAL};
  Mode mode = TRAIN;
  string modelfile;
  vector<string> files;

  int c = 0;
  while(c != -1)
  {
    c = getopt(argc, argv, "t:e:");
    switch(c)
    {
      case 't':
        modelfile = optarg;
        mode = TRAIN;
        break;
      case 'e':
        modelfile = optarg;
        mode = EVAL;
        break;
      default:
        break;
    }
  }
  // remaining arguments are image names
  while(optind < argc)
  {
    files.push_back(argv[optind++]);
  }

  // the random number generator of this thread
  rng = theRNG();

  if(mode == TRAIN)
  {
    ShapeModel shape;
    trainModel(shape, files, numTrainingWarps);
    shape.write(modelfile);
    printf("trained model:\n");
    shape.print();
#ifdef DEBUG
    shape.printDetailed();
#endif
  }
  else if(mode == EVAL)
  {
    ShapeModel shape;
    shape.read(modelfile);
    printf("loaded model:\n");
    shape.print();
#ifdef DEBUG
    shape.printDetailed();
#endif
    evalModel(shape, files);
  }
  //allAgainstAll(files);
  //firstAgainstRest(files);
  //selfAgainstWarps(files);
  //firstShapeModelAgainstRest(files);

  exit(EXIT_SUCCESS);
}
