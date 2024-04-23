/**
 * Test Yefeng Zheng's (University of Maryland, College Park) shape context.
 * @author Michael Zillich
 * @date December 2013
 */

#include "shape_match/shapecontext.h"

using namespace cv;
using namespace std;

//#define DEBUG

int edgeThresh = 90;
int numTrainingWarps = 100;
Vec3b backgroundColor(0, 0, 192), black(0, 0, 0);
RNG rng = theRNG();

//Parameters which may effect the performance
extern double T_Init; //Temparature of the Gibbs distribution.
//Used to transform the shape context distance to a probability measure.
int E_Ave = 5; //Average number of edges per point
double lambda_o = 1; //Regularization parameter (normalized) of the TPS deformation
int I_Max = 10; //Maximum number of iteration

//Flags of different options
int rotate_invariant_flag = 1; //Default: Use rotation invariance
int relax_graph_match_flag = 1; //Default: Using the relaxation labeling method for graph matching
int affine_LMS_flag = 1; //Default: Using LMS for the first iteration
int all_match_flag = 0; //Default: Do not force to find as many matches as possible
int init_label_outlier_flag = 0; //Default: Do not use heuristic rule to label outliers
int neighborhood_flag = 0; //Default: Simple neighborhood definition.
//         0 -- Simple neighborhood
//         1 -- Neighborhood definition robust under non-Uniform scale changes

//Parameter of the shape context
int nbins_theta = 12; //Number of bins in angle
//Decreasing nbins_theta to eight for the first iteration does not improve the performance.
int nbins_r = 5; //Number of bins in distance
double r_inner = 1.0 / 8; //The minimum distance
double r_outer = 2.0; //The maximum distance
double mean_dist_deform, mean_dist_model; //Mean distances

//Some global variables
double r_bins_edges[10]; //Lattice of radius
double **SCModel = NULL; //Shape contexts for the model shape
double **SCDeform = NULL; //Shape contexts for the deformed shape
double **r_array = NULL; //Distance array
double **theta_array = NULL; //Theta angle array
double **costmat = NULL; //Matrix of the shape context distances between points from two shapes
double *TPS = NULL; //Matrix of the TPS transform
double *InvTPS = NULL; //Inversion of the matrix of the TPS transform

/*****************************************************************************
/*      Name:           AllocateMemory
/*      Function:       Allocate memory for global variables
/*      Parameter:      nMaxPnt -- Maximum point number
/*      Return:         0 -- Succeed
/*****************************************************************************/
int AllocateMemory(int nMaxPnt) {
  SCModel = (double**) malloc(sizeof (double*)*nMaxPnt);
  SCDeform = (double**) malloc(sizeof (double*)*nMaxPnt);

  r_array = (double**) malloc(sizeof (double*)*nMaxPnt);
  theta_array = (double**) malloc(sizeof (double*)*nMaxPnt);

  costmat = (double**) malloc(sizeof (double*)*nMaxPnt);
  if (SCModel == NULL || SCDeform == NULL ||
          r_array == NULL || theta_array == NULL ||
          costmat == NULL) {
    printf("Memory used up!\n");
    return -1;
  }

  int nFeature = nbins_theta*nbins_r;
  for (int i = 0; i < nMaxPnt; i++) {
    SCModel[i] = (double*) malloc(sizeof (double)*nFeature);
    SCDeform[i] = (double*) malloc(sizeof (double)*nFeature);
    r_array[i] = (double*) malloc(sizeof (double)*nMaxPnt);
    theta_array[i] = (double*) malloc(sizeof (double)*nMaxPnt);
  }
  for (int i = 0; i < nMaxPnt; i++) {
    costmat[i] = (double*) malloc(sizeof (double)*nMaxPnt);
    for (int j = 0; j < nMaxPnt; j++)
      costmat[i][j] = 0;
  }
  TPS = (double*) malloc(sizeof (double)*(nMaxPnt + 3)*(nMaxPnt + 3));
  InvTPS = (double*) malloc(sizeof (double)*(nMaxPnt + 3)*(nMaxPnt + 3));
  return 0;
}

/*****************************************************************************
/*      Name:           FreeMemory
/*      Function:       Free memory used by global variables
/*      Parameter:      nMaxPnt -- Maximum point number
/*      Return:         0 -- Succeed
/*****************************************************************************/
int FreeMemory(int nMaxPnt) {
  int i;
  if (SCModel != NULL) {
    for (int i = 0; i < nMaxPnt; i++)
      free(SCModel[i]);
    free(SCModel);
  }

  if (SCDeform != NULL) {
    for (int i = 0; i < nMaxPnt; i++)
      free(SCDeform[i]);
    free(SCDeform);
  }

  if (r_array != NULL) {
    for (int i = 0; i < nMaxPnt; i++)
      free(r_array[i]);
    free(r_array);
  }

  if (theta_array != NULL) {
    for (int i = 0; i < nMaxPnt; i++)
      free(theta_array[i]);
    free(theta_array);
  }

  if (costmat != NULL) {
    for (i = 0; i < nMaxPnt; i++)
      free(costmat[i]);
    free(costmat);
  }

  if (TPS != NULL)
    free(TPS);
  if (InvTPS != NULL)
    free(InvTPS);
  return 0;
}

void binaryEdgeImageToPoints(const Mat &edgeImg, MYPOINT *&points, int &nPoints) {
  int MAX_POINTS = 400;
  nPoints = 0;
  for (int i = 0; i < edgeImg.rows; i++)
    for (int j = 0; j < edgeImg.cols; j++)
      if (edgeImg.at<unsigned char>(i, j) > 0)
        nPoints++;
  points = new MYPOINT[nPoints];
  int cnt = 0;
  for (int i = 0; i < edgeImg.rows; i++)
    for (int j = 0; j < edgeImg.cols; j++) {
      if (edgeImg.at<unsigned char>(i, j) > 0) {
        points[cnt].x = (double) j;
        points[cnt].y = (double) i;
        cnt++;
      }
    }
  static bool downSample = false;
  if (downSample && nPoints > MAX_POINTS) {
#ifdef DEBUG
    printf("reducing from %d points\n", nPoints);
#endif
    MYPOINT *reduced = new MYPOINT[MAX_POINTS];
    float step = (float) nPoints / (float) MAX_POINTS;
    for (int i = 0; i < MAX_POINTS; i++)
      reduced[i] = points[(int) ((float) i * step)];
    delete[] points;
    points = reduced;
    nPoints = MAX_POINTS;
  }
#ifdef DEBUG
  printf("have %d points\n", nPoints);
#endif
}

/**
 * Draw matches into an image, using OpenCV drawMatches() function.
 */
void drawResults(MYPOINT *modelPoints, int nModelPoints, Mat &image) {
  for (int i = 0; i < nModelPoints; i++) {
    image.at<unsigned char>(static_cast<int>(modelPoints[i].y), static_cast<int>(modelPoints[i].x)) = static_cast<unsigned char>(255);
  }
}

/**
 * Draw matches into an image, using OpenCV drawMatches() function.
 */
void drawResults(MYPOINT *modelPoints, int nModelPoints, MYPOINT *testPoints, int nTestPoints,
        const Mat &modelImage, const Mat &testImage, Mat &dbgImage) {
  vector<KeyPoint> keypoints1(nModelPoints), keypoints2(nTestPoints);
  vector<DMatch> matches1to2;

  for (int i = 0; i < nModelPoints; i++) {
    keypoints1[i].pt.x = modelPoints[i].x;
    keypoints1[i].pt.y = modelPoints[i].y;
  }
  for (int i = 0; i < nTestPoints; i++) {
    keypoints2[i].pt.x = testPoints[i].x;
    keypoints2[i].pt.y = testPoints[i].y;
  }
  for (int i = 0; i < nModelPoints; i++) {
    if (modelPoints[i].nMatch >= 0) {
      DMatch match;
      match.trainIdx = i;
      match.queryIdx = modelPoints[i].nMatch;
      matches1to2.push_back(match);
    }
  }

  drawMatches(modelImage, keypoints1,
          testImage, keypoints2,
          matches1to2, dbgImage);
}

void printSC(int nbins_theta, int nbins_r, double *SC) {
  for (int r = 0; r < nbins_r; r++) {
    for (int t = 0; t < nbins_theta; t++)
      printf("%6.3lf ", SC[r * nbins_theta + t]);
    printf("\n");
  }
  printf("\n");
}

/**
 * Add the shape contexts from all points into one shape context, i.e. add the histograms.
 */
static void sumShapeContext(int nPnt, int nbins_theta, int nbins_r, double **SC, double *sumSC) {
  int nbins = nbins_r*nbins_theta;
  memset(sumSC, 0, sizeof (double)*nbins_r * nbins_theta);
  for (int i = 0; i < nPnt; i++)
    for (int j = 0; j < nbins; j++)
      sumSC[j] += SC[i][j];
  for (int j = 0; j < nbins; j++)
    sumSC[j] /= (double) nPnt;
}

/**
 * Distance between two shape histograms, using chi-square distance.
 */
static double SCDist(int nbins_theta, int nbins_r, double *SC1, double *SC2) {
  int nbins = nbins_r*nbins_theta;
  double nsum = 0;
  for (int k = 0; k < nbins; k++)
    nsum += (SC1[k] - SC2[k]) * (SC1[k] - SC2[k]) / (SC1[k] + SC2[k] + std::numeric_limits<double>::epsilon());
  return nsum / 2.;
}

/**
 * Calculate shape context from binary edge image.
 */
void calculateShapeContext(const Mat &image, ShapeContext &sc) {
  MYPOINT *Pnt = NULL;
  int nPnt;

  binaryEdgeImageToPoints(image, Pnt, nPnt);
  // if there were no edge points, just leave sc all zeros
  if (nPnt == 0)
    return;

  AllocateMemory(nPnt);
  CalRBinEdge(nbins_r, r_inner, r_outer, r_bins_edges);
  for (int i = 0; i < nPnt; i++)
    Pnt[i].nMatch = 0;

  if (rotate_invariant_flag)
    nbins_theta = 4;
  else
    nbins_theta = 12;

  // Calculate the reference angles
  CalRefAngle(Pnt, nPnt);
  // Calculate the distance array
  CalPointDist(Pnt, nPnt, r_array, mean_dist_model);
  // Compute shape context for all points on the model shape
  CalShapeContext(Pnt, nPnt, nbins_theta, nbins_r, r_bins_edges, SCModel, r_array, mean_dist_model, rotate_invariant_flag);

  // Calculate complete shape context as sum of all shape context histograms
  double *sum = new double[nbins_r * nbins_theta];
  sumShapeContext(nPnt, nbins_theta, nbins_r, SCModel, sum);
  sc.resize(nbins_r, nbins_theta);
  for (int i = 0; i < nbins_r * nbins_theta; i++)
    sc.hist.at<float>(0, i) = sum[i];

  FreeMemory(nPnt);
  delete[] Pnt;
  delete[] sum;
}

/**
 * Calculate shape context from BGR color image.
 * Runs edge detection and calculates shape context from binary edge image.
 */
void calculateShapeContextBGR(const Mat &image, ShapeContext &sc, bool randomWarp) {
  Mat crop, gray, edge;
  calculateShapeContextBGR(image, sc, crop, gray, edge, randomWarp);
}

/**
 * Calculate shape context from BGR color image.
 * Runs edge detection and calculates shape context from binary edge image.
 */
void calculateShapeContextBGR(const Mat &image, ShapeContext &sc, Mat& crop, Mat& gray, Mat& edge, bool randomWarp) {
  Mat tmp;

  cropImage(image, crop);
  addBorder(crop);
  colorBackgroundBlack(crop);
  if (randomWarp)
    warpPerspectiveRand(crop, tmp, rng);
  else
    tmp = crop;
  cvtColor(tmp, gray, cv::COLOR_BGR2GRAY);
  blur(gray, edge, Size(3, 3));
  Canny(edge, edge, edgeThresh, edgeThresh * 3, 3);
  removeBorderEdges(tmp, edge);
  calculateShapeContext(edge, sc);

  //  imshow("image", image);
  //  imshow("crop", crop);
  //  imshow("BGR", tmp);
  //  imshow("edge", edge);
  //  sleep(1);

#ifdef DEBUG
  imshow("BGR", tmp);
  imshow("edge", edge);
  waitKey(0);
#endif
}

/**
 * Match model and test binary edge images using shape context histograms.
 * @param model single channel 8bit unsigned char image of model
 * @param test single channel 8bit unsigned char image to test against model
 */
double matchEdgeImages(const Mat &model, const Mat &test) {
  MYPOINT *PntModel = NULL; //Model point set
  MYPOINT *PntDeform = NULL; //Deformed point set
  int nPntModel, nPntDeform;

  // fill point arrays
  binaryEdgeImageToPoints(model, PntModel, nPntModel);
  binaryEdgeImageToPoints(test, PntDeform, nPntDeform);

  MYPOINT *PntModel2 = new MYPOINT[nPntModel]; //Working point set for shape matching
  int nMaxPnt = std::max(nPntModel, nPntDeform);

  //Allocate memory
  AllocateMemory(nMaxPnt);
  CalRBinEdge(nbins_r, r_inner, r_outer, r_bins_edges);

  //Initialization working point set
  for (int i = 0; i < nPntModel; i++) {
    PntModel[i].nMatch = 0;
    PntModel2[i] = PntModel[i];
  }

  if (rotate_invariant_flag)
    nbins_theta = 4;
  else
    nbins_theta = 12;

  //All points are used to calculate shape context. No outliers.
  for (int i = 0; i < nPntModel; i++)
    PntModel2[i].nMatch = 0;
  for (int i = 0; i < nPntDeform; i++)
    PntDeform[i].nMatch = 0;

  //Calculate the reference angles
  CalRefAngle(PntModel2, nPntModel);
  //Calculate the distance array
  CalPointDist(PntModel2, nPntModel, r_array, mean_dist_model);
  if (relax_graph_match_flag) //Set edges of the model graph
  {
    if (neighborhood_flag == 0)
      SetEdgeSimple(PntModel2, nPntModel, r_array, E_Ave);
    else
      SetEdgeNonUniformScale(PntModel2, nPntModel, r_array, E_Ave);
  }

  //Compute shape context for all points on the model shape
  CalShapeContext(PntModel2, nPntModel, nbins_theta, nbins_r, r_bins_edges, SCModel, r_array, mean_dist_model, rotate_invariant_flag);

  //Calculate the reference angles
  CalRefAngle(PntDeform, nPntDeform);
  //Calculate the distance array
  CalPointDist(PntDeform, nPntDeform, r_array, mean_dist_deform);
  if (relax_graph_match_flag) //Set edges of the deformed graph
  {
    if (neighborhood_flag == 0)
      SetEdgeSimple(PntDeform, nPntDeform, r_array, E_Ave);
    else
      SetEdgeNonUniformScale(PntDeform, nPntDeform, r_array, E_Ave);
  }

  //Compute shape context for all points on the deformed shape
  CalShapeContext(PntDeform, nPntDeform, nbins_theta, nbins_r, r_bins_edges, SCDeform, r_array, mean_dist_deform, rotate_invariant_flag);

  double *sumSCModel = new double[nbins_r * nbins_theta];
  sumShapeContext(nPntModel, nbins_theta, nbins_r, SCModel, sumSCModel);
  double *sumSCDeform = new double[nbins_r * nbins_theta];
  sumShapeContext(nPntDeform, nbins_theta, nbins_r, SCDeform, sumSCDeform);
  double dist = SCDist(nbins_theta, nbins_r, sumSCModel, sumSCDeform);

#ifdef DEBUG
  printf("model:\n");
  printSC(nbins_theta, nbins_r, sumSCModel);
  printf("test:\n");
  printSC(nbins_theta, nbins_r, sumSCDeform);
  printf("dist: %lf\n", dist);
#endif

  //Calculate the shape context distance between any point pair of two shapes
  //HistCost( SCModel, nPntModel, SCDeform, nPntDeform, nbins_theta*nbins_r, costmat );

  //Free memory
  delete[] PntModel;
  delete[] PntDeform;
  delete[] PntModel2;
  delete[] sumSCModel;
  delete[] sumSCDeform;

  return dist;
}

/**
 * Match model and test binary edge images using shape context plus alignment using graph matching.
 * @param model single channel 8bit unsigned char image of model
 * @param test single channel 8bit unsigned char image to test against model
 */
Mat matchAndAlignEdgeImages(const Mat &model, const Mat &test) {
  MYPOINT *PntModel = NULL; //Model point set
  MYPOINT *PntDeform = NULL; //Deformed point set
  int nPntModel, nPntDeform;

  // fill point arrays
  binaryEdgeImageToPoints(model, PntModel, nPntModel);
  binaryEdgeImageToPoints(test, PntDeform, nPntDeform);

  //PointMatch() function will change PntModel, keep a copy of PntModel
  MYPOINT *PntModel2 = new MYPOINT[nPntModel];
  for (int i = 0; i < nPntModel; i++)
    PntModel2[i] = PntModel[i];

  //Matching two sets of points
  //PntModel is warped toward PntDeform after point matching
  int nMinPnt = std::min(nPntModel, nPntDeform);
  int nMaxPnt = std::max(nPntModel, nPntDeform);
  if (nMinPnt > nMaxPnt / 2) //The outlier rate is not too high
    PointMatch(PntModel, nPntModel, PntDeform, nPntDeform);
  else
    PointMatchOutlier(PntModel, nPntModel, PntDeform, nPntDeform);

  //Since the point positions may be changed after matching, we copy matching results only.
  for (int i = 0; i < nPntModel; i++)
    PntModel2[i].nMatch = PntModel[i].nMatch;

  int nMatch = 0;
  for (int i = 0; i < nPntModel; i++) {
    if (PntModel[i].nMatch >= 0)
      nMatch++;
  }
#ifdef DEBUG
  printf("matching %d / %d\n", nMatch, nPntModel);
#endif
  Mat dbgImage;
  drawResults(PntModel2, nPntModel, PntDeform, nPntDeform, model, test, dbgImage);

  //Free memory
  delete PntModel;
  delete PntDeform;
  delete PntModel2;

  return dbgImage;
}

void trainModel(ShapeModel &shape, const Mat &modelImage, int numWarps) {
  ShapeContext sc;

  // first exemplar is the original
  calculateShapeContextBGR(modelImage, sc, false);
  shape.train(sc);

  // now create a few warped exemplars
  for (int i = 0; i < numWarps; i++) {
    calculateShapeContextBGR(modelImage, sc, true);
    shape.train(sc);
  }
}

void trainModel(ShapeModel &shape, const std::string &file, int numWarps) {
  Mat modelImage = imread(file.c_str(), 1);
  if (modelImage.empty()) {
    printf("Cannot read image file: %s\n", file.c_str());
    exit(EXIT_FAILURE);
  }
  trainModel(shape, modelImage, numWarps);
}

void trainModel(ShapeModel &shape, const std::vector<std::string> &files, int numWarps) {
  for (size_t i = 0; i < files.size(); i++)
    trainModel(shape, files[i], numWarps);
}

/**
 * Crop image to fit to the object.
 */
void cropImage(const Mat &image, Mat &crop) {
  int x1 = image.cols;
  int y1 = image.rows;
  int x2 = 0;
  int y2 = 0;
  for (int i = 0; i < image.rows; i++)
    for (int j = 0; j < image.cols; j++)
      if (image.at<Vec3b>(i, j) != backgroundColor && image.at<Vec3b>(i, j) != black) {
        x1 = min(x1, j);
        y1 = min(y1, i);
        x2 = max(x2, j);
        y2 = max(y2, i);
      }
  Mat roi(image, Rect(x1, y1, x2 - x1 + 1, y2 - y1 + 1));
  roi.copyTo(crop);
}

/**
 * Add a border of some pixels to have room for warping.
 * Note: changes images in place.
 */
void addBorder(Mat &image, int borderWidth) {
  Mat enlargedImage(image.rows + 2 * borderWidth, image.cols + 2 * borderWidth, image.type(), Scalar(0));
  Mat roiImage(enlargedImage, Rect(borderWidth, borderWidth, image.cols, image.rows));
  image.copyTo(roiImage);
  enlargedImage.copyTo(image);
}

/**
 * This is for testing images of indivvidual segmented objecgs,
 * saved from ADE
 * Remove all edges caused by object borders, so only object internal
 * edges remain.
 * @param image 3 channel 8 bit unsigned char BGR images
 * @param edge single channel 8 bit unsigned char edge image, to be filtered
 */
void removeBorderEdges(Mat &image, Mat &edge) {
  if (image.rows != edge.rows && image.cols != edge.cols)
    return;
  Mat mask(image.rows, image.cols, CV_8UC1);
  mask.setTo(255);
  for (int i = 0; i < image.rows; i++)
    for (int j = 0; j < image.cols; j++)
      if (image.at<Vec3b>(i, j) == black)
        mask.at<unsigned char>(i, j) = 0;
  Mat element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
  erode(mask, mask, element, Point(-1, -1), 3);
  Mat tmp;
  edge.copyTo(tmp, mask);
  tmp.copyTo(edge);
}

void warpPerspectiveRand(const Mat& src, Mat& dst, RNG& rng) {
  Mat H(3, 3, CV_32FC1);
  H.at<float>(0, 0) = rng.uniform(0.8f, 1.2f);
  H.at<float>(0, 1) = rng.uniform(-0.1f, 0.1f);
  H.at<float>(0, 2) = rng.uniform(-0.1f, 0.1f) * src.cols;
  H.at<float>(1, 0) = rng.uniform(-0.1f, 0.1f);
  H.at<float>(1, 1) = rng.uniform(0.8f, 1.2f);
  H.at<float>(1, 2) = rng.uniform(-0.1f, 0.1f) * src.rows;
  H.at<float>(2, 0) = rng.uniform(-1e-4f, 1e-4f);
  H.at<float>(2, 1) = rng.uniform(-1e-4f, 1e-4f);
  H.at<float>(2, 2) = rng.uniform(0.8f, 1.2f);

  warpPerspective(src, dst, H, src.size()); //INTER_NEAREST );
}

/**
 * Background in ADE is marked with (0,0,192). This is changed to (0,0,0).
 */
void colorBackgroundBlack(Mat &image) {
  for (int i = 0; i < image.rows; i++)
    for (int j = 0; j < image.cols; j++)
      if (image.at<Vec3b>(i, j) == backgroundColor)
        image.at<Vec3b>(i, j) = black;
}

void ShapeModel::train(ShapeContext &sc) {
  if (exemplars.size() == 0) {
    exemplars.push_back(sc);
    mean = sc;
    // choose some non-zero sttdev, so matching with only a single trained
    // exemplar has a chance
    stddev = 0.2;
  } else {
    float n = (float) exemplars.size();
    exemplars.push_back(sc);
    // update sliding mean
    mean.hist *= n;
    mean.hist += sc.hist;
    mean.hist /= (n + 1.);
    // calculate new stddev
    stddev = 0.;
    for (list<ShapeContext>::iterator it = exemplars.begin(); it != exemplars.end(); it++)
      stddev += powf(mean.distance(*it), 2.);
    stddev /= (n + 1.);
    stddev = sqrt(stddev);
  }
}

/**
 * @param sc shape context to match
 * @param score best matching score (= distance) amongst all exemplars
 * @param conf  value between [0 1] describing confidence in match
 * @return true if distance to mean is within stddev*tolerance
 * NOTE: This is a very inefficient nearest neighbour search!
 */
bool ShapeModel::match(ShapeContext& sc, float &score, float &conf, ShapeContext& match) {
  const float tolerance = 1.5;
  //score = mean.distance(sc);
  score = std::numeric_limits<float>::max(); //best score
  float currScore = std::numeric_limits<float>::max();
  float thresh = stddev*tolerance;
  for (list<ShapeContext>::iterator it = exemplars.begin(); it != exemplars.end(); it++) {
    currScore = it->distance(sc);
    if (currScore < score) {
      score = currScore;
      conf = (thresh - currScore) / thresh;
      match = (*it);
    }
  }
  if (score < thresh) {
    return true;
  } else {
    return false;
  }
}

void ShapeModel::print() {
  printf("mean:\n");
  mean.print();
  printf("stddev: %f\n", stddev);
}

void ShapeModel::printDetailed() {
  print();
  printf("\n");
  for (list<ShapeContext>::iterator it = exemplars.begin(); it != exemplars.end(); it++) {
    it->print();
    printf("\n");
  }
}

void ShapeModel::write(const string& filename) {
  FileStorage file(filename, FileStorage::WRITE);
  file << "mean_nbins_r" << mean.nbins_r;
  file << "mean_nbins_theta" << mean.nbins_theta;
  file << "mean_hist" << mean.hist;
  file << "stddev" << stddev;
  file << "exemplars" << "[";
  for (list<ShapeContext>::iterator it = exemplars.begin(); it != exemplars.end(); it++) {
    file << "{";
    file << "nbins_r" << it->nbins_r;
    file << "nbins_theta" << it->nbins_theta;
    file << "hist" << it->hist;
    file << "}";
  }
  file << "]";
  file.release();
}

void ShapeModel::read(const string& filename) {
  FileStorage file(filename, FileStorage::READ);
  file["mean_nbins_r"] >> mean.nbins_r;
  file["mean_nbins_theta"] >> mean.nbins_theta;
  file["mean_hist"] >> mean.hist;
  file["stddev"] >> stddev;
  FileNode ex = file["exemplars"];
  FileNodeIterator it = ex.begin(), it_end = ex.end();
  for (; it != it_end; ++it) {
    ShapeContext sc;
    (*it)["nbins_r"] >> sc.nbins_r;
    (*it)["nbins_theta"] >> sc.nbins_theta;
    (*it)["hist"] >> sc.hist;
    exemplars.push_back(sc);
  }
  file.release();
}
