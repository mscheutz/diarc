/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef COMPONENTVALIDATOR_HPP
#define	COMPONENTVALIDATOR_HPP

#include "ObjectValidator.hpp"
#include <pybind11/pytypes.h>

class ComponentValidator : public ObjectValidator {
  
public:
  typedef boost::shared_ptr<ComponentValidator> Ptr;
  typedef boost::shared_ptr<const ComponentValidator> ConstPtr;

  ComponentValidator(const long long& processorId, const unsigned int imgWidth, 
    const unsigned int imgHeight, const bool isStereo);
  virtual ~ComponentValidator();
  virtual void loadConfig(const std::string& configFile);
  virtual bool handleLearningNotification(const std::string& notification);

protected:

  std::vector<cv::dnn::Net> nets;
  
  // semantic parent type for this detector's descriptors (isA hierarchy)
  std::string parentType;
  // Name for object structure user gives as input
  std::string objectType;  
  
  virtual void handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification);
  virtual void handleFrameCompletionNotification(FrameCompletionNotification::ConstPtr notification);
  virtual void initModels(const std::string &model_path);

private:

  std::string newestModelPathFile = "../vision/src/main/resources/config/edu/tufts/hrilab/vision/imgProcs/componentvalidator/model_path.txt";

  typedef struct {
    std::string predicateKey;
    int predicateShape;
    int predicateAxis;
    int predicateIndex;
  } PredicateFeatures;

  typedef struct {
    float minChamferDist;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> bestSuperellipsoidsUnRot;
    pcl::PointCloud<pcl::PointXYZ>::Ptr objectUnRot;
    cv::Mat bestSemMatrixActUnRot;
    int modelIdx;
  } BestDescription;

  const std::vector<std::vector<int>> superEllipsoidParamsIdcs = {
    {3, 4, 5, 6, 7},  // shape indices of base component 
    {14, 15, 16, 17, 18},  // shape indices of side component 1
    {25, 26, 27, 28, 29}  // shape indices of side component 2
  };

  const std::vector<std::vector<int>> positionIdcs = {
    {0, 1, 2},  // position indices (x,y,z) of base component
    {11, 12, 13},  // position indices (delta x, delta y, delta z) of side component 1
    {22, 23, 24}  // position indices (delta x, delta y, delta z) of side component 2
  };

  std::vector<std::vector<int>> sizeIdcs = {
    {5, 6, 7},  // main axes (a_x, a_y, a_z) of base component
    {16, 17, 18},  // main axes (a_x, a_y, a_z) of side component 1
    {27, 28, 29}  // main axes (a_x, a_y, a_z) of side component 2
  };

  std::vector<std::vector<int>> sideCompIdcs = {
    {11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21},  // all position and shape indices of side component 1
    {22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32}  // all position and shape indices of side component 2
  };

  std::vector<std::vector<int>> rotationIdcs = {
    {8, 9, 10},  // azimuth and elevation of base component
    {19, 20, 21},  // azimuth and elevation of side component 1
    {30, 31, 32}  // azimuth and elevation of side component 2
  };

  std::vector<std::vector<int>> rotationIdcsSinCos = {
    {8, 9, 10, 11, 12, 13},  // sine & cosine of azimuth, sine & cosine of elevation, sine & cosine of roll of base component
    {22, 23, 24, 25, 26, 27},  // sine & cosine of azimuth, sine & cosine of elevation, sine & cosine of roll of side component 1
    {36, 37, 38, 39, 40, 41}  // sine & cosine of azimuth, sine & cosine of elevation, sine & cosine of roll of side component 2
  };

  // for filtering "known" objects before processing
  unsigned long long mostRecentKnownFrameNum = 0;
  unsigned long long mostRecentUnknownFrameNum = 0;
  std::unordered_map<unsigned long long, std::vector<diarc::stm::MemoryObject::Ptr>> knownMOs;
  std::unordered_map<unsigned long long, std::vector<diarc::stm::MemoryObject::Ptr>> unknownMOs;

  // Constants for de-normalizing output values back to 3D space
  const float EDGINESS_MIN = 0.01f;
  const float EDGINESS_MAX = 1.0f;
  const float AXIS_UNIT_CUBE_MAX = 10;
  const float ANGLE_MIN = 0.0f;
  const float ANGLE_MAX = 179.0f;

  const int depthImageHeight = 250;
  const int depthImageWidth = 250;
  bool userDefinedModelUsage = true; // does the user defined a visual search model?
  // Adjustable parameters for depth image preprocessing
  bool identifyShapes = false;
  bool visualizeReconstruction = false;
  int paddingTopBottom = 65;  // Average of what training depthimages have
  int paddingLeftRight = 65;  // Average of what training depthimages have

  int kernelSize = 19;

  // Thresholds to "discretize" the continuous semantic parameters into
  // semantic predicates
  double edginessSimilarityThreshold = 0.25;
  double shapeThreshold = 0.35;
  double positionToleranceDepth = 0.45;
  double positionToleranceHorizontal = 0.45;
  double positionToleranceVertical = 0.45;
  double existenceThreshold = 0.8;
  double volumeThreshold = 0.02;
  double eccentricityTolerance = 0.4;
  double chamferTolerance = 2;

  std::vector<PredicateFeatures> semanticProperties;

  void processUnknownObject(diarc::stm::MemoryObject::Ptr object);
  void removeKnownObjects(const std::vector<diarc::stm::MemoryObject::Ptr>& knownObjects, std::vector<diarc::stm::MemoryObject::Ptr>& unknownObjects);
  float calculateBBoxIoU(const cv::Rect& rect1, const cv::Rect& rect2);

  void denoiseAndUpsamplePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr& outputCloud);
  BestDescription findBestReconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr& objectCloud,
                                                          cv::Mat& objectDepthImage);
  void parseComponentShapes(const std::string& notification,
                       int& component_count,
                       std::unordered_map<int, std::string>& shapes,
                       std::string& type_name);                                                        
  std::vector<float> denormalizeSemanticParams(const cv::Mat& semMatrixActValuesMat);
  std::vector<float> unrotateSemanticParams(const cv::Mat& semMatrixAct, float rotationAngle);
  std::vector<float> orderComponentsByDistanceToCenter(const std::vector<float>& semParams);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> createSuperellipsoidObject(const std::vector<float>& semMatrixActValues);
  pcl::PointCloud<pcl::PointXYZ>::Ptr createSuperellipsoid(float a_x, float a_y, float a_z,
                                                           float n_1, float n_2,
                                                           float azimuth, float elevation,
                                                           float roll,
                                                           int resolution,
                                                           const Eigen::Vector3f& translation);                                                      
  void sampleSuperellipsoidMontecarlo(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                                      float a_x, float a_y, float a_z, 
                                      float n_1, float n_2, int num_samples);
  void sampleSuperellipsoidInverseMapping(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                          float a_x, float a_y, float a_z,
                                          float n_1, float n_2, int num_samples);
  void translatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                           const Eigen::Vector3f& translation);   
  void translatePointCloudToOrigin(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  void rotatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                        float angle_degrees, 
                        const Eigen::Vector3f& axis);  
  pcl::PointCloud<pcl::PointXYZ>::Ptr combinePointClouds(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds);    
  Eigen::Vector4f computeBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);   
  Eigen::Vector4f computeCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);    
  Eigen::Vector4f computeBoundingBoxCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  void scalePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float uniform_scale_factor);
  float scalePointCloudToUnitCube(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float cubeHalfSize);
  void printDimensions(BestDescription& bestDescriptionCandidate, float refCubeScaleFactor);
  void projectToYZPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected);   
  float computeChamferDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1, 
                               const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2);
  void saveCombinedSuperellipsoid(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds,
                                    const std::string& output_filename);  
  void applyRotationToImage(const cv::Mat &objectDepthImageNorm, cv::Mat &objectDepthImageNormRot,
                               const float angle);
  void createNaNMasks(const cv::Mat& objectDepthImage, cv::Mat& nanMask, cv::Mat& nanBackgroundMask, 
                      cv::Mat& nanInsideMask);
  void removeNansAndNoise(const cv::Mat &objectDepthImage, cv::Mat &objectDepthImageNorm);
  void cutOutObject(cv::Mat &objectDepthImageNorm, const cv::Mat &mask);
  bool calculateSemProperty(const cv::Mat& semMat, const int semPredShape, const int semPredAxis, 
                            const int semPredIdx);
  bool isSideComponentPresent(const int compIndex, const cv::Mat& semMat);
  bool processShape(const int compIndex, const cv::Mat& semMat, const int semPredShape);
  bool isRod(const float n1, const float n2, const float ax, const float ay, const float az);
  bool isDisk(const float n1, const float n, const float ax, const float ay, const float az);
  bool isBlock(const float n1, const float n2);
  bool isPart(const float n1, const float n2, const float ax, const float ay, const float az);
  bool processPosition(const int compIndex, const cv::Mat& semMat, const int semPredAxis);
  bool processSize(const int compIndex, const cv::Mat& semMat, const int semPredIdx); 
  bool processEccentricity(const cv::Mat& semMat, const int semPredIdx);
  bool processOrientation(const int compIndex, const cv::Mat &semMat, const int semPredIdx);
  cv::Mat applyMixedActivation(const cv::Mat& semMatrix);
  std::string generateFullDescription(std::vector<bool> foundPredicatesVector, float minChamfer);
  void populateMemoryObjectDescription(std::vector<bool> foundPredicatesVector, diarc::stm::MemoryObject::Ptr mo);
  bool isTargetColor(const cv::Mat& image, const std::vector<int>& indices, const cv::Scalar& color, float &confidence, cv::Mat_<float>& imageMask);
};



#endif	/* COMPONENTVALIDATOR_HPP */
