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
    cv::Mat bestSemMatrixSigUnRot;
    int modelIdx;
  } BestDescription;

  const std::vector<std::vector<int>> superEllipsoidParamsIdcs = {
    {3, 4, 5, 6, 7},  // shape indices of base component 
    {13, 14, 15, 16, 17},  // shape indices of side component 1
    {21, 22, 23, 24, 25}  // shape indices of side component 2
  };

  const std::vector<std::vector<int>> positionIdcs = {
    {0, 1, 2},  // position indices (x,y,z) of base component
    {10, 11, 12},  // position indices (delta x, delta y, delta z) of side component 1
    {18, 19, 20}  // position indices (delta x, delta y, delta z) of side component 2
  };

  std::vector<std::vector<int>> sizeIdcs = {
    {5, 6, 7},  // main axes (a_x, a_y, a_z) of base component
    {15, 16, 17},  // main axes (a_x, a_y, a_z) of side component 1
    {25, 26, 27}  // main axes (a_x, a_y, a_z) of side component 2
  };

  std::vector<std::vector<int>> sideCompIdcs = {
    {10, 11, 12, 13, 14, 15, 16, 17},
    {18, 19, 20, 21, 22, 23, 24, 25}
  };

  std::vector<std::vector<int>> rotationIdcs = {
    {8, 9},  // azimuth and elevation of base component
    {18, 19},  // azimuth and elevation of side component 1
    {28, 29}  // azimuth and elevation of side component 2
  };

  // Constants for de-normalizing output values back to 3D space
  const float EDGINESS_MIN = 0.01f;
  const float EDGINESS_MAX = 1.0f;
  const float AXIS_UNIT_CUBE_MAX = 10;
  const float ANGLE_MIN = 0.0f;
  const float ANGLE_MAX = 89.0f;

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
  
  void denoiseAndUpsamplePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr& outputCloud);
  BestDescription findBestReconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr& objectCloud,
                                                          cv::Mat& objectDepthImage);
  void parseComponentShapes(const std::string& notification,
                       int& component_count,
                       std::unordered_map<int, std::string>& shapes,
                       std::string& type_name);                                                        
  std::vector<float> denormalizeSemanticParams(const cv::Mat& semMatrixSigValuesMat);
  std::vector<float> unrotateSemanticParams(const cv::Mat& semMatrixSig, float rotationAngle);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> createSuperellipsoidObject(const std::vector<float>& semMatrixSigValues);
  pcl::PointCloud<pcl::PointXYZ>::Ptr createSuperellipsoid(float a_x, float a_y, float a_z,
                                                           float n_1, float n_2,
                                                           float azimuth, float elevation,
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
  void printRescaledAxes(BestDescription& bestDescriptionCandidate, float refCubeScaleFactor);
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
  cv::Mat applySigmoidToMatrix(const cv::Mat& semMatrix);
  std::string generateFullDescription(std::vector<bool> foundPredicatesVector, float minChamfer);
  void populateMemoryObjectDescription(std::vector<bool> foundPredicatesVector, diarc::stm::MemoryObject::Ptr mo);
  
};



#endif	/* COMPONENTVALIDATOR_HPP */