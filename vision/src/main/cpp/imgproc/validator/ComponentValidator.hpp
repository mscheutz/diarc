/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef COMPONENTVALIDATOR_HPP
#define	COMPONENTVALIDATOR_HPP

#include "ObjectValidator.hpp"

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
  std::unordered_map<std::string, int> classToId;
  std::unordered_map<int, std::string> idToClass;

  virtual void handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification);
  virtual void initModels(const std::string &model_path1, const std::string &model_path2, const std::string &model_path3);

private:

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
    {23, 24, 25}  // main axes (a_x, a_y, a_z) of side component 2
  };

  std::vector<std::vector<int>> sideCompIdcs = {
    {10, 11, 12, 13, 14, 15, 16, 17},
    {18, 19, 20, 21, 22, 23, 24, 25}
  };

  std::vector<std::vector<int>> rotationIdcs = {
    {8, 9},  // azimuth and elevation of base component
    {26, 27},  // azimuth and elevation of side component 1
    {28, 29}  // azimuth and elevation of side component 2
  };

  int numRandomAngles = 9;  // Number of additional angles (not counting 0)

  // Constants for de-normalizing output values back to 3D space
  const float dfac = 1.2f;
  const float EDGINESS_MIN = 0.01f;
  const float EDGINESS_MAX = 1.0f;
  const float AXIS_UNIT_CUBE_MAX = 10;
  const float ANGLE_MIN = 0.0f;
  const float ANGLE_MAX = 89.0f;

  // Voxel size for IoU computation
  const float voxel_size = 0.35f;

  // Constants for depth image preprocessing
  const int minNormValue = 0;
  const int maxNormValue = 255;
  const int depthImageHeight = 250;
  const int depthImageWidth = 250;

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

  std::vector<PredicateFeatures> semanticProperties;
  
  void denoiseAndUpsamplePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr& outputCloud);
  BestDescription findBestReconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr& objectCloud,
                                                          cv::Mat& objectDepthImage);
  std::pair<std::vector<float>, std::vector<float>> applyCNNandPostProcess(const cv::Mat& inputImage, float angle);
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
  void translatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::Vector3f& translation);   
  void translatePointCloudToOrigin(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  void rotatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float angle_degrees, const Eigen::Vector3f& axis);  
  pcl::PointCloud<pcl::PointXYZ>::Ptr combinePointClouds(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds);    
  Eigen::Vector4f computeBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);   
  Eigen::Vector4f computeCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);    
  Eigen::Vector4f computeBoundingBoxCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  void scalePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float uniform_scale_factor);
  void scalePointCloudToUnitCube(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float cubeHalfSize);
  void projectToYZPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected);  
  float computeIou(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);
  void voxelizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);  
  float computeChamferDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2);
  void saveCombinedSuperellipsoid(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds,
                                    const std::string& output_filename);  
  void applyRotationToImage(const cv::Mat &objectDepthImageNorm, cv::Mat &objectDepthImageNormRot,
                               const float angle);
  void createNaNMasks(const cv::Mat& objectDepthImage, cv::Mat& nanMask, cv::Mat& nanBackgroundMask, 
                      cv::Mat& nanInsideMask);
  void removeNansAndNoise(const cv::Mat &objectDepthImage, cv::Mat &objectDepthImageNorm);
  void centerObject(cv::Mat& objectDepthImageNorm, const cv::Mat& mask);
  void cutOutObject(cv::Mat &objectDepthImageNorm, const cv::Mat &mask);
  void normalizeDepthImage(const cv::Mat& objectDepthImage, const cv::Mat& mask, cv::Mat& objectDepthImageNorm);
  bool calculateSemProperty(const cv::Mat& semMat, const int semPredShape, const int semPredAxis, 
                            const int semPredIdx);
  bool isSideComponentPresent(const int compIndex, const cv::Mat& semMat);
  bool processShape(const int compIndex, const cv::Mat& semMat, const int semPredShape);
  bool isRod(const float n1, const float n2, const float ax, const float ay, const float az);
  bool isDisk(const float n1, const float n, const float ax, const float ay, const float az);
  bool isPart(const float n1, const float n2, const float ax, const float ay, const float az);
  bool processPosition(const int compIndex, const cv::Mat& semMat, const int semPredAxis);
  bool processSize(const int compIndex, const cv::Mat& semMat, const int semPredIdx); 
  bool processEccentricity(const cv::Mat& semMat, const int semPredIdx);
  bool processOrientation(const int compIndex, const cv::Mat &semMat, const int semPredIdx);
  cv::Mat applySigmoidToMatrix(const cv::Mat& semMatrix);
  std::string generateFullDescription(std::vector<bool> foundPredicatesVector);
  void populateMemoryObjectDescription(std::vector<bool> foundPredicatesVector, diarc::stm::MemoryObject::Ptr mo);
  
};



#endif	/* COMPONENTVALIDATOR_HPP */