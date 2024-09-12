/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   OrientationValidator.hpp
 * Author: evan
 *
 * Created on March 27, 2014, 5:23 PM
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
  
protected:
  cv::dnn::Net net;

  // semantic parent type for this detector's descriptors (isA hierarchy)
  std::string parentType;
  std::unordered_map<std::string, int> classToId;
  std::unordered_map<int, std::string> idToClass;

  virtual void handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification);
  virtual void initModel(const std::string& model_path);

private:

  typedef struct {
    std::string predicateKey;
    int predicateShape;
    int predicateAxis;
    int predicateIndex;
  } predicateFeatures;

  const std::vector<std::vector<int>> shapeIdcs = {
    {3, 4, 5, 6},  // shape indices of base component 
    {11, 12, 13, 14},  // shape indices of side component 1
    {19, 20, 21, 22}  // shape indices of side component 2  
  };

  const std::vector<std::vector<int>> positionIdcs = {
    {0, 1, 2},  // position indices (x,y,z) of base component
    {8, 9, 10},  // position indices (delta x, delta y, delta z) of side component 1
    {16, 17, 18}  // position indices (delta x, delta y, delta z) of side component 2
  };

  std::vector<std::vector<int>> sizeIdcs = {
    {5, 6, 7},  // main axes (a_x, a_y, a_z) of base component
    {13, 14, 15},  // main axes (a_x, a_y, a_z) of side component 1
    {21, 22, 23}  // main axes (a_x, a_y, a_z) of side component 2
  };

  std::vector<std::vector<int>> sideCompIdcs = {
    { 8, 9, 10, 11, 12, 13, 14, 15},
    {16, 17, 18, 19, 20, 21, 22, 23}
  };

  const float minNormValue = 0;
  const float maxNormValue = 255;
  const int depthImageHeight = 192;
  const int depthImageWidth = 256;
  const int padding = 0;

  // Thresholds to "discretize" the continuous semantic parameters into
  // semantic predicates
  const float lowerEdginessThreshold = 0.2;
  const float upperEdginessThreshold = 0.8;
  const float similarityTolerance = 0.25;
  const float positionTolerance = 0.45;
  const float existenceThreshold = 0.8;
  const float volumeThreshold = 0.02;
  const double eccentricityTolerance = 0.4;
  const double widthCompressionFactor = 1.0;  
  const double heightCompressionFactor = 1.0; 

  std::vector<predicateFeatures> semanticProperties;
  
  void createNaNMasks(const cv::Mat& objectDepthImage, cv::Mat& nanMask, cv::Mat& nanBackgroundMask, 
    cv::Mat& nanInsideMask);
  void removeNansAndNoise(const cv::Mat& objectDepthImage, cv::Mat& objectDepthImageNorm);
  void centerObject(cv::Mat& objectDepthImageNorm, const cv::Mat& mask);
  void normalizeDepthImage(const cv::Mat& objectDepthImage, const cv::Mat& mask, cv::Mat& objectDepthImageNorm);
  bool calculateSemProperty(const cv::Mat& semMat, const int semPredShape, const int semPredAxis, 
    const int semPredIdx);
  bool isSideComponentPresent(const int compIndex, const cv::Mat& semMat);
  bool processShape(const int compIndex, const cv::Mat& semMat, const int semPredShape);
  bool isCylinder(const float n1, const float n2);
  bool isEllipsoid(const float n1, const float n2);
  bool isCuboid(const float n1, const float n2);
  bool processPosition(const int compIndex, const cv::Mat& semMat, const int semPredAxis);
  bool processSize(const int compIndex, const cv::Mat& semMat, const int semPredIdx); 
  bool processEccentricity(const cv::Mat& semMat, const int semPredIdx);                                  
  cv::Mat applySigmoidToMatrix(const cv::Mat& semMatrix);
  
};



#endif	/* COMPONENTVALIDATOR_HPP */
