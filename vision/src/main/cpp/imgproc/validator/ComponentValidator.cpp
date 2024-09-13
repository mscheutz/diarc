/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "ComponentValidator.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <jsoncpp/json/reader.h>
#include <boost/shared_ptr.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <experimental/filesystem>
#include <fstream>
#include <iostream>

#include <sstream>
#include <iomanip>
#include <ctime>
#include <thread>  
 
#include "capture/calibration/Cameras.hpp"
#include "display/Display.hpp"
#include "capture/util/CaptureUtilities.hpp"
#include "common/notification/CaptureNotification.hpp"
#include "stm/util/StmUtilities.hpp"
#include "point_clouds/PointCloudUtilities.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/common/common.h>


using namespace ade::stm;
namespace fs = std::experimental::filesystem;


float sigmoid(float x) {
    return 1.0 / (1.0 + std::exp(-x));
}


// Function to denormalize values
double denormalize(float norm_value, float min_value, float max_value) {
    return norm_value * (max_value - min_value) + min_value;
}


ComponentValidator::ComponentValidator(const long long& processorId, const unsigned int imgWidth, 
  const unsigned int imgHeight, const bool isStereo)
: ObjectValidator(processorId, imgWidth, imgHeight, isStereo) {
  visionProcessName = "ComponentValidator";
  logger = log4cxx::Logger::getLogger("ade.imgproc.validator.ComponentValidator");
}


ComponentValidator::~ComponentValidator() {
}


void ComponentValidator::loadConfig(const std::string &configFile) {
  LOG4CXX_TRACE(logger, "[loadConfig] method entered.");
  //get directory
  unsigned found = configFile.find_last_of("/\\"); // check if config file is filename or path
  std::string dir = configFile.substr(0, found + 1);
  std::string model_file;
  std::string type;
  std::string descriptorName;
  int shape;
  int axis;
  int index;

  Json::CharReaderBuilder builder;
  Json::Value root;
  JSONCPP_STRING errs;
  std::ifstream fileStream(configFile, std::ifstream::binary);
  if (!Json::parseFromStream(builder, fileStream, &root, &errs)) {
    LOG4CXX_ERROR(logger, boost::format("Error parsing json file: %s") % configFile.c_str());
    return;
  }

  // predicate descriptor options
  parentType = root["processor"]["type"].asString();
  int arity = root["processor"]["arity"].asInt();
  for (auto predicateValue: root["predicates"]) {

    std::string descriptorName = predicateValue["name"].asString();
    // Retrieve integer values if present or default to -1 if not present
    shape = predicateValue.isMember("shape") ? predicateValue["shape"].asInt() : -1;
    axis = predicateValue.isMember("axis") ? predicateValue["axis"].asInt() : -1;
    index = predicateValue.isMember("index") ? predicateValue["index"].asInt() : -1;
    semanticProperties.push_back({descriptorName, shape, axis, index});
    
  }

  Json::Value config = root["config"];
  model_file = dir + config["model"].asString();
  ComponentValidator::initModel(model_file);

  identifyShapes = config["identify_shapes"].asBool();
  minNormValue = config["min_norm_value"].asInt64();
  maxNormValue = config["max_norm_value"].asInt64();
  depthImageHeight = config["depth_image_height"].asInt64();
  depthImageWidth = config["depth_image_width"].asInt64();
  padding = config["padding"].asInt64();
  kernelSize = config["kernel_size"].asInt64();
  lowerEdginessThreshold = config["lower_edginess_threshold"].asDouble();
  upperEdginessThreshold = config["upper_edginess_threshold"].asDouble();
  similarityTolerance = config["similarity_tolerance"].asDouble();
  positionTolerance = config["position_tolerance"].asDouble();
  existenceThreshold = config["existence_threshold"].asDouble();
  volumeThreshold = config["volume_threshold"].asDouble();
  eccentricityTolerance = config["eccentricity_tolerance"].asDouble();
  widthCompressionFactor = config["width_compression_factor"].asDouble();
  heightCompressionFactor = config["height_compression_factor"].asDouble();

}


void ComponentValidator::initModel(const std::string &model_path) {
  net = cv::dnn::readNetFromONNX(model_path + ".onnx");
}


void ComponentValidator::handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification) {

  LOG4CXX_TRACE(logger, "[ComponentValidation] method entered.");

  MemoryObject::Ptr object = notification->object;
  
  // check if this typeId should currently be processed
  DescriptorsByTypeConstPtr types = getTypes();
  DescriptorsByType::const_iterator types_itr = types->find(object->getTypeId());

  if (types_itr != types->end()) {

    // get current descriptors to process
    TypesByDescriptorConstPtr descriptors = getDescriptors();
    TypesByDescriptor::const_iterator descriptor_iter;
    std::tr1::unordered_set<long long>::const_iterator typeIds_itr;

    // get MemoryObject cloud and remove NaNs
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr objectCloud = object->getDetectionMask()->getObjectPointCloud();
    
    // get depth image from object point cloud
    cv::Mat objectDepthImage(objectCloud->height, objectCloud->width, CV_32FC1);
    ade::pc::util::pointCloudToDepth(objectDepthImage, objectCloud);

    // Remove nans from background / object foreground 
    cv::Mat objectDepthImageNorm = objectDepthImage.clone();
    removeNansAndNoise(objectDepthImage, objectDepthImageNorm);
    double scalefactor = 1.0;
    cv::Scalar noMean(0.0, 0.0, 0.0);  // we do not want any mean subtraction
    cv::Size depthImageSize(depthImageWidth, depthImageHeight);
    cv::Mat blob;
    cv::dnn::blobFromImage(objectDepthImageNorm, blob, scalefactor, depthImageSize, noMean, true, CV_32F); 

    // Apply Semantic Encoder model to input depth image
    net.setInput(blob);
    cv::Mat semMatrix = net.forward();
    LOG4CXX_TRACE(logger, "[ComponentDetector] Model applied to input image.");

    // Apply sigmoid to the matrix
    cv::Mat semMatrixSig = applySigmoidToMatrix(semMatrix);

    // Check if the output matrix is valid before using it
    if (semMatrixSig.empty()) {
      LOG4CXX_DEBUG(logger, boost::format("Semantic Matrix not valid"));
    }

    float confidence = 0.99; //TODO: How could I set this in a good way?
    int predShape;
    int predAxis;
    int predIdx;
    bool predFound;
    std::vector<bool> foundPredicatesVector(semanticProperties.size(), false);

    bool sideComp1IsPresent = isSideComponentPresent(1, semMatrixSig);
    bool sideComp2IsPresent = isSideComponentPresent(2, semMatrixSig);

    // Determine if semantic predicates are present for object and save results in vector of booleans
    for (auto sp = semanticProperties.begin(); sp != semanticProperties.end() - 1; ++sp) {
      predShape = sp->predicateShape;
      predAxis = sp->predicateAxis;
      predIdx = sp->predicateIndex;

      foundPredicatesVector[predIdx] = calculateSemProperty(semMatrixSig, predShape, predAxis, predIdx);

      // If side component 1 is not found, we do not need to go further
      if (sideComp1IsPresent == false && predIdx >= 7){
        foundPredicatesVector[0] = true;
        break;
      }
      
      // If side component 2 is not found, we do not need to go further
      if (sideComp2IsPresent == false && predIdx >= 23) {
        foundPredicatesVector[1] = true;
        break;
      }

      if (identifyShapes) {

        if (predIdx == 5) {
          if (foundPredicatesVector[3] == false && foundPredicatesVector[4] == false)
          {
            foundPredicatesVector[5] = true;
          }
        }
        if (sideComp1IsPresent == true && predIdx == 9) {
          if (foundPredicatesVector[7] == false && foundPredicatesVector[8] == false)
          {
            foundPredicatesVector[9] = true;
          }
        }
        if (sideComp2IsPresent == true && predIdx == 25) {
          if (foundPredicatesVector[23] == false && foundPredicatesVector[24] == false)
          {
            foundPredicatesVector[25] = true;
          }
        }
      } else {

        std::vector<int> shapeIndices = {3, 4, 5, 7, 8, 9, 23, 24, 25};

        for (int idx : shapeIndices) {
            foundPredicatesVector[idx] = false;
        }

      }
      
    }

    if (sideComp1IsPresent == true && sideComp2IsPresent == true) {
      foundPredicatesVector[2] = true;
    }
    
    PredicateHelper::Set::const_iterator descriptors_itr;
    for (descriptors_itr = types_itr->second.begin(); descriptors_itr != types_itr->second.end(); 
      ++descriptors_itr) {

      std::string keyToFind = descriptors_itr->getName();

      if (keyToFind == "any" || keyToFind == "unknown") {
        // Access all elements except the last one
        for (auto sp = semanticProperties.begin(); sp != semanticProperties.end() - 1; ++sp) {
          
          // Set the corresponding index in foundPredicatesVector to predFound
          size_t index = std::distance(semanticProperties.begin(), sp);
          bool predFound = foundPredicatesVector[index];
          
          if (predFound) {
              
            object->addValidationResult(confidence, PredicateHelper(*descriptors_itr));
            object->addValidationResult(confidence, PredicateHelper(
                        sp->predicateKey + "(" + descriptors_itr->getArg(0) + ")"));
            
            // Logging the message with concatenated details
            LOG4CXX_TRACE(logger, "Detection notification sent. Predicate details: " 
              << sp->predicateKey << "(" 
              << descriptors_itr->getArg(0) << ")");

          }
        }
        
        sendValidationNotifications(object);
        
      } else {
      
        auto it = std::find_if(semanticProperties.begin(), semanticProperties.end(),
          [&keyToFind](const predicateFeatures& pf) {
            return pf.predicateKey == keyToFind;
        });

        // Check if the element was found
        if (it == semanticProperties.end()) {
          continue;
        } 

        predFound = foundPredicatesVector[it->predicateIndex];
        
        if (!predFound) {
          LOG4CXX_DEBUG(logger, boost::format("Semantic Property not found"));
          continue;

        } else {
          object->addValidationResult(confidence, *descriptors_itr);
          sendValidationNotifications(object);
        }
      }
    } 
  }
}


void ComponentValidator::createNaNMasks(const cv::Mat& objectDepthImage, cv::Mat& nanMask, cv::Mat& nanBackgroundMask, 
  cv::Mat& nanInsideMask) {

  // Initialize masks
  nanMask = cv::Mat(objectDepthImage.rows, objectDepthImage.cols, CV_8U);
  nanBackgroundMask = cv::Mat(objectDepthImage.rows, objectDepthImage.cols, CV_8U);
  nanInsideMask = cv::Mat(objectDepthImage.rows, objectDepthImage.cols, CV_8U);

  // Create mask where non-NaN values are segmented
  cv::compare(objectDepthImage, objectDepthImage, nanMask, cv::CMP_EQ);

  // Close nan values inside the object
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  cv::morphologyEx(nanMask, nanBackgroundMask, cv::MORPH_CLOSE, element);

  // Segment nan values inside the object
  cv::compare(nanMask, nanBackgroundMask, nanInsideMask, cv::CMP_NE);

}


void ComponentValidator::removeNansAndNoise(const cv::Mat& objectDepthImage, cv::Mat& objectDepthImageNorm) {

  // Variables to store the min and max values and their locations
  double minVal, maxVal;
  cv::Point minLoc, maxLoc;

  // masks to indicate nan positions for noise removal and background processing
  cv::Mat nanMask, nanBackgroundMask, nanInsideMask;
  createNaNMasks(objectDepthImage, nanMask, nanBackgroundMask, nanInsideMask);
  
  // Normalize depth values to specific range and set background to maxvalue + 1
  normalizeDepthImage(objectDepthImage, nanBackgroundMask, objectDepthImageNorm);

  // Impaint nan values inside object 
  int impaintingRadius = 30;
  cv::inpaint(objectDepthImageNorm, nanInsideMask, objectDepthImageNorm, impaintingRadius, cv::INPAINT_TELEA);
  centerObject(objectDepthImageNorm, nanBackgroundMask);

  objectDepthImageNorm.convertTo(objectDepthImageNorm, CV_8U);
  // Apply median blur to the entire image
  cv::medianBlur(objectDepthImageNorm, objectDepthImageNorm, kernelSize);

  // Calculate new dimensions
  int newWidth = static_cast<int>(objectDepthImageNorm.cols * widthCompressionFactor);
  int newHeight = static_cast<int>(objectDepthImageNorm.rows * heightCompressionFactor);
  objectDepthImageNorm.convertTo(objectDepthImageNorm, CV_32F);

  // If necessary, pad depth image with more background
  int top = padding;
  int bottom = top;
  int left = top*depthImageWidth/depthImageHeight;
  int right = top*depthImageWidth/depthImageHeight;
  int borderType = cv::BORDER_CONSTANT;
  cv::Scalar borderColor(maxNormValue, maxNormValue, maxNormValue);
  cv::copyMakeBorder(objectDepthImageNorm, objectDepthImageNorm, top, bottom, left, right, borderType, borderColor);

  // Resize the image to the dimension / depth range the model expects
  cv::resize(objectDepthImageNorm, objectDepthImageNorm, cv::Size(newWidth, newHeight));
  cv::normalize(objectDepthImageNorm, objectDepthImageNorm, 0.85, 1.0, cv::NORM_MINMAX);
}


void ComponentValidator::centerObject(cv::Mat& objectDepthImageNorm, const cv::Mat& mask) {

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  if (!contours.empty()) {
    cv::Moments M = cv::moments(contours[0]);
    cv::Point centroid(M.m10 / M.m00, M.m01 / M.m00);

    int imgWidth = objectDepthImageNorm.cols;
    int imgHeight = objectDepthImageNorm.rows;
    int offsetX = imgWidth / 2 - centroid.x;
    int offsetY = imgHeight / 2 - centroid.y;

    cv::Mat translationMatrix = (cv::Mat_<double>(2, 3) << 1, 0, offsetX, 0, 1, offsetY);

    cv::warpAffine(objectDepthImageNorm, objectDepthImageNorm, translationMatrix, objectDepthImageNorm.size(),
          cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(maxNormValue + 1));
  } else {
    LOG4CXX_DEBUG(logger, "[ComponentValidation] no contour found in depth image.");          
  }
}


bool ComponentValidator::isSideComponentPresent(const int compIndex, const cv::Mat& semMat) {

  float dx = semMat.at<float>(0, sideCompIdcs[compIndex - 1][0]); 
  float dy = semMat.at<float>(0, sideCompIdcs[compIndex - 1][1]); 
  float dz = semMat.at<float>(0, sideCompIdcs[compIndex - 1][2]); 
  float n1 = semMat.at<float>(0, sideCompIdcs[compIndex - 1][3]); 
  float n2 = semMat.at<float>(0, sideCompIdcs[compIndex - 1][4]); 
  float ax = semMat.at<float>(0, sideCompIdcs[compIndex - 1][5]); 
  float ay = semMat.at<float>(0, sideCompIdcs[compIndex - 1][6]); 
  float az = semMat.at<float>(0, sideCompIdcs[compIndex - 1][7]); 
  
  float sum = dx + dy + dz + n1 + n2 + ax + ay + az;

  if (sum > existenceThreshold)
  {
    return true;
  } else {
    return false;
  }
}


void ComponentValidator::normalizeDepthImage(const cv::Mat& objectDepthImage, const cv::Mat& mask, cv::Mat& objectDepthImageNorm) {
  cv::Mat inputROI;
  double minVal, maxVal;
  cv::minMaxLoc(objectDepthImage, &minVal, &maxVal, nullptr, nullptr, mask);

  double scale = (double)(maxNormValue - minNormValue) / (maxVal - minVal);
  double shift = -minVal * scale + minNormValue;

  objectDepthImage.convertTo(objectDepthImageNorm, CV_32FC1, scale, shift);
  objectDepthImageNorm.setTo(cv::Scalar(maxNormValue + 1), ~mask);
}


bool ComponentValidator::calculateSemProperty(const cv::Mat& semMat, const int semPredShape, const int semPredAxis, const int semPredIdx) {

  bool semMatch = false;
  double maxValue;
  int maxIndex;
  cv::Point maxLoc;
  int compIndex = -1;
  bool shapePredicate = false;
  bool positionPredicate = false;
  bool sizePredicate = false;
  bool eccentricityPredicate = false;
  
  switch (semPredIdx) {
    //we look for base component shape predicates
    case 0: case 1: case 2:
      break;
    //we look for base component shape predicates
    case 3: case 4: case 5:
      compIndex = 0;
      shapePredicate = true;
      break;
    //we look for side component 1 shape predicates
    case 7: case 8: case 9: 
      compIndex = 1;
      shapePredicate = true;
      break;
    //we look for side component 2 shape predicates
    case 23: case 24: case 25: 
      compIndex = 2;
      shapePredicate = true;
      break;
    //we look for side component 1 position predicates
    case 10: case 11: case 12: case 13: case 14: case 15: case 16: case 17: case 18:
      compIndex = 1;
      positionPredicate = true;
      break;
      //we look for side component 2 position predicates
    case 26: case 27: case 28: case 29: case 30: case 31: case 32: case 33: case 34:
      compIndex = 2;
      positionPredicate = true;
      break;
    //we look for side component 1 size predicates
    case 19: case 20: case 21:
      compIndex = 1;
      sizePredicate = true;
      break;
    //we look for side component 2 size predicates
    case 35: case 36: case 37:
      compIndex = 2;
      sizePredicate = true;
      break;
    case 6: case 22: case 38:
      eccentricityPredicate = true;
      break;
    default:
      LOG4CXX_DEBUG(logger, "[ComponentValidation] semPredIdx value not defined ");  
      return semMatch;
  }

  if (shapePredicate) {
    semMatch = processShape(compIndex, semMat, semPredShape);
  }

  if (positionPredicate) {
    semMatch = processPosition(compIndex, semMat, semPredAxis);
  }

  if (sizePredicate) {
    semMatch = processSize(compIndex, semMat, semPredIdx);
  }

  if (eccentricityPredicate) {
    semMatch = processEccentricity(semMat, semPredIdx);
  }

  return semMatch;

}


bool ComponentValidator::processShape(const int compIndex, const cv::Mat& semMat, const int semPredShape) {
  
  const auto& idcs = superEllipsoidParamsIdcs[compIndex];
  // Extract the specified semantic matrix column for the current component
  cv::Mat shapeParams = semMat.col(idcs[0]);
  for (size_t i = 1; i < idcs.size(); ++i) {
    cv::hconcat(shapeParams, semMat.col(idcs[i]), shapeParams);
  }

  const float n1 = shapeParams.at<float>(0, 0);
  const float n2 = shapeParams.at<float>(0, 1);
  const float ax = shapeParams.at<float>(0, 2);
  const float ay = shapeParams.at<float>(0, 3);

  // Determine if the shape matches the predicate we are looking for
  switch (semPredShape) {
    case 0: 
      return isCylinder(n1, n2);
    case 1: 
      return isCuboid(n1, n2);
    case 2: 
      return isEllipsoid(n1, n2);
    default:
      return false;
  }
}


bool ComponentValidator::isCylinder(const float n1, const float n2) {
    
    bool isN1Small = n1 < lowerEdginessThreshold;
    bool isN2Large = n2 > upperEdginessThreshold;
    
    //bool edginessDifferent = std::abs(n2 - n1) > similarityTolerance;
    bool edginessDifferent = (n2 - n1) > similarityTolerance;

    // All conditions must be true
    return edginessDifferent;
}


bool ComponentValidator::isEllipsoid(const float n1, const float n2) {
     
    bool isN1Large = n1 > upperEdginessThreshold;
    bool isN2Large = n2 > upperEdginessThreshold;
    
    // All conditions must be true
    return isN1Large && isN2Large;
}


bool ComponentValidator::isCuboid(const float n1, const float n2) {
    
    bool isN1Small = n1 < lowerEdginessThreshold;
    bool isN2Small = n2 < lowerEdginessThreshold;
    
    // All conditions must be true
    return isN1Small && isN2Small;
}


// Check discrete position 
bool ComponentValidator::processPosition(const int compIndex, const cv::Mat& semMat, const int semPredAxis) {

  // Initialize semMatch to false
  bool semPositionMatch = false;
  float distance;
  
  if (semPredAxis == 0 || semPredAxis == 1 || semPredAxis == 6) {  // x-axis distance
    distance = semMat.at<float>(0, positionIdcs[compIndex][0]);
  } else if (semPredAxis == 2 || semPredAxis == 3 || semPredAxis == 7) {  // y-axis distance
    distance = semMat.at<float>(0, positionIdcs[compIndex][1]);
  } else if (semPredAxis == 4 || semPredAxis == 5 || semPredAxis == 8) {  // z-axis distance
    distance = semMat.at<float>(0, positionIdcs[compIndex][2]);   
  } else {
    std::cerr << "Axis " << semPredAxis << " is not part of the defined groups." << std::endl;
  }

  if ((semPredAxis == 0 || semPredAxis == 2 || semPredAxis == 4) && (distance > (1 - positionTolerance))) {
    // in front (x-axis), to the right (y-axis), on top (z-axis)
    semPositionMatch = true;  
   
  } else if ((semPredAxis == 1 || semPredAxis == 3 || semPredAxis == 5) && (distance < (0 + positionTolerance))) {
    // behind (x-axis), to the left (y-axis), at bottom (z-axis), 
    semPositionMatch = true;
    
  } else if ((semPredAxis == 6 || semPredAxis == 7 || semPredAxis == 8) && (distance > (0 + positionTolerance)) && (distance < (1 - positionTolerance))) {
    // centered along depth axis (x-axis), horizontally (y-axis), vertically (z-axis), 
    semPositionMatch = true;
    
  } 

  // Return the result
  return semPositionMatch;

}


bool ComponentValidator::processSize(const int compIndex, const cv::Mat& semMat, const int semPredIdx) {

  // Initialize semMatch to false
  bool semSizeMatch = false;

  float axb = semMat.at<float>(0, sizeIdcs[0][0]); 
  float ayb = semMat.at<float>(0, sizeIdcs[0][1]); 
  float azb = semMat.at<float>(0, sizeIdcs[0][2]); 
  float axs = semMat.at<float>(0, sizeIdcs[compIndex][0]); 
  float ays = semMat.at<float>(0, sizeIdcs[compIndex][1]); 
  float azs = semMat.at<float>(0, sizeIdcs[compIndex][2]); 
  float volumeBase = ayb * azb;
  float volumeSide = ays * azs;

  float volumeDifference = volumeSide - volumeBase;

  if ((semPredIdx == 19 || semPredIdx == 35) && (volumeDifference > (0 + volumeThreshold))) {
    semSizeMatch = true;  // component is larger
  } else if ((semPredIdx == 20 || semPredIdx == 36) && (volumeDifference < (0 - volumeThreshold))) {
    semSizeMatch = true;  // component is smaller
  } else if ((semPredIdx == 21 || semPredIdx == 37) && (volumeDifference > (0 - volumeThreshold)) && 
    (volumeDifference < (0 + volumeThreshold))) {
    semSizeMatch = true;  // component is of similar size
  }
  
  // Return the result
  return semSizeMatch;

}


bool ComponentValidator::processEccentricity(const cv::Mat& semMat, const int semPredIdx) {

  int compIndex;

  switch (semPredIdx) {
    //we look for base component eccentricity 
    case 6: 
      compIndex = 0;
      break;
    //we look for side component 1 eccentricity 
    case 22: 
      compIndex = 1;
      break;
    //we look for side component 2 eccentricity 
    case 38: 
      compIndex = 2;
      break;
    default:
      std::cerr << "semPredIdx is not supposed to appear at the Eccentricity check" << semPredIdx << std::endl;
      return false;
  }

  float ay = semMat.at<float>(0, sizeIdcs[compIndex][1]); 
  float az = semMat.at<float>(0, sizeIdcs[compIndex][2]); 

  // If main axes differ "enough", we consider the component as elongated
  bool semEccentricityMatch = std::abs(ay - az) > eccentricityTolerance;
  
  // Return the result
  return semEccentricityMatch;

}


// Function to apply sigmoid to matrix and check dimensions
cv::Mat ComponentValidator::applySigmoidToMatrix(const cv::Mat& matrix) {
  // Initialize the output matrix
  cv::Mat matrixSig = matrix.clone();

  // Check if the matrix is not empty and has the correct dimensions
  if (!matrix.empty() && matrix.cols == 24 && matrix.rows == 1) {
    // Apply the sigmoid function to each element in the matrix
    for (int i = 0; i < matrix.cols; ++i) {
      matrixSig.at<float>(0, i) = sigmoid(matrix.at<float>(0, i));
    }
   } else {
    LOG4CXX_DEBUG(logger, "[ComponentValidation] Invalid semantic matrix dimensions ");  
    return cv::Mat(); // Return an empty matrix to indicate an error
  }

  return matrixSig; // Return the modified matrix

}
