/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "ComponentValidator.hpp"

#include <opencv2/opencv.hpp>
#include <jsoncpp/json/reader.h>
#include <boost/shared_ptr.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <experimental/filesystem>
#include <fstream>
#include <iostream>

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
}


void ComponentValidator::initModel(const std::string &model_path) {
  net = cv::dnn::readNetFromONNX(model_path + ".onnx");
}


void ComponentValidator::handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification) {

  LOG4CXX_TRACE(logger, "[ComponentValidation] method entered.");

  MemoryObject::Ptr object = notification->object;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr sceneCloud = object->getDetectionMask()->getCaptureData()->cloud;

  // check if this typeId should currently be processed
  DescriptorsByTypeConstPtr types = getTypes();
  DescriptorsByType::const_iterator types_itr = types->find(object->getTypeId());
  if (types_itr != types->end()) {
  
    // get current descriptors to process
    TypesByDescriptorConstPtr descriptors = getDescriptors();
    TypesByDescriptor::const_iterator descriptor_iter;
    std::tr1::unordered_set<long long>::const_iterator typeIds_itr;

    // get MemoryObject cloud and remove NaNs
    std::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> objectCloud = object->getDetectionMask()->getObjectPointCloud();

    cv::Mat objectDepthImage(objectCloud->height, objectCloud->width, CV_32FC1);
    ade::pc::util::pointCloudToDepth(objectDepthImage, objectCloud);

    cv::Mat mask = cv::Mat(objectDepthImage.rows, objectDepthImage.cols, CV_8U);
    // Create mask where NaN values are set to 0, others to 1
    cv::compare(objectDepthImage, objectDepthImage, mask, cv::CMP_EQ);
    
    // Normalize depth values to the range [0, 255]
    cv::Mat objectDepthImageNorm = objectDepthImage.clone();
    normalizeDepthImage(objectDepthImage, mask, objectDepthImageNorm);

    centerObject(objectDepthImageNorm, mask);

    // Pre-processing values for the input image
    double scalefactor = 1.0;
    cv::Scalar noMean(0.0, 0.0, 0.0);  // we do not want any mean subtraction
    cv::Size depthImageSize(depthImageWidth, depthImageHeight);
    
    // convert image to Semantic Encoder dimensions and format 
    cv::Mat blob;
    cv::dnn::blobFromImage(objectDepthImageNorm, blob, scalefactor, depthImageSize, noMean, true, CV_32F); 
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

    PredicateHelper::Set::const_iterator descriptors_itr;
    for (descriptors_itr = types_itr->second.begin(); descriptors_itr != types_itr->second.end(); ++descriptors_itr) {

      std::string keyToFind = descriptors_itr->getName();
      
      if (keyToFind == "any")
      {
        
        // Access all elements except the last one
        for (auto sp = semanticProperties.begin(); sp != semanticProperties.end() - 1; ++sp) {
          predShape = sp->predicateShape;
          predAxis = sp->predicateAxis;
          predIdx = sp->predicateIndex;
          predFound = calculateSemProperty(semMatrixSig, predShape, predAxis, predIdx);
          
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

        predShape = it->predicateShape;
        predAxis = it->predicateAxis;
        predIdx = it->predicateIndex;

        predFound = calculateSemProperty(semMatrixSig, predShape, predAxis, predIdx);
        
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

  switch (semPredIdx) {
    //we look for base component shape predicates
    case 0: case 1: case 2: case 3:
      compIndex = 0;
      shapePredicate = true;
      break;
    //we look for side component 1 shape predicates
    case 4: case 5: case 6: case 7:
      compIndex = 1;
      shapePredicate = true;
      break;
    //we look for side component 2 shape predicates
    case 16: case 17: case 18: case 19:
      compIndex = 2;
      shapePredicate = true;
      break;
    //we look for side component 1 position predicates
    case 8: case 9: case 10: case 11: case 12: case 13:
      compIndex = 1;
      positionPredicate = true;
      break;
      //we look for side component 2 position predicates
    case 20: case 21: case 22: case 23: case 24: case 25:
      compIndex = 2;
      positionPredicate = true;
      break;
    //we look for side component 1 size predicates
    case 14: case 15:
      compIndex = 1;
      sizePredicate = true;
      break;
    //we look for side component 2 size predicates
    case 26: case 27:
      compIndex = 2;
      sizePredicate = true;
      break;
    default:
      std::cerr << "semPredIdx not defined " << semPredIdx << std::endl;
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

  return semMatch;

}


bool ComponentValidator::processShape(const int compIndex, const cv::Mat& semMat, const int semPredShape) {
  
  ComponentValidator::shapeIdcs = {
    {0, 1, 2, 3},  //shape indices of base component 
    {8, 9, 10, 11},  //shape indices of side component 1
    {16, 17, 18, 19}  //shape indices of side component 2
  };
  
  // Initialize semMatch to false
  bool semShapeMatch = false;
  double maxValue;
  cv::Point maxLoc;

  const auto& idcs = shapeIdcs[compIndex];
  // Extract the specified semantic matrix column for the current component
  cv::Mat shapeParams = semMat.col(idcs[0]);
  for (size_t i = 1; i < idcs.size(); ++i) {
    cv::hconcat(shapeParams, semMat.col(idcs[i]), shapeParams);
  }

  // Find the maximum value among the values of side component 2
  cv::minMaxLoc(shapeParams, nullptr, &maxValue, nullptr, &maxLoc);
  int maxIndex = maxLoc.x;

  // Determine shape only if side component is present
  if (compIndex != 0 && maxValue > 0.5) {
    int chosenIndex = maxLoc.x;
    // If the shape is the same as the shape we are looking for
    if (maxIndex == semPredShape) {
      semShapeMatch = true;
    }
  } else if (compIndex == 0) {
    // The base component will always exist
    if (maxIndex == semPredShape) {
      semShapeMatch = true;
    }
  }

  // Return the result
  return semShapeMatch;

}


bool ComponentValidator::processPosition(const int compIndex, const cv::Mat& semMat, const int semPredAxis) {

  ComponentValidator::positionIdcs = {
    {4, 5, 6},  // spatial indices (x,y,z) of base component
    {12, 13, 14},  // spatial indices (x,y,z) of side component 1
    {20, 21, 22}  // spatial indices (x,y,z) of side component 2
  };

  float maxLength = 10;
  float maxShift = 3.5;
  
  // Initialize semMatch to false
  bool semPositionMatch = false;

  float baseLocation; 
  float sideLocation;
  float distance;
  
  if (semPredAxis == 0 || semPredAxis == 1 ) {  // x-axis distance
    baseLocation = semMat.at<float>(0, positionIdcs[0][0]);
    sideLocation = semMat.at<float>(0, positionIdcs[compIndex][0]);
  } else if (semPredAxis == 2 || semPredAxis == 3) {  // y-axis distance
    baseLocation = semMat.at<float>(0, positionIdcs[0][1]);  
    sideLocation = semMat.at<float>(0, positionIdcs[compIndex][1]);
  } else if (semPredAxis == 4 || semPredAxis == 5) {  // z-axis distance
    baseLocation = semMat.at<float>(0, positionIdcs[0][2]); 
    sideLocation = semMat.at<float>(0, positionIdcs[compIndex][2]);   
  } else {
    std::cerr << "Axis " << semPredAxis << " is not part of the defined groups." << std::endl;
  }

  float baseLocationDeNorm = denormalize(baseLocation, -maxShift, maxShift);
  float sideLocationDeNorm = denormalize(sideLocation, -maxShift - maxLength, maxShift + maxLength);

  distance =  sideLocationDeNorm - baseLocationDeNorm;

  if ((semPredAxis == 0 || semPredAxis == 2 || semPredAxis == 4) && distance > 0) {
    // to the right (x-axis), on top (y-axis), in front (z-axis)
    semPositionMatch = true;  
   
  } else if ((semPredAxis == 1 || semPredAxis == 3 || semPredAxis == 5) && distance < 0) {
    // to the left (x-axis), at bottom (y-axis), behind (z-axis)
    semPositionMatch = true;

  } 

  // Return the result
  return semPositionMatch;

}


bool ComponentValidator::processSize(const int compIndex, const cv::Mat& semMat, const int semPredIdx) {

  ComponentValidator::sizeIdcs = {
    {7},  // size indices of base component
    {15},  // size indices of side component 1
    {23}  // size indices of side component 2
  };
  
  // Initialize semMatch to false
  bool semSizeMatch = false;

  float baseSize = semMat.at<float>(0, sizeIdcs[0][0]); 
  float sideSize = semMat.at<float>(0, sizeIdcs[compIndex][0]); 
  float difference = sideSize - baseSize;

  if ((semPredIdx == 14 || semPredIdx == 26) && difference >= 0) {
    semSizeMatch = true;
  }
  
  if ((semPredIdx == 15 || semPredIdx == 27) && difference < 0) {
    semSizeMatch = true;
  }
  
  // Return the result
  return semSizeMatch;

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
    std::cerr << "Invalid matrix dimensions!" << std::endl;
    return cv::Mat(); // Return an empty matrix to indicate an error
  }

  
  return matrixSig; // Return the modified matrix

}