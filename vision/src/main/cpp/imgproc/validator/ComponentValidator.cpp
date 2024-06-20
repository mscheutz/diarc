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


using namespace ade::stm;
namespace fs = std::experimental::filesystem;

float sigmoid(float x) {
    return 1.0 / (1.0 + std::exp(-x));
}


// Function to denormalize values
double denormalize(float norm_value, float min_value, float max_value) {
    return norm_value * (max_value - min_value) + min_value;
}


ComponentValidator::ComponentValidator(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight)
: ObjectValidator(processorId, imgWidth, imgHeight, false) {
  visionProcessName = "ComponentValidator";
  logger = log4cxx::Logger::getLogger("ade.imgproc.validator.ComponentValidator");
}


ComponentValidator::~ComponentValidator() {
}


void ComponentValidator::loadConfig(const std::string& config) {
  LOG4CXX_TRACE(logger, "[loadConfig] method entered.");

  //get directory
  unsigned found = config.find_last_of("/\\");
  std::string dir = config.substr(0, found + 1);
  std::string model_file;

  // populate tree structure pt
  using boost::property_tree::ptree;
  ptree pt;
  read_xml(config, pt);
  int predicateCounter = 0;

  // Set to track loaded models
  std::set<std::string> loadedModels;

  // parse xml file
  BOOST_FOREACH(ptree::value_type const& predicateNode, pt.get_child("processor")) {
    if (predicateNode.first.compare("predicate") == 0) {
      std::string descriptorName = predicateNode.second.get<std::string> ("<xmlattr>.name", "unknown");
      LOG4CXX_DEBUG(logger, boost::format("[loadConfig] descriptorName: %s.") % descriptorName);
      

      model_file = dir + predicateNode.second.get_child("model").get<std::string> ("<xmlattr>.filename", "unknown");
      
      if (loadedModels.find(model_file) == loadedModels.end()) {
                // Load the model if it hasn't been loaded yet
                ComponentValidator::initModel(model_file);
                loadedModels.insert(model_file); // Mark this model as loaded
                LOG4CXX_DEBUG(logger, boost::format("[loadConfig] Loaded model: %s.") % model_file);
                std::cout << "Model loaded from: " << model_file << std::endl;
            } else {
                LOG4CXX_DEBUG(logger, "[loadConfig] Model already loaded, skipping.");
                //std::cout << "Model loading skipped for: " << model_file << std::endl;
            }

      // -1 if we do not ask for the component's shape
      // shape value 0 -> cube, 1 -> sphere, 2 -> cone, 3 -> cylinder
      int shape = predicateNode.second.get("shape", -1);
      int axis = predicateNode.second.get("axis", -1);
      int index = predicateNode.second.get("index", -1);

      semanticProperties.push_back({descriptorName, shape, axis, index});
      predicateCounter = predicateCounter + 1;
    }
  }
}


/*
void ComponentValidator::loadConfig(const std::string &configFile) {
  //get directory
  unsigned found = configFile.find_last_of("/\\"); // check if config file is filename or path
  std::string dir = configFile.substr(0, found + 1);
  std::string model_file;
  std::string backEnd;
  std::string type;
 
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
    std::string name = predicateValue["name"].asString();
    int classID = predicateValue["classID"].asInt();
    classToId.insert({name, classID});
    idToClass.insert({classID, name});
  }

  Json::Value config = root["config"];
  model_file = dir + config["model"].asString();
  ComponentValidator::initModel(model_file);
}
*/


void ComponentValidator::initModel(const std::string &model_path) {
  fs::path onnx_model_abs = fs::canonical(model_path + ".onnx");
  net = cv::dnn::readNetFromONNX(onnx_model_abs.string());
}


void ComponentValidator::handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification) {
  LOG4CXX_TRACE(logger, "[ComponentValidation] method entered.");
  
  // get current descriptors to process
  TypesByDescriptorConstPtr descriptors = getDescriptors();
  TypesByDescriptor::const_iterator descriptor_iter;
  std::tr1::unordered_set<long long>::const_iterator typeIds_itr;
  
  MemoryObject::Ptr object = notification->object;
  
  // get captured frames info
  const cv::Mat currFrame = object->getCaptureData()->frame;

  // Pre-processing values for the input image
  double scalefactor = 1.0;
  cv::Scalar noMean(0.0, 0.0, 0.0);  // we do not want any mean subtraction
  int width = 256;
  int height = 192;
  cv::Size size(width, height);

  // convert image to grayscale and floats
  cv::Mat grayscaleFrame;
  cv::cvtColor(currFrame, grayscaleFrame, cv::COLOR_BGR2GRAY);
  
  // convert image to Semantic Encoder dimensions  
  cv::Mat blob;
  cv::dnn::blobFromImage(grayscaleFrame, blob, scalefactor, size, noMean, false); 
  // Extract Semantic Descriptions 
  net.setInput(blob);
  cv::Mat semMatrix = net.forward();
  // Apply sigmoid to the matrix
  cv::Mat semMatrixSig = applySigmoidToMatrix(semMatrix);

  // Check if the output matrix is valid before using it
  if (semMatrixSig.empty()) {
        std::cerr << "Failed to apply Sigmoid to matrix." << std::endl;
  }

  float confidence = 1.0f; //TODO: How could I set this in a good way?
  int predShape;
  int predAxis;
  int predIdx;
  bool predFound;

  PredicateHelper::Set::const_iterator descriptors_itr;
  DescriptorsByTypeConstPtr types = getTypes();
  DescriptorsByType::const_iterator types_itr = types->find(object->getTypeId());
  
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
          object->addValidationResult(confidence, PredicateHelper(sp->predicateKey + 
                                  "(" + descriptors_itr->getArg(0) + ")"));
        }
        std::cout << "Predicate " << sp->predicateKey + 
                                  "(" + descriptors_itr->getArg(0) + ")" << " found? " << predFound << std::endl;
      }
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
      std::cout << "Predicate " << it->predicateKey + 
                                  "(" + descriptors_itr->getArg(0) + ")" << " found? " << predFound << std::endl;
      
      if (!predFound) {
        LOG4CXX_DEBUG(logger, boost::format("Semantic Property not found"));
        continue;
      }

      //object->addValidationResult(confidence, *descriptors_itr);
      object->addValidationResult(confidence, PredicateHelper(*descriptors_itr));
      object->addValidationResult(confidence, PredicateHelper(it->predicateKey + 
                                  "(" + descriptors_itr->getArg(0) + ")"));
    } 
   
  }
  
  sendValidationNotifications(object);

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
      //std::cout << "Element " << i << " is " << matrixSig.at<float>(0, i) << std::endl;
    }
  } else {
    std::cerr << "Invalid matrix dimensions!" << std::endl;
    return cv::Mat(); // Return an empty matrix to indicate an error
  }

  return matrixSig; // Return the modified matrix

}