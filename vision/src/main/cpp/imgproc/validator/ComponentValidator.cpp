
/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "ComponentValidator.hpp"

#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>

#include "capture/calibration/Cameras.hpp"
#include "display/Display.hpp"

#include <jsoncpp/json/reader.h>

#include "capture/util/CaptureUtilities.hpp"
#include "common/notification/CaptureNotification.hpp"
#include "stm/util/StmUtilities.hpp"
#include <experimental/filesystem>
#include <fstream>
#include <iostream>

using namespace ade::stm;
namespace fs = std::experimental::filesystem;

float sigmoid(float x) {
    return 1.0 / (1.0 + std::exp(-x));
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

  // parse xml file
  BOOST_FOREACH(ptree::value_type const& predicateNode, pt.get_child("processor")) {
    if (predicateNode.first.compare("predicate") == 0) {
      std::string descriptorName = predicateNode.second.get<std::string> ("<xmlattr>.name", "unknown");
      LOG4CXX_DEBUG(logger, boost::format("[loadConfig] descriptorName: %s.") % descriptorName);

      model_file = dir + predicateNode.second.get_child("model").get<std::string> ("<xmlattr>.filename", "unknown");
      ComponentValidator::initModel(model_file);
      LOG4CXX_DEBUG(logger, boost::format("[loadConfig] modelFilename: %s.") % model_file);
      std::cout << model_file << std::endl;
      semanticProperties.push_back(descriptorName);

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
  std::cout << onnx_model_abs << std::endl;
  net = cv::dnn::readNetFromONNX(onnx_model_abs.string());
}

void ComponentValidator::handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification) {
  LOG4CXX_TRACE(logger, "[ComponentValidation] method entered.");
  std::cout << "hallo" << std::endl;

  // get current descriptors to process
  TypesByDescriptorConstPtr descriptors = getDescriptors();
  TypesByDescriptor::const_iterator descriptor_iter;
  std::tr1::unordered_set<long long>::const_iterator typeIds_itr;
  
  MemoryObject::Ptr object = notification->object;

  std::vector<ComponentCandidate> candidates;
  
  // get captured frames info
  const cv::Mat currFrame = object->getCaptureData()->frame;

  // This is just for myself to see what is happening
  std::string filename = "/home/schneiders/Documents/currFrame.jpg"; 
  bool result = cv::imwrite(filename, currFrame);
  
  // Pre-processing values for the input image
  double scalefactor = 1.0;
  cv::Scalar noMean(0.0, 0.0, 0.0);  // we do not want any mean subtraction
  int width = 256;
  int height = 192;
  float threshold = 0.5;

  cv::Size size(width, height);

  ComponentValidator::shapeIdcs = {
    {0, 1, 2, 3},  //shape indices of base component indices
    {8, 9, 10, 11},  //shape indices of side component 1
    {16, 17, 18, 19}  //shape indices of side component 2
  };

  ComponentValidator::spatialIdcs = {
    {12, 13, 14, 15},  //spatial indices of side component 1
    {20, 21, 22, 23}  //spatial indices of side component 2
  };

  ComponentValidator::shapeNames = {
    "cube",
    "sphere",
    "cone",
    "cylinder"
  };

  // convert image to grayscale and floats
  cv::Mat grayscaleFrame;
  cv::cvtColor(currFrame, grayscaleFrame, cv::COLOR_BGR2GRAY);
  std::cout << "size of frame: " << grayscaleFrame.size() << std::endl;

  // convert image to Semantic Encoder dimensions  
  cv::Mat blob;
  cv::dnn::blobFromImage(grayscaleFrame, blob, scalefactor, size, noMean, false); 
  //cv::normalize(blob, blob, 41, 82, cv::NORM_MINMAX);

  // Extract Semantic Descriptions 
  net.setInput(blob);

  cv::Mat semMatrix = net.forward();
  cv::Mat semMatrixSig = semMatrix.clone();

  // Iterate over Semantic Descriptions and print values
  if (!semMatrix.empty() && semMatrix.cols == 24 && semMatrix.rows == 1) {
    
    for (int i = 0; i < semMatrix.cols; ++i) {
  
        semMatrixSig.at<float>(0, i) = sigmoid(semMatrix.at<float>(0, i));
        std::cout << "semantic parameter" << semMatrixSig.at<float>(0, i) << std::endl;
    }
  } else {
      std::cerr << "Invalid matrix dimensions!" << std::endl;
  }

  TypesByDescriptor::const_iterator descriptors_itr;
  for (descriptors_itr = descriptors->begin(); descriptors_itr != descriptors->end(); ++descriptors_itr) {
    
    auto it = std::find(semanticProperties.begin(), semanticProperties.end(), descriptors_itr->first.getName());
    
    if (it == semanticProperties.end()) {
      continue;
    } 

    std::string predicateName = *it;

    bool found = calculateSemProperty(semMatrixSig, predicateName);

  }

}


bool ComponentValidator::calculateSemProperty(const cv::Mat& semMat, const std::string& semPred) {

  ComponentValidator::shapeIdcs = {
    {0, 1, 2, 3},  //shape indices of base component indices
    {8, 9, 10, 11},  //shape indices of side component 1
    {16, 17, 18, 19}  //shape indices of side component 2
  };

  ComponentValidator::shapeNames = {
    "cone",
    "cube",
    "cylinder",
    "sphere"
  };

  bool shapeFound = false;
  double maxValue;
  cv::Point maxLoc;

  for (size_t groupIndex = 0; groupIndex < shapeIdcs.size(); ++groupIndex) {

    const auto& idcs = shapeIdcs[groupIndex];
    // Extract the specified columns for the current group
    cv::Mat shapeParams = semMat.col(idcs[0]);
    for (size_t i = 1; i < idcs.size(); ++i) {
      cv::hconcat(shapeParams, semMat.col(idcs[i]), shapeParams);
    }

    // Find the maximum value among the selected elements for the current group
    cv::minMaxLoc(shapeParams, nullptr, &maxValue, nullptr, &maxLoc);

    // Check if the maximum value is greater than 0.5 for the current group
    if (maxValue > 0.5) {
      std::cout << "The max. value in the current group is greater than 0.5." << std::endl;
      int chosenIndex = maxLoc.x;
      std::string chosenString = shapeNames[chosenIndex];
      
      if (chosenString == semPred) {

        shapeFound = true;
      } else {

        shapeFound = false;
      }
    }
  }
  
  return shapeFound;
}