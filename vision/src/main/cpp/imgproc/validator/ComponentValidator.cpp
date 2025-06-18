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
#include <random> 

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
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/obj_io.h>
#include <pcl/surface/convex_hull.h>


using namespace diarc::stm;
namespace fs = std::experimental::filesystem;


float sigmoid(float x) {
  return 1.0 / (1.0 + std::exp(-x));
}


float denormalize(float normalized_value, float min_value, float max_value) {
  return min_value + (normalized_value * (max_value - min_value));
}


ComponentValidator::ComponentValidator(const long long &processorId, const unsigned int imgWidth,
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
  visualizeReconstruction = config["visualize_reconstruction"].asBool();
  dynamicPadding = config["dynamic_padding"].asBool();
  padding = config["padding"].asInt64();
  kernelSize = config["kernel_size"].asInt64();
  edginessSimilarityThreshold = config["edginess_similarity_threshold"].asDouble();
  shapeThreshold = config["shape_threshold"].asDouble();
  positionToleranceDepth = config["position_tolerance_front_behind"].asDouble();
  positionToleranceHorizontal = config["position_tolerance_left_right"].asDouble();
  positionToleranceVertical = config["position_tolerance_bottom_top"].asDouble();
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

    cv::Mat bestSemMatrixSig = cv::Mat::zeros(1, 24, CV_32F);  // Track the model output with the highest IoU 
    // get current descriptors to process
    TypesByDescriptorConstPtr descriptors = getDescriptors();
    TypesByDescriptor::const_iterator descriptor_iter;
    std::tr1::unordered_set<long long>::const_iterator typeIds_itr;

    // get MemoryObject cloud, denoise and upsample it
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr objectCloudOrig = object->getDetectionMask()->getObjectPointCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr denoisedCloud(new pcl::PointCloud<pcl::PointXYZ>());
    *denoisedCloud = *objectCloudOrig;

    // get depth image from object point cloud
    cv::Mat objectDepthImage(denoisedCloud->height, denoisedCloud->width, CV_32FC1);
    diarc::pc::util::pointCloudToDepth(objectDepthImage, denoisedCloud);

    // align object point cloud with superellipsoid coordinate system
    pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloud = denoisedCloud;
    translatePointCloudToOrigin(objectCloud);
    rotatePointCloud(objectCloud, 180.0f, Eigen::Vector3f::UnitX());
    rotatePointCloud(objectCloud, 90.0f, Eigen::Vector3f::UnitY());
    // reconstruct superellipsoids for different object positions, choose the one with the highest IoU
    BestDescription bestDescriptionCandidate = findBestReconstruction(objectCloud, objectDepthImage, 
                                                                        paddingConfigs);

    if (bestDescriptionCandidate.bestPadIdx != -1 && visualizeReconstruction == true) {

      visualizeSuperellipsoids({combinePointClouds({bestDescriptionCandidate.bestIoUSuperellipsoids})},
                                bestDescriptionCandidate.bestProjectedObject,
                                bestDescriptionCandidate.minChamferDist);
      std::cout << "Press any key to continue." << std::endl;
            }

    // Check if the output matrix is valid before using it
    if (bestDescriptionCandidate.bestSemMatrixSig.empty()) {
      LOG4CXX_DEBUG(logger, boost::format("Semantic Matrix not valid"));
    }

    float confidence = 0.99; //TODO: Should I set this to the IoU value?
    int predShape;
    int predAxis;
    int predIdx;
    bool predFound;
    std::vector<bool> foundPredicatesVector(semanticProperties.size(), false);

    bool sideComp1IsPresent = isSideComponentPresent(1, bestDescriptionCandidate.bestSemMatrixSig);
    bool sideComp2IsPresent = isSideComponentPresent(2, bestDescriptionCandidate.bestSemMatrixSig);

    // Determine if semantic predicates are present for object and save results in vector of booleans
    for (auto sp = semanticProperties.begin(); sp != semanticProperties.end() - 1; ++sp) {
      predShape = sp->predicateShape;
      predAxis = sp->predicateAxis;
      predIdx = sp->predicateIndex;

      // Compute semantic properties for the model output that achieved the highest IoU
      foundPredicatesVector[predIdx] = calculateSemProperty(bestDescriptionCandidate.bestSemMatrixSig, predShape, predAxis, predIdx);

      // If side component 1 is not found, we do not need to go further
      if (sideComp1IsPresent == false && predIdx >= 7) {
        foundPredicatesVector[0] = true;
        break;
      }

      // If side component 2 is not found, we do not need to go further
      if (sideComp2IsPresent == false && predIdx >= 23) {
        foundPredicatesVector[1] = true;
        break;
      }

      // set shape to part if we couldn't assign a shape to the existing component
      if (identifyShapes) {
        if (predIdx == 5) {
          if (foundPredicatesVector[3] == false && foundPredicatesVector[4] == false) {
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
        for (int idx: shapeIndices) {
          foundPredicatesVector[idx] = false;
        }
      }
    }

    if (sideComp1IsPresent && sideComp2IsPresent) {
      foundPredicatesVector[2] = true;
    }

    PredicateHelper::Set::const_iterator descriptors_itr;
    for (descriptors_itr = types_itr->second.begin(); descriptors_itr != types_itr->second.end();
         ++descriptors_itr) {

      std::string keyToFind = descriptors_itr->getName();

      // Add each identified predicate (shape, size, position, ...) as validation result
      if (keyToFind == "any" || keyToFind == "unknown") {
        //populateMemoryObjectDescription(foundPredicatesVector, object);
        std::string descriptor = generateFullDescription(foundPredicatesVector);
        LOG4CXX_DEBUG(logger, boost::format("Description: %s") % descriptor);
        object->addValidationResult(0.9f, PredicateHelper("unknown(" + object->getVariableName() + ")"));
        object->addValidationResult(0.9f, PredicateHelper(descriptor + "(" + object->getVariableName() + ")"));

        sendValidationNotifications(object);

        // Add full text description as validation result
      } else if (keyToFind == "full_description") {

        std::string fullTextDescription = generateFullDescription(foundPredicatesVector);

        object->addValidationResult(confidence, PredicateHelper(*descriptors_itr));
        object->addValidationResult(confidence, PredicateHelper(
                fullTextDescription + "(" + descriptors_itr->getArg(0) + ")"));
        sendValidationNotifications(object);

        // Add validation result for specific predicates we are looking for
      } else {

        auto it = std::find_if(semanticProperties.begin(), semanticProperties.end(),
                               [&keyToFind](const PredicateFeatures &pf) {
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


ComponentValidator::BestDescription ComponentValidator::findBestReconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr& objectCloud,
                                                                               cv::Mat& objectDepthImage,
                                                                               std::vector<std::vector<int>>& paddingConfigs) {
  // Initialize the decription                                                                               
  BestDescription descriptionCandidate{-1.0, std::numeric_limits<float>::max(), -1, 
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>{}, nullptr, cv::Mat()};
  cv::Mat objectDepthImageNorm = objectDepthImage.clone();
  cv::Mat objectDepthImageNormDifPad = objectDepthImage.clone();
  std::vector<std::vector<int>> paddingCandidates;
  removeNansAndNoise(objectDepthImage, objectDepthImageNorm);
  int newWidth = static_cast<int>(objectDepthImageNorm.cols * widthCompressionFactor);
  int newHeight = static_cast<int>(objectDepthImageNorm.rows * heightCompressionFactor);

  auto filterCloud = [](const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {  
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& point : cloud->points) {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        filteredCloud->points.push_back(point);
      }
    }
    filteredCloud->width = filteredCloud->points.size();
    filteredCloud->height = 1;
    filteredCloud->is_dense = true;
    return filteredCloud;
  };

  objectCloud = filterCloud(objectCloud);  // Remove inf values from object point cloud

  // Test each padding and see which description leads to the superellipsoid reconstruction with the lowest Chamfer Distance
  for (int padIdx = 0; padIdx < paddingConfigs.size(); ++padIdx) {
    Eigen::Vector4f bboxObjCloud = computeBoundingBox(objectCloud);
    Eigen::Vector4f centroidObjCloud = computeCentroid(objectCloud);
    
    std::vector<int> extraPadding = paddingConfigs[padIdx];

    applyPaddingToImage(objectDepthImageNorm, objectDepthImageNormDifPad, extraPadding);
    
    // Convert opencv Mat into CNN input format
    double scalefactor = 1.0;
    cv::Scalar noMean(0.0, 0.0, 0.0); 
    cv::Size depthImageSize(depthImageWidth, depthImageHeight);
    cv::Mat blob;
    cv::dnn::blobFromImage(objectDepthImageNormDifPad, blob, scalefactor, depthImageSize, noMean, true, CV_32F);
    // Apply Semantic Encoder
    net.setInput(blob);
    cv::Mat semMatrix = net.forward();

    // Instead of using static padding, use bounding-box based padding 
    if (padIdx == 0 && dynamicPadding == true) {
      paddingCandidates = calculateDynamicPadding(objectDepthImageNorm);
      if (paddingCandidates.size() > 1) {
        paddingConfigs = paddingCandidates;
      }
    }

    // Apply sigmoid to output vector
    cv::Mat semMatrixSig = applySigmoidToMatrix(semMatrix);
    // Denormalize output vector back to actual 3d space
    std::vector<float> semMatrixSigDenorm = denormalizeSemanticParams(semMatrixSig);
    // Create superellipsoid based reconstruction in point cloud format
    auto clouds = createSuperellipsoidObject(semMatrixSigDenorm);
    // Rotate superellipsoid recontruciton such that it aligns with object point cloud COS
    for (auto& cloud : clouds) {
      rotatePointCloud(cloud, -90.0f, Eigen::Vector3f::UnitX());
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr combinedCloud = combinePointClouds(clouds);
    Eigen::Vector4f bboxSuperellipsoid = computeBoundingBox(combinedCloud);
    Eigen::Vector4f centroidSuperellipsoid = computeCentroid(combinedCloud);

    // Re-scale object point cloud to superelipsoid dimensions
    float yScaleFactor = bboxSuperellipsoid[1] / bboxObjCloud[1]; // Height ratio for Y
    float zScaleFactor = bboxSuperellipsoid[2] / bboxObjCloud[2]; // Height ratio for Z
    float maxScaleFactor = std::max(yScaleFactor, zScaleFactor); // Maximum scale factor
    //float mean_scale_factor = ((bbox_superellipsoid[1] + bbox_superellipsoid[2]) / 2) /
    //                          ((bbox_obj_cloud[1] + bbox_obj_cloud[2]) / 2);
    scalePointCloud(objectCloud, maxScaleFactor);
    translatePointCloud(objectCloud, (centroidSuperellipsoid - centroidObjCloud).head<3>());

    // Project both point clouds to y-z plane for IoU computation (we do not have points at the back)
    pcl::PointCloud<pcl::PointXYZ>::Ptr projectedObject(new pcl::PointCloud<pcl::PointXYZ>);
    projectToYZPlane(objectCloud, projectedObject);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> projectedSupereggs;
    for (auto& cloud : clouds) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr projectedCloud(new pcl::PointCloud<pcl::PointXYZ>);
      projectToYZPlane(cloud, projectedCloud);
      projectedSupereggs.push_back(projectedCloud);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr projectedCombinedSuperegg = combinePointClouds(projectedSupereggs);
    
    float chamferDistance = computeChamferDistance(projectedCombinedSuperegg, projectedObject);
    float iou = 0.5; //compute_iou(projected_combined_superegg, projected_object);
    
    // Find the description corresponding to the reconstruction with the highest IoU
    if (chamferDistance < descriptionCandidate.minChamferDist) {
      descriptionCandidate.maxIoU = iou;
      descriptionCandidate.minChamferDist = chamferDistance;
      descriptionCandidate.bestPadIdx = padIdx;
      descriptionCandidate.bestIoUSuperellipsoids = projectedSupereggs;
      descriptionCandidate.bestProjectedObject = projectedObject;
      descriptionCandidate.bestSemMatrixSig = semMatrixSig;
    }
  }

  return descriptionCandidate;
}


std::vector<std::vector<int>> ComponentValidator::calculateDynamicPadding(cv::Mat& objectDepthImage) {
  // Find the maximum value in the image (background value)
  double maxVal;
  cv::minMaxLoc(objectDepthImage, nullptr, &maxVal);
  // Create a binary mask for pixels that are not background
  cv::Mat mask = objectDepthImage < maxVal; // True for foreground pixels
  // Find non-zero points (foreground object pixels)
  std::vector<cv::Point> nonZeroPoints;
  cv::findNonZero(mask, nonZeroPoints);

  // Handle case where no object is found
  if (nonZeroPoints.empty()) {
    return {{0, 0, 0, 0}}; 
  }

  // Get the bounding box of the non-zero points
  cv::Rect boundingBox = cv::boundingRect(nonZeroPoints);

  // Extract bounding box coordinates
  int minX = boundingBox.x;
  int minY = boundingBox.y;
  int maxX = boundingBox.x + boundingBox.width - 1;
  int maxY = boundingBox.y + boundingBox.height - 1;
  int boundingBoxWidth = maxX - minX;
  int boundingBoxHeight = maxY - minY;
    
  // Split region into 3 x 3 grid
  int parts = 3;
  std::vector<int> horizontalPaddings;
  std::vector<int> verticalPaddings;

  // Split boundingBoxWidth into three equal parts
  for (int i = 1; i <= (parts - 1); ++i) {
    horizontalPaddings.push_back(i * (boundingBoxWidth / 2) / parts);
  }
  // Split boundingBoxHeight into three equal parts
  for (int i = 1; i <= (parts - 1); ++i) {
    verticalPaddings.push_back(i * (boundingBoxHeight / 2) / parts);
  }

  // Fill padding configuration based on bounding box dimensions
  std::vector<std::vector<int>> combinations;
  // Add individual paddings (top, bottom, left, right)
  for (const auto& top : verticalPaddings) {
    combinations.push_back({top, 0, 0, 0}); // Top only
  }
  for (const auto& bottom : verticalPaddings) {
    combinations.push_back({0, bottom, 0, 0}); // Bottom only
  }
  for (const auto& left : horizontalPaddings) {
    combinations.push_back({0, 0, left, 0}); // Left only
  }
  for (const auto& right : horizontalPaddings) {
    combinations.push_back({0, 0, 0, right}); // Right only
  }

  // Add combinations of two paddings
  for (const auto& top : verticalPaddings) {
    for (const auto& left : horizontalPaddings) {
      combinations.push_back({top, 0, left, 0}); // Top and left
    }
    for (const auto& right : horizontalPaddings) {
      combinations.push_back({top, 0, 0, right}); // Top and right
    }
  }
  for (const auto& bottom : verticalPaddings) {
    for (const auto& left : horizontalPaddings) {
      combinations.push_back({0, bottom, left, 0}); // Bottom and left
    }
    for (const auto& right : horizontalPaddings) {
      combinations.push_back({0, bottom, 0, right}); // Bottom and right
    }
  }

  return combinations;
}


void ComponentValidator::visualizeSuperellipsoids(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds1,
                                                  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, float chamferDistance) {
  // Define colors (BGR format for OpenCV)
  cv::Vec3b colors[2] = {
    cv::Vec3b(120, 0, 255), 
    cv::Vec3b(0, 255, 0),  
  };

  // Define bounds
  float minY = std::numeric_limits<float>::max();
  float maxY = std::numeric_limits<float>::lowest();
  float minZ = std::numeric_limits<float>::max();
  float maxZ = std::numeric_limits<float>::lowest();

  // Compute the bounds for each cloud in clouds1 and add points to the set
  for (const auto& cloud : clouds1) {
    for (const auto& point : cloud->points) {
      if (point.y < minY) minY = point.y;
      if (point.y > maxY) maxY = point.y;
      if (point.z < minZ) minZ = point.z;
      if (point.z > maxZ) maxZ = point.z;
    }
  }

  // Compute the bounds for cloud2
  for (const auto& point : cloud2->points) {
    if (point.y < minY) minY = point.y;
    if (point.y > maxY) maxY = point.y;
    if (point.z < minZ) minZ = point.z;
    if (point.z > maxZ) maxZ = point.z;
  }

  // Calculate the aspect ratio
  float aspectRatio = (maxZ - minZ) / (maxY - minY);

  // Define image dimensions
  const int baseWidth = 540;
  int height = static_cast<int>(baseWidth / aspectRatio);
  int width = baseWidth;

  // Create an empty image
  cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);

  // Define scale factors for the image
  float yScale = static_cast<float>(height) / 30.0;
  float zScale = static_cast<float>(width) / 30.0;

  // Draw all points for cloud 2 a bit thicker (as object point cloud is quite sparse)
  for (const auto& point : cloud2->points) {
    int x = static_cast<int>((height - 1) - (point.y * yScale + height / 2));
    int y = static_cast<int>((width - 1) - (point.z * zScale + width / 2));

    if (x >= 0 && x < height && y >= 0 && y < width) {
            
      image.at<cv::Vec3b>(x, y) = cv::Vec3b(0, 255, 0); // Green for cloud2
      int radius = 3; // Adjust radius for the circle size
      cv::circle(image, cv::Point(y, x), radius, cv::Scalar(colors[0][0], colors[0][1], colors[0][2]), cv::FILLED);
    }
  }

  // Draw each cloud in clouds1 
  for (size_t i = 0; i < clouds1.size() && i < 3; ++i) { // We have at most 3 clouds
    const auto& cloud = clouds1[i];
    cv::Vec3b color = colors[1]; 

    for (const auto& point : cloud->points) {
      int x = static_cast<int>((height - 1) - (point.y * yScale + height / 2));
      int y = static_cast<int>((width - 1) - (point.z * zScale + width / 2));

      if (x >= 0 && x < height && y >= 0 && y < width) {
        image.at<cv::Vec3b>(x, y) = color;
      }
    }
  }

  // Add IoU text to image
  cv::putText(image, "Chamf. Dist. " + std::to_string(chamferDistance), cv::Point(10, 30), 
              cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 0), 2);  

  // Draw the legend in the bottom left corner
  int legendX = 10;
  int legendY = height - 90;
  int rectSize = 10;

  cv::rectangle(image, cv::Point(legendX, legendY),
                cv::Point(legendX + rectSize, legendY + rectSize ), colors[0], -1);
        
  cv::putText(image, "Target Cloud", cv::Point(legendX + rectSize + 10, legendY + rectSize),
              cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
    
  cv::rectangle(image, cv::Point(legendX, legendY + 65), cv::Point(legendX + rectSize, legendY + rectSize + 45),
                cv::Vec3b(0, 255, 0), -1); 
  cv::putText(image, "Model Output", cv::Point(legendX + rectSize + 10, legendY + rectSize + 55 ),
              cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);

  // Display the image
  cv::imshow("Projected Point Clouds", image);
  cv::waitKey(0);

}


// Function to de-normalize the semMatrixSigValues to the original 3D space for superellipsoid rendering
std::vector<float> ComponentValidator::denormalizeSemanticParams(const cv::Mat& semMatrixSigValuesMat) {
  // Convert cv::Mat to std::vector<float>
  std::vector<float> semMatrixSigValues(semMatrixSigValuesMat.begin<float>(), semMatrixSigValuesMat.end<float>());

  // Vector to store the de-normalized values, set to default value that will tell us later if comp is present
  std::vector<float> zDenorm(21, -100.0f); 

  // De-normalize the sem_params related to 'b'
  zDenorm[0] = denormalize(semMatrixSigValues[3], EDGINESS_MIN, EDGINESS_MAX);
  zDenorm[1] = denormalize(semMatrixSigValues[4], EDGINESS_MIN, EDGINESS_MAX);
  zDenorm[2] = denormalize(semMatrixSigValues[5], AXIS_SCALE_MIN, AXIS_SCALE_MAX);
  zDenorm[3] = denormalize(semMatrixSigValues[6], AXIS_SCALE_MIN, AXIS_SCALE_MAX);
  zDenorm[4] = denormalize(semMatrixSigValues[7], AXIS_SCALE_MIN, AXIS_SCALE_MAX);

  // If side component 1 is present denormalize its values. Otherwise its values will be -100
  float sum1 = 0;

  for (int i = 8; i < 16; ++i) {
    sum1 += semMatrixSigValues[i];
  }

  if (sum1 > existenceThreshold) {
    // De-normalize the sem_params related to 's1'
    zDenorm[5] = denormalize(semMatrixSigValues[8], -2 * AXIS_SCALE_MAX / dfac, 2 * AXIS_SCALE_MAX / dfac);
    zDenorm[6] = denormalize(semMatrixSigValues[9], -2 * AXIS_SCALE_MAX / dfac, 2 * AXIS_SCALE_MAX / dfac);
    zDenorm[7] = denormalize(semMatrixSigValues[10], -2 * AXIS_SCALE_MAX / dfac, 2 * AXIS_SCALE_MAX / dfac);
    zDenorm[8] = denormalize(semMatrixSigValues[11], EDGINESS_MIN, EDGINESS_MAX);
    zDenorm[9] = denormalize(semMatrixSigValues[12], EDGINESS_MIN, EDGINESS_MAX);
    zDenorm[10] = denormalize(semMatrixSigValues[13], AXIS_SCALE_MIN, AXIS_SCALE_MAX);
    zDenorm[11] = denormalize(semMatrixSigValues[14], AXIS_SCALE_MIN, AXIS_SCALE_MAX);
    zDenorm[12] = denormalize(semMatrixSigValues[15], AXIS_SCALE_MIN, AXIS_SCALE_MAX);
    
  }

  // If side component 2 is present denormalize its values. Otherwise its values will be -100
  float sum2 = 0;

  for (int i = 16; i < 24; ++i) {
    sum2 += semMatrixSigValues[i];
  }

  if (sum2 > existenceThreshold) {
    
    // De-normalize the sem_params related to 's2'
    zDenorm[13] = denormalize(semMatrixSigValues[16], -2 * AXIS_SCALE_MAX / dfac, 2 * AXIS_SCALE_MAX / dfac);
    zDenorm[14] = denormalize(semMatrixSigValues[17], -2 * AXIS_SCALE_MAX / dfac, 2 * AXIS_SCALE_MAX / dfac);
    zDenorm[15] = denormalize(semMatrixSigValues[18], -2 * AXIS_SCALE_MAX / dfac, 2 * AXIS_SCALE_MAX / dfac);
    zDenorm[16] = denormalize(semMatrixSigValues[19], EDGINESS_MIN, EDGINESS_MAX);
    zDenorm[17] = denormalize(semMatrixSigValues[20], EDGINESS_MIN, EDGINESS_MAX);
    zDenorm[18] = denormalize(semMatrixSigValues[21], AXIS_SCALE_MIN, AXIS_SCALE_MAX);
    zDenorm[19] = denormalize(semMatrixSigValues[22], AXIS_SCALE_MIN, AXIS_SCALE_MAX);
    zDenorm[20] = denormalize(semMatrixSigValues[23], AXIS_SCALE_MIN, AXIS_SCALE_MAX);
    
  }

  return zDenorm;
}


void ComponentValidator::sampleSuperellipsoidMontecarlo(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                                                        float a_x, float a_y, float a_z, 
                                                        float n_1, float n_2, int num_samples) {
  // Clear the cloud to avoid any issues with previous points
  cloud->points.clear();

  // Create random number generators for x, y, z coordinates
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> dist_x(-a_x, a_x);
  std::uniform_real_distribution<float> dist_y(-a_y, a_y);
  std::uniform_real_distribution<float> dist_z(-a_z, a_z);

  // Monte Carlo method to sample points on the surface
  int samples_added = 0;
  while (samples_added < num_samples) {
    // Generate random x, y, z coordinates within the bounding box
    float x = dist_x(gen);
    float y = dist_y(gen);
    float z = dist_z(gen);

    // Check if the point satisfies the superellipsoid equation (surface condition)
    float lhs = std::pow(std::pow(std::abs(x / a_x), 2.0f / n_2) + std::pow(std::abs(y / a_y), 2.0f / n_2), n_2 / 2.0f) +
                          std::pow(std::abs(z / a_z), 2.0f / n_1);

    // If the point is on the surface (within a tolerance), add it to the cloud
    if (std::abs(lhs - 1.0f) < 1e-3) {  
        cloud->points.emplace_back(x, y, z);
        ++samples_added;
    }
  }

  // Set cloud dimensions for an unorganized point cloud
  cloud->width = cloud->points.size(); // Total number of points
  cloud->height = 1;                   
  cloud->is_dense = true;              

}


void ComponentValidator::sampleSuperellipsoidInverseMapping(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                            float a_x, float a_y, float a_z,
                                                            float n_1, float n_2, int num_samples) { 
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> dist(-1.0f, 1.0f);

  int samples_added = 0;

  while (samples_added < num_samples) {
    // Sample points in the range of [-1, 1] for the mapping
    float u = dist(gen);
    float v = dist(gen);

    // Map the sampled points to the surface
    float theta = M_PI * (u + 1) / 2; // Mapping u to [0, pi]
    float phi = M_PI * (v + 1);        // Mapping v to [0, 2*pi]

    // Compute superellipsoid coordinates
    float x = a_x * copysign(1, cos(phi)) * std::pow(std::abs(cos(phi)), n_1) *
                  copysign(1, cos(theta)) * std::pow(std::abs(cos(theta)), n_2);
    float y = a_y * copysign(1, cos(phi)) * std::pow(std::abs(cos(phi)), n_1) *
                  copysign(1, sin(theta)) * std::pow(std::abs(sin(theta)), n_2);
    float z = a_z * copysign(1, sin(phi)) * std::pow(std::abs(sin(phi)), n_1);

    // Add point to the cloud
    cloud->points.emplace_back(x, y, z);
    ++samples_added;
  }

  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;
}


void ComponentValidator::translatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::Vector3f& translation) {
  // Create an Affine transformation matrix for translation
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() = translation;

  // Apply the translation transformation to the point cloud
  pcl::transformPointCloud(*cloud, *cloud, transform);
}


void ComponentValidator::translatePointCloudToOrigin(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  // Compute the centroid of the point cloud
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);

  // Compute the translation to move the centroid to the origin (0, 0, 0)
  Eigen::Vector3f translation = -centroid.head<3>();  // Negate the centroid position to move to the origin
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() = translation;
  pcl::transformPointCloud(*cloud, *cloud, transform);
}


pcl::PointCloud<pcl::PointXYZ>::Ptr ComponentValidator::createSuperellipsoid(float a_x, float a_y, float a_z,
                                                                             float n_1, float n_2, int resolution,
                                                                             const Eigen::Vector3f& translation) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    
  int number_of_points = (resolution + 1) * (resolution + 1) * 2;
  cloud->width = number_of_points;
  cloud->height = 1;
  cloud->is_dense = false;

  // Combination of superellipsoid sampling leads to dense and well distributed point cloud
  sampleSuperellipsoidMontecarlo(cloud, a_x, a_y, a_z, n_1, n_2, resolution);
  sampleSuperellipsoidInverseMapping(cloud, a_x, a_y, a_z, n_1, n_2, resolution);

  // Translate the superellipsoid to its determined location
  translatePointCloud(cloud, translation);

  return cloud; 
}


std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> ComponentValidator::createSuperellipsoidObject(const std::vector<float>& semMatrixSigValues) {
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
  float EPSILON = 1e-6;  // Add small epsilon for the "is-component-present-check"

  float x_0_1 = 0.0f, y_0_1 = 0.0f, z_0_1 = 0.0f;  // base component is centered
  float n_1_1 = semMatrixSigValues[0], n_2_1 = semMatrixSigValues[1];
  float a_x1 = semMatrixSigValues[2], a_y1 = semMatrixSigValues[3], a_z1 = semMatrixSigValues[4];
  float resolution1 = 1550;
  Eigen::Vector3f translation1(x_0_1, y_0_1, z_0_1);
  Eigen::Vector3f axes1(a_x1, a_y1, a_z1);

  // Create base component superellipsoid
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 = createSuperellipsoid(a_x1, a_y1, a_z1, n_1_1, n_2_1, resolution1, translation1);
   
  // Add the first cloud if it was created successfully
  if (cloud1) {
    clouds.push_back(cloud1);
  }

  float x_0_2 = semMatrixSigValues[5], y_0_2 = semMatrixSigValues[6], z_0_2 = semMatrixSigValues[7];
  float n_1_2 = semMatrixSigValues[8], n_2_2 = semMatrixSigValues[9];
  float a_x2 = semMatrixSigValues[10], a_y2 = semMatrixSigValues[11], a_z2 = semMatrixSigValues[12];
  float resolution2 = 1550;

  Eigen::Vector3f translation2(x_0_2, y_0_2, z_0_2);
  Eigen::Vector3f axes2(a_x2, a_y2, a_z2);

  if (fabs(x_0_2 + 100) >= EPSILON && fabs(y_0_2 + 100) >= EPSILON && fabs(z_0_2 + 100) >= EPSILON &&
    fabs(n_1_2 + 100) >= EPSILON && fabs(n_2_2 + 100) >= EPSILON &&
    fabs(a_x2 + 100) >= EPSILON && fabs(a_y2 + 100) >= EPSILON && fabs(a_z2 + 100) >= EPSILON) {
    
    // Create first side component superellipsoid (while denormalizing, its presence is indicated by values != -100)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 = createSuperellipsoid(a_x2, a_y2, a_z2, n_1_2, n_2_2, resolution2, translation2);
    
    // Add the second cloud if it was created successfully
    if (cloud2) {
      clouds.push_back(cloud2);
    }   
  }  
  
  float x_0_3 = semMatrixSigValues[13], y_0_3 = semMatrixSigValues[14], z_0_3 = semMatrixSigValues[15];
  float n_1_3 = semMatrixSigValues[16], n_2_3 = semMatrixSigValues[17];
  float a_x3 = semMatrixSigValues[18], a_y3 = semMatrixSigValues[19], a_z3 = semMatrixSigValues[20];
  float resolution3 = 1550;

  Eigen::Vector3f translation3(x_0_3, y_0_3, z_0_3);
  Eigen::Vector3f axes3(a_x3, a_y3, a_z3);
  
  if (fabs(x_0_3 + 100) >= EPSILON && fabs(y_0_3 + 100) >= EPSILON && fabs(z_0_3 + 100) >= EPSILON &&
    fabs(n_1_3 + 100) >= EPSILON && fabs(n_2_3 + 100) >= EPSILON &&
    fabs(a_x3 + 100) >= EPSILON && fabs(a_y3 + 100) >= EPSILON && fabs(a_z3 + 100) >= EPSILON) {
    
    // Create second side component superellipsoid (while denormalizing, its presence is indicated by values != -100)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3 = createSuperellipsoid(a_x3, a_y3, a_z3, n_1_3, n_2_3, resolution3, translation3);

    // Add the third cloud if it was created successfully
    if (cloud3) {
      clouds.push_back(cloud3);
    }   
  }
 
  return clouds; // Return the vector of created superellipsoid clouds
}


Eigen::Vector4f ComponentValidator::computeBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D(*cloud, min_pt, max_pt);
  return max_pt - min_pt;  // Return the size (dimensions) of the bounding box
}


Eigen::Vector4f ComponentValidator::computeCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    return centroid;
}


void ComponentValidator::denoiseAndUpsamplePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud,
                                                      pcl::PointCloud<pcl::PointXYZ>::Ptr& outputCloud) {
  // Denoise point cloud before upsampling it
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(inputCloud);
  sor.setKeepOrganized(true);  // We want to keep the organized structure
  sor.setMeanK(50);  // Number of nearest neighbors to analyze for each point
  sor.setStddevMulThresh(1.0);  // Set standard deviation threshold for filtering
  sor.filter(*outputCloud);

  int inputWidth = outputCloud->width;
  int inputHeight = outputCloud->height;

  /*
  // Double the resolution for upsampling
  int outputWidth = inputWidth * 2;
  int outputHeight = inputHeight * 2;
  outputCloud->width = outputWidth;
  outputCloud->height = outputHeight;
  outputCloud->is_dense = inputCloud->is_dense;
  outputCloud->points.resize(outputWidth * outputHeight);

  // Perform bilinear interpolation over the organized grid
  for (int y = 0; y < outputHeight; ++y) {
    for (int x = 0; x < outputWidth; ++x) {
      float gx = x / 2.0f;
      float gy = y / 2.0f;

      int gxi = static_cast<int>(gx);
      int gyi = static_cast<int>(gy);

      // Get fractional part
      float dx = gx - gxi;
      float dy = gy - gyi;

      // Get the four neighboring points from the original grid
      pcl::PointXYZ p00 = inputCloud->at(gxi, gyi);
      pcl::PointXYZ p01 = inputCloud->at(gxi, std::min(gyi + 1, inputHeight - 1));
      pcl::PointXYZ p10 = inputCloud->at(std::min(gxi + 1, inputWidth - 1), gyi);
      pcl::PointXYZ p11 = inputCloud->at(std::min(gxi + 1, inputWidth - 1), std::min(gyi + 1, inputHeight - 1));

      // Bilinear interpolation on XYZ values
      pcl::PointXYZ newPoint;
      newPoint.x = (1 - dx) * ((1 - dy) * p00.x + dy * p01.x) + dx * ((1 - dy) * p10.x + dy * p11.x);
      newPoint.y = (1 - dx) * ((1 - dy) * p00.y + dy * p01.y) + dx * ((1 - dy) * p10.y + dy * p11.y);
      newPoint.z = (1 - dx) * ((1 - dy) * p00.z + dy * p01.z) + dx * ((1 - dy) * p10.z + dy * p11.z);

      // Assign new point to upsampled cloud
      outputCloud->at(x, y) = newPoint;
    }
  }
 
  // Do another stronger denoising with the upsampled cloud
  sor.setInputCloud(outputCloud);
  sor.setKeepOrganized(true);  
  sor.setMeanK(150);  
  sor.setStddevMulThresh(0.5);  
  sor.filter(*outputCloud);
 */
}


void ComponentValidator::rotatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float angle_degrees, const Eigen::Vector3f& axis) {
  // Compute rotation matrix and apply rotation
  float angle_radians = angle_degrees * M_PI / 180.0f;
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.rotate(Eigen::AngleAxisf(angle_radians, axis));
  pcl::transformPointCloud(*cloud, *cloud, transform);
}


pcl::PointCloud<pcl::PointXYZ>::Ptr ComponentValidator::combinePointClouds(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds) {
  // Combine all superellipsoid point clouds into one combined point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr combinedCloud(new pcl::PointCloud<pcl::PointXYZ>());
  for (const auto& cloud : clouds) {
    *combinedCloud += *cloud;  // Add points from each cloud to the combined cloud
  }
  return combinedCloud;
}


void ComponentValidator::scalePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float uniform_scale_factor) {
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.scale(Eigen::Vector3f(uniform_scale_factor, uniform_scale_factor, uniform_scale_factor));

  // Apply uniform scaling to the point cloud
  pcl::transformPointCloud(*cloud, *cloud, transform);
}


void ComponentValidator::projectToYZPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected) {
  cloud_projected->clear();
  for (const auto& point : cloud->points) {
    pcl::PointXYZ projected_point;
    projected_point.x = 0.0f; // Set x to 0 
    projected_point.y = point.y;
    projected_point.z = point.z;
    cloud_projected->points.push_back(projected_point);
  }
  cloud_projected->width = cloud_projected->points.size();
  cloud_projected->height = 1; 
  cloud_projected->is_dense = true;
}


float ComponentValidator::computeIou(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2) {
  // Voxelize both clouds
  voxelizeCloud(cloud1);
  voxelizeCloud(cloud2);

  // Create sets of voxel centers for both clouds
  std::set<std::pair<int, int>> voxels1;
  std::set<std::pair<int, int>> voxels2;

  for (const auto& point : cloud1->points) {
    int y_voxel = static_cast<int>(std::floor(point.y / voxel_size));
    int z_voxel = static_cast<int>(std::floor(point.z / voxel_size));
    voxels1.emplace(y_voxel, z_voxel);
  }

  for (const auto& point : cloud2->points) {
    int y_voxel = static_cast<int>(std::floor(point.y / voxel_size));
    int z_voxel = static_cast<int>(std::floor(point.z / voxel_size));
    voxels2.emplace(y_voxel, z_voxel);
  }

  // Compute the intersection and union of the voxel sets
  std::set<std::pair<int, int>> intersection_voxels;
  std::set_intersection(voxels1.begin(), voxels1.end(),
                        voxels2.begin(), voxels2.end(),
                        std::inserter(intersection_voxels, intersection_voxels.begin()));

  std::set<std::pair<int, int>> union_voxels;
  std::set_union(voxels1.begin(), voxels1.end(),
                 voxels2.begin(), voxels2.end(),
                 std::inserter(union_voxels, union_voxels.begin()));

  // Calculate IoU
  float iou = static_cast<float>(intersection_voxels.size()) / static_cast<float>(union_voxels.size());

  return iou;
}


void ComponentValidator::voxelizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  // Create a set to store unique voxel centers
  std::set<std::pair<int, int>> voxel_centers;

  for (const auto& point : cloud->points) {
    int y_voxel = static_cast<int>(std::floor(point.y / voxel_size));
    int z_voxel = static_cast<int>(std::floor(point.z / voxel_size));
    voxel_centers.emplace(y_voxel, z_voxel);
  }

  // Clear the original cloud and insert voxel centers as new points
  cloud->points.clear();
  for (const auto& voxel : voxel_centers) {
    pcl::PointXYZ new_point;
    new_point.x = 0.0; 
    new_point.y = voxel.first * voxel_size; // Convert back to real coordinates
    new_point.z = voxel.second * voxel_size; // Convert back to real coordinates
    cloud->points.push_back(new_point);
  }
  cloud->width = static_cast<uint32_t>(cloud->points.size());
  cloud->height = 1; 
}


float ComponentValidator::computeChamferDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1, 
                                                 const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2) {

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree1, kdtree2;
  kdtree1.setInputCloud(cloud1);
  kdtree2.setInputCloud(cloud2);

  double distance1 = 0.0;
  double threshold = 1e6;

  for (const auto& point : cloud1->points) {
    std::vector<int> indices(1);
    std::vector<float> dists(1);
    if (kdtree2.nearestKSearch(point, 1, indices, dists) > 0) {

      if (dists[0] < threshold) {
        distance1 += dists[0]; // Accumulate the distance
      } 
    }
  }

  double distance2 = 0.0;
  for (const auto& point : cloud2->points) {
    std::vector<int> indices(1);
    std::vector<float> dists(1);
    if (kdtree1.nearestKSearch(point, 1, indices, dists) > 0) {
      if (dists[0] < threshold) {
        distance2 += dists[0]; // Accumulate the distance
      } 
    }   
  }

  // Average distance from cloud1 to cloud2 and from cloud2 to cloud1
  double chamferDistance = (distance1 / cloud1->size() + distance2 / cloud2->size()) / 2.0;

  return chamferDistance;
}


void ComponentValidator::saveCombinedSuperellipsoid(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds,
                                                    const std::string& output_filename) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud = combinePointClouds(clouds);

  // Convert PointCloud to PolygonMesh
  pcl::PolygonMesh mesh;
  pcl::toPCLPointCloud2(*combined_cloud, mesh.cloud);

  // Save the mesh as an OBJ file (to look at it in blender)
  pcl::io::saveOBJFile(output_filename, mesh);
}


void ComponentValidator::applyPaddingToImage(const cv::Mat &objectDepthImageNorm, cv::Mat &objectDepthImageNormDifPad, 
                                             const std::vector<int>& extraPadding) {
  int newWidth = static_cast<int>(objectDepthImageNorm.cols * widthCompressionFactor);
  int newHeight = static_cast<int>(objectDepthImageNorm.rows * heightCompressionFactor);

  // Add extra padding values if provided, else default to 0
  int topExtra = (extraPadding.size() > 0) ? extraPadding[0] : 0;
  int bottomExtra = (extraPadding.size() > 1) ? extraPadding[1] : 0;
  int leftExtra = (extraPadding.size() > 2) ? extraPadding[2] : 0;
  int rightExtra = (extraPadding.size() > 3) ? extraPadding[3] : 0;

  // Base padding values
  int top = padding + topExtra;
  int bottom = padding + bottomExtra;
  int left = padding * depthImageWidth / depthImageHeight + leftExtra;
  int right = padding * depthImageWidth / depthImageHeight + rightExtra;
  
  int borderType = cv::BORDER_CONSTANT;
  cv::Scalar borderColor(maxNormValue, maxNormValue, maxNormValue);
  cv::copyMakeBorder(objectDepthImageNorm, objectDepthImageNormDifPad, top, bottom, left, right, borderType, borderColor);

  // Resize the image to the dimension / depth range the model expects
  cv::resize(objectDepthImageNormDifPad, objectDepthImageNormDifPad, cv::Size(newWidth, newHeight));
  cv::normalize(objectDepthImageNormDifPad, objectDepthImageNormDifPad, 0.85, 1.0, cv::NORM_MINMAX);

}


void ComponentValidator::createNaNMasks(const cv::Mat &objectDepthImage, cv::Mat &nanMask, cv::Mat &nanBackgroundMask,
                                        cv::Mat &nanInsideMask) {

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


void ComponentValidator::removeNansAndNoise(const cv::Mat &objectDepthImage, cv::Mat &objectDepthImageNorm) {

  // Variables to store the min and max values and their locations
  double minVal, maxVal;
  cv::Point minLoc, maxLoc;

  // Masks to indicate NaN positions for noise removal and background processing
  cv::Mat nanMask, nanBackgroundMask, nanInsideMask;
  createNaNMasks(objectDepthImage, nanMask, nanBackgroundMask, nanInsideMask);

  // Normalize depth values to specific range and set background to maxvalue + 1
  normalizeDepthImage(objectDepthImage, nanBackgroundMask, objectDepthImageNorm);

  // Inpaint NaN values inside object
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
}


void ComponentValidator::centerObject(cv::Mat &objectDepthImageNorm, const cv::Mat &mask) {

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


bool ComponentValidator::isSideComponentPresent(const int compIndex, const cv::Mat &semMat) {

  float dx = semMat.at<float>(0, sideCompIdcs[compIndex - 1][0]);
  float dy = semMat.at<float>(0, sideCompIdcs[compIndex - 1][1]);
  float dz = semMat.at<float>(0, sideCompIdcs[compIndex - 1][2]);
  float n1 = semMat.at<float>(0, sideCompIdcs[compIndex - 1][3]);
  float n2 = semMat.at<float>(0, sideCompIdcs[compIndex - 1][4]);
  float ax = semMat.at<float>(0, sideCompIdcs[compIndex - 1][5]);
  float ay = semMat.at<float>(0, sideCompIdcs[compIndex - 1][6]);
  float az = semMat.at<float>(0, sideCompIdcs[compIndex - 1][7]);

  float sum = dx + dy + dz + n1 + n2 + ax + ay + az;

  if (sum > existenceThreshold) {
    return true;
  } else {
    return false;
  }
}


void ComponentValidator::normalizeDepthImage(const cv::Mat &objectDepthImage, const cv::Mat &mask,
                                             cv::Mat &objectDepthImageNorm) {
  cv::Mat inputROI;
  double minVal, maxVal;
  cv::minMaxLoc(objectDepthImage, &minVal, &maxVal, nullptr, nullptr, mask);

  double scale = (double) (maxNormValue - minNormValue) / (maxVal - minVal);
  double shift = -minVal * scale + minNormValue;

  objectDepthImage.convertTo(objectDepthImageNorm, CV_32FC1, scale, shift);
  objectDepthImageNorm.setTo(cv::Scalar(maxNormValue + 1), ~mask);
}


bool ComponentValidator::calculateSemProperty(const cv::Mat &semMat, const int semPredShape, const int semPredAxis,
                                              const int semPredIdx) {

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
    case 0:
    case 1:
    case 2:
      break;
      //we look for base component shape predicates
    case 3:
    case 4:
    case 5:
      compIndex = 0;
      shapePredicate = true;
      break;
      //we look for side component 1 shape predicates
    case 7:
    case 8:
    case 9:
      compIndex = 1;
      shapePredicate = true;
      break;
      //we look for side component 2 shape predicates
    case 23:
    case 24:
    case 25:
      compIndex = 2;
      shapePredicate = true;
      break;
      //we look for side component 1 position predicates
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    case 16:
    case 17:
    case 18:
      compIndex = 1;
      positionPredicate = true;
      break;
      //we look for side component 2 position predicates
    case 26:
    case 27:
    case 28:
    case 29:
    case 30:
    case 31:
    case 32:
    case 33:
    case 34:
      compIndex = 2;
      positionPredicate = true;
      break;
      //we look for side component 1 size predicates
    case 19:
    case 20:
    case 21:
      compIndex = 1;
      sizePredicate = true;
      break;
      //we look for side component 2 size predicates
    case 35:
    case 36:
    case 37:
      compIndex = 2;
      sizePredicate = true;
      break;
    case 6:
    case 22:
    case 38:
      eccentricityPredicate = true;
      break;
    default: LOG4CXX_DEBUG(logger, "[ComponentValidation] semPredIdx value not defined ");
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


bool ComponentValidator::processShape(const int compIndex, const cv::Mat &semMat, const int semPredShape) {

  const auto &idcs = superEllipsoidParamsIdcs[compIndex];
  // Extract the specified semantic matrix column for the current component
  cv::Mat shapeParams = semMat.col(idcs[0]);
  for (size_t i = 1; i < idcs.size(); ++i) {
    cv::hconcat(shapeParams, semMat.col(idcs[i]), shapeParams);
  }

  const float n1 = shapeParams.at<float>(0, 0);
  const float n2 = shapeParams.at<float>(0, 1);
  const float ax = shapeParams.at<float>(0, 2);
  const float ay = shapeParams.at<float>(0, 3);
  const float az = shapeParams.at<float>(0, 4);

  // Determine if the shape matches the predicate we are looking for
  switch (semPredShape) {
    case 0:
      return isRod(n1, n2, ax, ay, az);
    case 1:
      return isDisk(n1, n2, ax, ay, az);
    default:
      return false;
  }
}


bool ComponentValidator::isRod(const float n1, const float n2, const float ax, const float ay, const float az) {

  bool edginessDifferent = std::abs(n2 - n1) > edginessSimilarityThreshold;
  bool isElonagted = (az - ay) > shapeThreshold;

  return edginessDifferent && isElonagted;
}


bool ComponentValidator::isDisk(const float n1, const float n2, const float ax, const float ay, const float az) {

  bool edginessDifferent = std::abs(n2 - n1) > edginessSimilarityThreshold;
  bool isShort = (az - ay) < shapeThreshold;

  return edginessDifferent && isShort;
}


// Check discrete position 
bool ComponentValidator::processPosition(const int compIndex, const cv::Mat &semMat, const int semPredAxis) {

  // Initialize semMatch to false
  bool semPositionMatch = false;
  float distance;
  double positionTolerance;

  if (semPredAxis == 0 || semPredAxis == 1 || semPredAxis == 6) {  // x-axis distance
    distance = semMat.at<float>(0, positionIdcs[compIndex][0]);
    positionTolerance = positionToleranceDepth;
  } else if (semPredAxis == 2 || semPredAxis == 3 || semPredAxis == 7) {  // y-axis distance
    distance = semMat.at<float>(0, positionIdcs[compIndex][1]);
    positionTolerance = positionToleranceHorizontal;
  } else if (semPredAxis == 4 || semPredAxis == 5 || semPredAxis == 8) {  // z-axis distance
    distance = semMat.at<float>(0, positionIdcs[compIndex][2]);
    positionTolerance = positionToleranceVertical;
  } else {
    std::cerr << "Axis " << semPredAxis << " is not part of the defined groups." << std::endl;
  }

  if ((semPredAxis == 0 || semPredAxis == 2 || semPredAxis == 4) && (distance > (1 - positionTolerance))) {
    // in front (x-axis), to the right (y-axis), on top (z-axis)
    semPositionMatch = true;

  } else if ((semPredAxis == 1 || semPredAxis == 3 || semPredAxis == 5) && (distance < (0 + positionTolerance))) {
    // behind (x-axis), to the left (y-axis), at bottom (z-axis), 
    semPositionMatch = true;

  } else if ((semPredAxis == 6 || semPredAxis == 7 || semPredAxis == 8) && (distance > (0 + positionTolerance)) &&
             (distance < (1 - positionTolerance))) {
    // centered along depth axis (x-axis), horizontally (y-axis), vertically (z-axis), 
    semPositionMatch = false;  // people found this information confusing, so we don't send it. 

  }

  // Return the result
  return semPositionMatch;

}


bool ComponentValidator::processSize(const int compIndex, const cv::Mat &semMat, const int semPredIdx) {

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


bool ComponentValidator::processEccentricity(const cv::Mat &semMat, const int semPredIdx) {

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
cv::Mat ComponentValidator::applySigmoidToMatrix(const cv::Mat &matrix) {
  // Check if the matrix is not empty and has the correct dimensions
  if (!matrix.empty() && matrix.cols == 21 && matrix.rows == 1) {
    // Create a new matrix with 3 additional columns (24 + 3 = 27)
    cv::Mat matrixWithZeros = cv::Mat::zeros(1, 24, CV_32F);

    // Copy original matrix values to the new matrix, starting after the 3 initial zeros
    for (int i = 0; i < matrix.cols; ++i) {
      matrixWithZeros.at<float>(0, i + 3) = sigmoid(matrix.at<float>(0, i));
    }

    return matrixWithZeros; // Return the new matrix with prepended zeros and sigmoid applied
  } else {
    LOG4CXX_DEBUG(logger, "[ComponentValidation] Invalid semantic matrix dimensions ");
    return cv::Mat(); // Return an empty matrix to indicate an error
  }
}

// Generate a full text description based on the identified predicates
void ComponentValidator::populateMemoryObjectDescription(std::vector<bool> foundPredicatesVector, MemoryObject::Ptr mo) {

  // Helper lambdas to determine shape and size
  auto getShape = [&](int rodIndex, int diskIndex) -> std::string {
      if (foundPredicatesVector[rodIndex]) return "rod";
      if (foundPredicatesVector[diskIndex]) return "disk";
      return "part";
  };

  auto getSize = [&](int longIndex) -> std::string {
      return foundPredicatesVector[longIndex] ? "long" : "";
  };

  auto getPosition = [&](std::vector<int> positionIndices) -> std::string {
      std::string position = "";
      const std::vector<std::string> positionLabels = {"in front", "behind", "to the right", "to the left", "on top",
                                                       "at the bottom"};

      for (size_t i = 0; i < positionIndices.size(); ++i) {
        if (foundPredicatesVector[positionIndices[i]]) {
          position += positionLabels[i] + " ";
        }
      }

      return position;
  };

  // add "unknown" to all objects
  mo->addValidationResult(0.9f, PredicateHelper("unknown(" + mo->getVariableName() + ")"));

  // Central component
  std::string shapeCentral = getShape(3, 4);
  std::string sizeCentral = getSize(6);

  // Object with one component
  if (foundPredicatesVector[0]) {
    LOG4CXX_DEBUG(logger, "[populateMemoryObjectDescription] populating single component object");
    mo->addValidationResult(0.9f, PredicateHelper(sizeCentral+ "(" + mo->getVariableName() + ")"));
    mo->addValidationResult(0.9f, PredicateHelper(shapeCentral+ "(" + mo->getVariableName() + ")"));

    // "The object is a " + sizeCentral + " " + shapeCentral + ".";
    return;
  }

  // Object with two components
  if (foundPredicatesVector[1]) {
    LOG4CXX_DEBUG(logger, "[populateMemoryObjectDescription] populating two component object");
    std::string shapeSecond = getShape(7, 8);
    std::string sizeSecond = getSize(22);
    std::string positionsSecond = getPosition({10, 11, 12, 13, 14, 15});

    // central component
    LOG4CXX_DEBUG(logger, "[populateMemoryObjectDescription] populating two component 0");
    MemoryObject::Ptr component1 = MemoryObject::Ptr(new MemoryObject(mo->getId(), "Y", mo->getCaptureData(), mo->getDetectionMask()));
    component1->addValidationResult(0.9f, PredicateHelper(sizeCentral+ "(" + component1->getVariableName()  + ")"));
    component1->addValidationResult(0.9f, PredicateHelper(shapeCentral+ "(" + component1->getVariableName()  + ")"));
    component1->addRelation(0.9f, PredicateHelper("has_a(" + mo->getVariableName() + "," + component1->getVariableName() + ")"), mo);

    // second component
    LOG4CXX_DEBUG(logger, "[populateMemoryObjectDescription] populating two component 1");
    MemoryObject::Ptr component2 = MemoryObject::Ptr(new MemoryObject(mo->getId(), "Z", mo->getCaptureData(), mo->getDetectionMask()));
    component2->addValidationResult(0.9f, PredicateHelper(sizeSecond+ "(" + component2->getVariableName()  + ")"));
    component2->addValidationResult(0.9f, PredicateHelper(shapeSecond+ "(" + component2->getVariableName()  + ")"));
    component2->addRelation(0.9f, PredicateHelper(positionsSecond + "(" + component2->getVariableName() + "," + component1->getVariableName() + ")"), component1);

    LOG4CXX_DEBUG(logger, "[populateMemoryObjectDescription] populating two component 2");
    // "The object is a " + sizeCentral + " " + shapeCentral + " with a " + sizeSecond + " " + shapeSecond + " " + positionsSecond + ".";
    return;
  }

  // Object with three components
  if (foundPredicatesVector[2]) {
    LOG4CXX_DEBUG(logger, "[populateMemoryObjectDescription] populating three component object");
    std::string shapeSecond = getShape(7, 8);
    std::string sizeSecond = getSize(22);
    std::string positionsSecond = getPosition({10, 11, 12, 13, 14, 15});

    std::string shapeThird = getShape(23, 24);
    std::string sizeThird = getSize(38);
    std::string positionsThird = getPosition({26, 27, 28, 29, 30, 31});

    // central component
    LOG4CXX_DEBUG(logger, "[populateMemoryObjectDescription] populating three component 0");
    MemoryObject::Ptr component1 = MemoryObject::Ptr(new MemoryObject(mo->getId(), "Y", mo->getCaptureData(), mo->getDetectionMask()));
    component1->addValidationResult(0.9f, PredicateHelper(sizeCentral+ "(" + component1->getVariableName()  + ")"));
    component1->addValidationResult(0.9f, PredicateHelper(shapeCentral+ "(" + component1->getVariableName()  + ")"));
    component1->addRelation(0.9f, PredicateHelper("has_a(" + mo->getVariableName() + "," + component1->getVariableName() + ")"), mo);

    // second component
    LOG4CXX_DEBUG(logger, "[populateMemoryObjectDescription] populating three component 1");
    MemoryObject::Ptr component2 = MemoryObject::Ptr(new MemoryObject(mo->getId(), "Z", mo->getCaptureData(), mo->getDetectionMask()));
    component2->addValidationResult(0.9f, PredicateHelper(sizeSecond+ "(" + component2->getVariableName()  + ")"));
    component2->addValidationResult(0.9f, PredicateHelper(shapeSecond+ "(" + component2->getVariableName()  + ")"));
    component2->addRelation(0.9f, PredicateHelper(positionsSecond + "(" + component2->getVariableName() + "," + component1->getVariableName() + ")"), component1);

    // third component
    LOG4CXX_DEBUG(logger, "[populateMemoryObjectDescription] populating three component 2");
    MemoryObject::Ptr component3 = MemoryObject::Ptr(new MemoryObject(mo->getId(), "W", mo->getCaptureData(), mo->getDetectionMask()));
    component3->addValidationResult(0.9f, PredicateHelper(sizeThird+ "(" + component3->getVariableName()  + ")"));
    component3->addValidationResult(0.9f, PredicateHelper(shapeThird+ "(" + component3->getVariableName()  + ")"));
    component3->addRelation(0.9f, PredicateHelper(positionsThird + "(" + component3->getVariableName() + "," + component1->getVariableName() + ")"), component1);

    LOG4CXX_DEBUG(logger, "[populateMemoryObjectDescription] populating three component 3");
    //"The object has a " + sizeCentral + " central " + shapeCentral +
    //" with one " + sizeSecond + " " + shapeSecond + " " + positionsSecond +
    //" and a " + sizeThird + " " + shapeThird + " " + positionsThird + ".";
    return;
  }

}

// Generate a full text description based on the identified predicates
std::string ComponentValidator::generateFullDescription(std::vector<bool> foundPredicatesVector) {

  // Helper lambdas to determine shape and size
  auto getShape = [&](int rodIndex, int diskIndex) -> std::string {
      if (foundPredicatesVector[rodIndex]) return "rod";
      if (foundPredicatesVector[diskIndex]) return "disk";
      return "part";
  };

  auto getSize = [&](int longIndex) -> std::string {
      return foundPredicatesVector[longIndex] ? "long" : "";
  };

  auto getPosition = [&](std::vector<int> positionIndices) -> std::string {
      std::string position = "";
      const std::vector<std::string> positionLabels = {"in_front", "behind", "to_the_right", "to_the_left", "on_top",
                                                       "at_the_bottom"};

      for (size_t i = 0; i < positionIndices.size(); ++i) {
        if (foundPredicatesVector[positionIndices[i]]) {
          position += positionLabels[i] + " ";
        }
      }

      return position;
  };

  // Central component
  std::string shapeCentral = getShape(3, 4);
  std::string sizeCentral = getSize(6);

  // Object with one component
  if (foundPredicatesVector[0]) {
    return sizeCentral + "_" + shapeCentral;
  }

  // Object with two components
  if (foundPredicatesVector[1]) {
    std::string shapeSecond = getShape(7, 8);
    std::string sizeSecond = getSize(22);
    std::string positionsSecond = getPosition({10, 11, 12, 13, 14, 15});

    return sizeCentral + "_" + shapeCentral +
           "_with_a_" + sizeSecond + "_" + shapeSecond + "_" + positionsSecond;
  }

  // Object with three components
  if (foundPredicatesVector[2]) {
    std::string shapeSecond = getShape(7, 8);
    std::string sizeSecond = getSize(22);
    std::string positionsSecond = getPosition({10, 11, 12, 13, 14, 15});

    std::string shapeThird = getShape(23, 24);
    std::string sizeThird = getSize(38);
    std::string positionsThird = getPosition({26, 27, 28, 29, 30, 31});

    return sizeCentral + "_central_" + shapeCentral +
           "_with_one_" + sizeSecond + "_" + shapeSecond + "_" + positionsSecond +
           "_and_a_" + sizeThird + "_" + shapeThird + "_" + positionsThird;
  }

  // If no proper description
  return "No proper description generated";
}
