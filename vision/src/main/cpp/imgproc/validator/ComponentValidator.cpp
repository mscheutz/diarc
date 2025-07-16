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
#include <boost/process.hpp>
#include <experimental/filesystem>

#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <thread>
#include <random>
#include <vector>
#include <chrono>

#include "capture/calibration/Cameras.hpp"
#include "display/Display.hpp"
#include "capture/util/CaptureUtilities.hpp"
#include "common/notification/CaptureNotification.hpp"
#include "stm/util/StmUtilities.hpp"
#include "point_clouds/PointCloudUtilities.hpp"
#include "pyvis/PythonHelper.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/obj_io.h>
#include <pcl/surface/convex_hull.h>
#include <numeric> // for accumulate


namespace py = pybind11;
namespace bp = boost::process;

using namespace diarc::stm;
namespace fs = std::experimental::filesystem;

constexpr int SE_SIZE = 8; // Superellipsoid param count


// Function to find orientation using PCA
double computeObjectOrientation(const cv::Mat& img) {
  std::vector<cv::Point> points;

  // Extract non-zero pixels as object points
  for (int y = 0; y < img.rows; ++y) {
    for (int x = 0; x < img.cols; ++x) {
      float depth = img.at<float>(y, x);
      if (depth > 1e-5f && std::isfinite(depth)) {
        points.emplace_back(x, y);
      }
    }
  }

  if (points.size() < 10) return 0.0; // Not enough points to analyze

  cv::Mat data_pts(points.size(), 2, CV_64F);
  for (size_t i = 0; i < points.size(); ++i) {
    data_pts.at<double>(i, 0) = static_cast<double>(points[i].x);
    data_pts.at<double>(i, 1) = static_cast<double>(points[i].y);
  }

  // Perform PCA
  cv::PCA pca_analysis(data_pts, cv::Mat(), cv::PCA::DATA_AS_ROW);

  // Get orientation in radians
  cv::Point2d eigen_vec = cv::Point2d(pca_analysis.eigenvectors.at<double>(0, 0),
                                        pca_analysis.eigenvectors.at<double>(0, 1));
  double angle = std::atan2(eigen_vec.y, eigen_vec.x); // radians

  return angle * 180.0 / CV_PI; // convert to degrees
}


bool compareSE(const std::vector<float>& data, int idx1, int idx2,
                               float cx = 0.0f, float cy = 0.0f, float cz = 0.0f) {
  float dx1 = data[idx1]     - cx;
  float dy1 = data[idx1 + 1] - cy;
  float dz1 = data[idx1 + 2] - cz;
  float dist1 = dx1*dx1 + dy1*dy1 + dz1*dz1;  // squared distance

  float dx2 = data[idx2]     - cx;
  float dy2 = data[idx2 + 1] - cy;
  float dz2 = data[idx2 + 2] - cz;
  float dist2 = dx2*dx2 + dy2*dy2 + dz2*dz2;

  return dist1 < dist2;  // ascending order: closer first
}


std::vector<float> weightedAverageSuperEllipParams(
    const std::vector<std::vector<float>>& superEllipParams,
    const std::vector<float>& chamferWeights,
    size_t topK = 3,
    float epsilon = 1e-6f,
    float maxAbsDist = 10.0f)  // <-- Max allowed deviation per parameter
{
  if (superEllipParams.empty() || superEllipParams.size() != chamferWeights.size())
    return {};

  size_t n = superEllipParams.size();
  size_t paramCount = superEllipParams[0].size();
  topK = std::min(topK, n);  // Clamp to available size

  // Sort indices by ascending Chamfer distance
  std::vector<std::pair<size_t, float>> indexedWeights;
  for (size_t i = 0; i < n; ++i)
    indexedWeights.emplace_back(i, chamferWeights[i]);

  std::sort(indexedWeights.begin(), indexedWeights.end(),
    [](const auto& a, const auto& b) {
      return a.second < b.second;
  });

  // Reference vector = top-1
  const auto& topParams = superEllipParams[indexedWeights[0].first];

  std::vector<float> weightedSum(paramCount, 0.0f);
  std::vector<float> weightPerDim(paramCount, 0.0f);  // new!

  // Loop through top-K vectors
  for (size_t i = 0; i < topK; ++i) {
    size_t idx = indexedWeights[i].first;
    const auto& params = superEllipParams[idx];

    float weight = 1.0f / (chamferWeights[idx] + epsilon);

    for (size_t j = 0; j < paramCount; ++j) {
      float diff = std::abs(params[j] - topParams[j]);
      if (diff <= maxAbsDist) {
        //std::cout << "Adding parameter " << j << " with value " << params[j]
        //                << " and weight " << weight << std::endl;
        weightedSum[j] += params[j] * weight;
        weightPerDim[j] += weight;
      }
    }
  }

  // Normalize each dimension
  std::vector<float> result(paramCount, 0.0f);
  for (size_t j = 0; j < paramCount; ++j) {
    if (weightPerDim[j] > 0.0f)
      result[j] = weightedSum[j] / weightPerDim[j];
    else
      result[j] = 0.0f;  // or topParams[j] if fallback is preferred
  }

  return result;
}


void rotatePointInVector(std::vector<float>& vec, int idx, float angle_degrees, const Eigen::Vector3f& axis) {
  float angle_radians = angle_degrees * M_PI / 180.0f;

  Eigen::Vector3f point(vec[idx], vec[idx + 1], vec[idx + 2]);
  Eigen::Matrix3f rotation = Eigen::AngleAxisf(angle_radians, axis.normalized()).toRotationMatrix();
  Eigen::Vector3f rotated = rotation * point;

  vec[idx]     = rotated.x();
  vec[idx + 1] = rotated.y();
  vec[idx + 2] = rotated.z();
}


void visualizePointClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds1,
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds2) {

  float min_y = std::numeric_limits<float>::max(), max_y = std::numeric_limits<float>::lowest();
  float min_z = std::numeric_limits<float>::max(), max_z = std::numeric_limits<float>::lowest();
  float min_x = std::numeric_limits<float>::max(), max_x = std::numeric_limits<float>::lowest();

  for (const auto& cloud : clouds1) {
    for (const auto& point : cloud->points) {
      min_y = std::min(min_y, point.y);
      max_y = std::max(max_y, point.y);
      min_z = std::min(min_z, point.z);
      max_z = std::max(max_z, point.z);
      min_x = std::min(min_x, point.x);
      max_x = std::max(max_x, point.x);
    }
  }

  for (const auto& cloud : clouds2) {
    for (const auto& point : cloud->points) {
      min_y = std::min(min_y, point.y);
      max_y = std::max(max_y, point.y);
      min_z = std::min(min_z, point.z);
      max_z = std::max(max_z, point.z);
      min_x = std::min(min_x, point.x);
      max_x = std::max(max_x, point.x);
    }
  }

  float aspect_ratio = (max_z - min_z) / (max_y - min_y);
  const int base_width = 440;
  int height = static_cast<int>(base_width / aspect_ratio);
  int width = base_width;

  cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);

  float y_scale = static_cast<float>(height) / (max_y - min_y) / 2;
  float z_scale = static_cast<float>(width) / (max_z - min_z) / 2;

  auto normalize_depth_to_intensity = [min_x, max_x](float x) {
    return static_cast<int>(255 * (x - min_x) / (max_x - min_x)); // Linear normalization
  };

  int point_radius = 3;
  std::vector<std::vector<float>> depth_map(height, std::vector<float>(width, std::numeric_limits<float>::max()));

  auto process_cloud = [&](const auto& cloud, cv::ColormapTypes colormap_type, bool thick) {
    std::vector<std::pair<float, pcl::PointXYZ>> sorted_points;
    for (const auto& point : cloud->points) {
      sorted_points.emplace_back(point.x, point);
    }

    std::sort(sorted_points.begin(), sorted_points.end(), [](const auto& a, const auto& b) {
      return a.first < b.first;
    });

    for (const auto& sorted_point : sorted_points) {
      const pcl::PointXYZ& point = sorted_point.second;
      int intensity = normalize_depth_to_intensity(point.x);

      int x = static_cast<int>((height - 1) - (point.y * y_scale + height / 2));
      int y = static_cast<int>((width - 1) - (point.z * z_scale + width / 2));

      if (x >= 0 && x < height && y >= 0 && y < width) {
        // Ensure the closest point is always used
        if (point.x < depth_map[x][y]) {
          depth_map[x][y] = point.x;  // Store the new closest depth
          // Color-code the point based on its z-value (depth)
          int intensity = normalize_depth_to_intensity(point.x);
          cv::Mat intensityMat(1, 1, CV_8UC1, cv::Scalar(intensity));
          cv::Mat coloredMat;

          cv::applyColorMap(intensityMat, coloredMat, colormap_type);
          cv::Vec3b color = coloredMat.at<cv::Vec3b>(0, 0);
          image.at<cv::Vec3b>(x, y) = color;

          if (thick == true) {
            int radius = 2;
            cv::circle(image, cv::Point(y, x), radius, cv::Scalar(color[0], color[1], color[2]), cv::FILLED);
          }
        }
      }
    }
  };

  // Process the object point cloud
  for (size_t i = 0; i < clouds1.size() && i < 3; ++i) {
    process_cloud(clouds1[i], cv::COLORMAP_AUTUMN, true);
  }

  // Process the superellipsoid reconstruction point clouds
  for (const auto& cloud : clouds2) {
    process_cloud(cloud, cv::COLORMAP_WINTER, true);
  }

  cv::imshow("Projected Point Clouds", image);
  cv::waitKey(0);
}


float sigmoid(float x) {
  return 1.0 / (1.0 + std::exp(-x));
}


float denormalize(float normalized_value, float min_value, float max_value) {
  return min_value + (normalized_value * (max_value - min_value));
}


pcl::PointCloud<pcl::PointXYZ>::Ptr filterOrganizedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
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
}


ComponentValidator::ComponentValidator(const long long &processorId, const unsigned int imgWidth,
                                       const unsigned int imgHeight, const bool isStereo)
        : ObjectValidator(processorId, imgWidth, imgHeight, isStereo) {
  visionProcessName = "ComponentValidator";
  logger = log4cxx::Logger::getLogger("diarc.imgproc.validator.ComponentValidator");
}


ComponentValidator::~ComponentValidator() {
}


void ComponentValidator::loadConfig(const std::string &configFile) {
  LOG4CXX_TRACE(logger, "[loadConfig] method entered.");
  //get directory
  unsigned found = configFile.find_last_of("/\\"); // check if config file is filename or path
  std::string dir = configFile.substr(0, found + 1);
  std::string model_file1, model_file2, model_file3;
  std::string backbone_file, head_file1, head_file2, head_file3;
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
  model_file1 = dir + config["model1"].asString();
  model_file2 = dir + config["model2"].asString();
  model_file3 = dir + config["model3"].asString();

  ComponentValidator::initModels(model_file1, model_file2, model_file3);

  identifyShapes = config["identify_shapes"].asBool();
  visualizeReconstruction = config["visualize_reconstruction"].asBool();
  kernelSize = config["kernel_size"].asInt64();
  edginessSimilarityThreshold = config["edginess_similarity_threshold"].asDouble();
  positionToleranceDepth = config["position_tolerance_front_behind"].asDouble();
  positionToleranceHorizontal = config["position_tolerance_left_right"].asDouble();
  positionToleranceVertical = config["position_tolerance_bottom_top"].asDouble();
  existenceThreshold = config["existence_threshold"].asDouble();
  volumeThreshold = config["volume_threshold"].asDouble();
  eccentricityTolerance = config["eccentricity_tolerance"].asDouble();
}


void ComponentValidator::initModels(const std::string &model_path1, const std::string &model_path2, const std::string &model_path3) {
  nets.push_back(cv::dnn::readNetFromONNX(model_path1 + ".onnx"));
  nets.push_back(cv::dnn::readNetFromONNX(model_path2 + ".onnx"));
  nets.push_back(cv::dnn::readNetFromONNX(model_path3 + ".onnx"));
}

bool ComponentValidator::handleLearningNotification(const std::string& notification) {
  LOG4CXX_DEBUG(logger, boost::format("[ComponentValidation] learning: %s") % notification.c_str());
  // NOTE: loadConfig and init might not have been called yet
 
  std::vector<std::string> args = {
    "--background",
    "--python-use-system-env",
    "--python",
    "/home/schneiders/Documents/Documents/geometric-primitives-dataset/render_blender/render_supereggs_unit_cube_rotated.py",
    "--",
    "--number_of_samples", "10",
    "--num_of_side_components", "2",
    "--shape_type1", "firstshape_rod",
    "--shape_type2", "secondshape_rod",
    "--shape_type3", "thirdshape_rod",
    "--position2", "secondposition_at_the_bottom",
    "--position3", "thirdposition_to_the_left",
    "--size2", "secondsize_smaller",
    "--size3", "thirdsize_larger",
  };

  // Launch Blender process
  int ret = bp::system(
    "/usr/bin/blender",    // Or simply "blender" if it's in PATH
    bp::args(args)
  );

  if (ret != 0) {
    LOG4CXX_ERROR(logger, boost::format("Rendering script returned with exit code != 0"));
  }

  PythonHelper* pyInstance = PythonHelper::getInstance();
  pyInstance->initializePython();
  LOG4CXX_DEBUG(logger, "Training is started.");
  try {
    LOG4CXX_DEBUG(logger, "[learn] waiting on GIL...");
    pyInstance->acquireGIL();
    LOG4CXX_DEBUG(logger, "[learn] GIL acquired.");
    py::module_ trainmodule = pyInstance->importModule("train_encoder_bybind.train_supereggs");
    py::object trainerWrapper = trainmodule.attr("TrainerWrapper");
    py::object trainer = trainerWrapper("local");
    trainer.attr("run")();
    LOG4CXX_DEBUG(logger, "Training finished.");
  } catch (py::error_already_set& e) {
    LOG4CXX_ERROR(logger, boost::format("[init] %s") % e.what());
  }

  pyInstance->releaseGIL();
  return true;
  
}


void ComponentValidator::handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification) {
  LOG4CXX_TRACE(logger, "[ComponentValidation] method entered.");
  /*
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

    scalePointCloudToUnitCube(denoisedCloud, AXIS_UNIT_CUBE_MAX);

    cv::Mat objectDepthImage(denoisedCloud->height, denoisedCloud->width, CV_32FC1);
    diarc::pc::util::pointCloudToDepthWithinReferenceCube(objectDepthImage, denoisedCloud,
                                                        AXIS_UNIT_CUBE_MAX);

    // align object point cloud with superellipsoid coordinate system
    pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloud = denoisedCloud;
    rotatePointCloud(objectCloud, 180.0f, Eigen::Vector3f::UnitX());
    rotatePointCloud(objectCloud, 90.0f, Eigen::Vector3f::UnitY());

    // reconstruct superellipsoids for different rotations, use best (lowest chamfer distance) reconstruction
    BestDescription bestDescriptionCandidate = findBestReconstruction(objectCloud, objectDepthImage);

    if (visualizeReconstruction == true) {

      visualizePointClouds({bestDescriptionCandidate.objectUnRot}, bestDescriptionCandidate.bestSuperellipsoidsUnRot);
      std::cout << "Best Chamfer distance: "
                << bestDescriptionCandidate.minChamferDist << std::endl;
      std::cout << "Press any key to continue." << std::endl;
    }

    // Check if the output matrix is valid before using it
    if (bestDescriptionCandidate.bestSemMatrixSigUnRot.empty()) {
      LOG4CXX_DEBUG(logger, boost::format("Semantic Matrix not valid"));
    }

    float confidence = 0.99; //TODO: Should I set this to the IoU value?
    int predShape;
    int predAxis;
    int predIdx;
    bool predFound;
    std::vector<bool> foundPredicatesVector(semanticProperties.size(), false);

    bool sideComp1IsPresent = isSideComponentPresent(1, bestDescriptionCandidate.bestSemMatrixSigUnRot);
    bool sideComp2IsPresent = isSideComponentPresent(2, bestDescriptionCandidate.bestSemMatrixSigUnRot);

    // Determine if semantic predicates are present for object and save results in vector of booleans
    for (auto sp = semanticProperties.begin(); sp != semanticProperties.end() - 1; ++sp) {
      predShape = sp->predicateShape;
      predAxis = sp->predicateAxis;
      predIdx = sp->predicateIndex;

      // Compute semantic properties for the model output that achieved the highest IoU
      foundPredicatesVector[predIdx] = calculateSemProperty(bestDescriptionCandidate.bestSemMatrixSigUnRot, predShape, predAxis, predIdx);

      // If side component 1 is not found, we do not need to go further
      if (sideComp1IsPresent == false && predIdx >= 9) {
        foundPredicatesVector[0] = true;
        break;
      }

      // If side component 2 is not found, we do not need to go further
      if (sideComp2IsPresent == false && predIdx >= 27) {
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
        if (sideComp1IsPresent == true && predIdx == 11) {
          if (foundPredicatesVector[9] == false && foundPredicatesVector[10] == false)
          {
            foundPredicatesVector[11] = true;
          }
        }
        if (sideComp2IsPresent == true && predIdx == 29) {
          if (foundPredicatesVector[27] == false && foundPredicatesVector[28] == false)
          {
            foundPredicatesVector[29] = true;
          }
        }
      } else {
        std::vector<int> shapeIndices = {3, 4, 5, 9, 10, 11, 27, 28, 29};
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
        LOG4CXX_INFO(logger, boost::format("Description: %s") % descriptor);
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
        //std::cout << "Full description: " << fullTextDescription << std::endl;

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
*/
}


ComponentValidator::BestDescription ComponentValidator::findBestReconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr& objectCloudOrig,
                                                                               cv::Mat& objectDepthImage) {

  //std::vector<std::vector<float>> superEllipParams;
  //std::vector<float> chamferWeights;

  cv::Mat objectDepthImageNorm = objectDepthImage.clone();
  cv::Mat objectDepthImageNormRot = objectDepthImage.clone();

  // Initialize the best description candidate
  BestDescription descriptionCandidate{std::numeric_limits<float>::max(),
                                       std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>{},
                                       nullptr,
                                       cv::Mat(),
                                       0};

  removeNansAndNoise(objectDepthImage, objectDepthImageNorm);

  double currentAngle = computeObjectOrientation(objectDepthImageNorm);
  double desiredAngle = 0.0;
  double rotationAnglePCA = desiredAngle - currentAngle;

  // Remove inf values from object point cloud while keeping its organized structure
  objectCloudOrig = filterOrganizedCloud(objectCloudOrig);

  std::vector<float> rotationAnglesToTest;
  for (int angle = 0; angle < 360; angle += 10) {
      rotationAnglesToTest.push_back(static_cast<float>(angle));
  }

  for (float rotationAngle : rotationAnglesToTest) {

    rotationAngle = rotationAnglePCA + rotationAngle; // Add PCA rotation to the angle
    pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    objectCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*objectCloudOrig);

    // Rotate point cloud and depth image
    if (rotationAngle != 0) {
      rotatePointCloud(objectCloud, rotationAngle, Eigen::Vector3f::UnitX());
      applyRotationToImage(objectDepthImageNorm, objectDepthImageNormRot, rotationAngle);
    }

    // Convert opencv Mat into CNN input format
    double scalefactor = 1.0;
    cv::Scalar noMean(0.0, 0.0, 0.0);
    cv::Size depthImageSize(depthImageWidth, depthImageHeight);

    for (size_t netIdx = 2; netIdx <= 2; ++netIdx) {

      cv::Mat blob;
      cv::dnn::blobFromImage(objectDepthImageNormRot, blob, scalefactor, depthImageSize, noMean, true, CV_32F);

      // Apply Semantic Encoder
      nets[netIdx].setInput(blob);
      cv::Mat semMatrix = nets[netIdx].forward();
      cv::Mat semMatrixSig = applySigmoidToMatrix(semMatrix);

      //cv::Mat blobSingle = blob.reshape(1, 250); // reshape to [192, 256]
      //cv::normalize(blobSingle, blobSingle, 0, 255, cv::NORM_MINMAX);
      //cv::imwrite("/home/schneiders/Desktop/blob.jpg", blobSingle);

      std::vector<float> semMatrixSigDenormUnRot = unrotateSemanticParams(semMatrixSig, rotationAngle);
      auto cloudsUnRot = createSuperellipsoidObject(semMatrixSigDenormUnRot);

      // Rotate superellipsoid recontruciton such that it aligns with object point cloud COS
      for (auto& cloud : cloudsUnRot) {
        rotatePointCloud(cloud, -90.0f, Eigen::Vector3f::UnitX());
      }

      // Project both point clouds to y-z plane for IoU computation (we do not have points at the back)
      pcl::PointCloud<pcl::PointXYZ>::Ptr projectedObject(new pcl::PointCloud<pcl::PointXYZ>);
      projectToYZPlane(objectCloudOrig, projectedObject);

      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> projectedSupereggs;
      for (auto& cloud : cloudsUnRot) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr projectedCloud(new pcl::PointCloud<pcl::PointXYZ>);
        projectToYZPlane(cloud, projectedCloud);
        projectedSupereggs.push_back(projectedCloud);
      }

      //visualizePointClouds({objectCloudOrig}, cloudsUnRot);

      pcl::PointCloud<pcl::PointXYZ>::Ptr combinedSupereggProjected = combinePointClouds(projectedSupereggs);

      float chamferDistance = computeChamferDistance(combinedSupereggProjected, projectedObject);
      //chamferWeights.push_back(chamferDistance);
      //superEllipParams.push_back(semMatrixSigDenormUnRot);

      // Find the description corresponding to the reconstruction with the lowest chamfer distance
      if (chamferDistance < descriptionCandidate.minChamferDist) {

        cv::Mat semInformationUnRot(1, semMatrixSigDenormUnRot.size(), CV_32F, semMatrixSigDenormUnRot.data());

        descriptionCandidate.minChamferDist = chamferDistance;
        descriptionCandidate.bestSuperellipsoidsUnRot = cloudsUnRot;
        descriptionCandidate.objectUnRot = objectCloudOrig;
        descriptionCandidate.bestSemMatrixSigUnRot = semInformationUnRot.clone();
        descriptionCandidate.modelIdx = netIdx;
      }
    }
  }

  /*
  std::vector<float> avgSuperEllipParams = weightedAverageSuperEllipParams(superEllipParams, chamferWeights);

  auto cloudsAvg = createSuperellipsoidObject(avgSuperEllipParams);

  // Rotate superellipsoid recontruciton such that it aligns with object point cloud COS
  for (auto& cloud : cloudsAvg) {
    rotatePointCloud(cloud, -90.0f, Eigen::Vector3f::UnitX());
  }

  visualizePointClouds({objectCloudOrig}, cloudsAvg);
  */
  return descriptionCandidate;

}


// Function to de-normalize the semMatrixSigValues to the original 3D space for superellipsoid rendering
std::vector<float> ComponentValidator::denormalizeSemanticParams(const cv::Mat& semMatrixSigValuesMat) {
  // Convert cv::Mat to std::vector<float>
  std::vector<float> semMatrixSigValues(semMatrixSigValuesMat.begin<float>(), semMatrixSigValuesMat.end<float>());

  // Vector to store the de-normalized values, set to default value that will tell us later if comp is present
  std::vector<float> zDenorm(30, -100.0f);

  // De-normalize the sem_params related to 'b'
  zDenorm[0] = denormalize(semMatrixSigValues[0], -AXIS_UNIT_CUBE_MAX, AXIS_UNIT_CUBE_MAX);
  zDenorm[1] = denormalize(semMatrixSigValues[1], -AXIS_UNIT_CUBE_MAX, AXIS_UNIT_CUBE_MAX);
  zDenorm[2] = denormalize(semMatrixSigValues[2], -AXIS_UNIT_CUBE_MAX, AXIS_UNIT_CUBE_MAX);
  zDenorm[3] = denormalize(semMatrixSigValues[3], EDGINESS_MIN, EDGINESS_MAX);
  zDenorm[4] = denormalize(semMatrixSigValues[4], EDGINESS_MIN, EDGINESS_MAX);
  zDenorm[5] = semMatrixSigValues[5] * AXIS_UNIT_CUBE_MAX;
  zDenorm[6] = semMatrixSigValues[6] * AXIS_UNIT_CUBE_MAX;
  zDenorm[7] = semMatrixSigValues[7] * AXIS_UNIT_CUBE_MAX;
  zDenorm[8] = denormalize(semMatrixSigValues[8], ANGLE_MIN, ANGLE_MAX);
  zDenorm[9] = denormalize(semMatrixSigValues[9], ANGLE_MIN, ANGLE_MAX);

  // If the axes are smaller than what is reasonable (unit cube object), the component is not present
  if (semMatrixSigValues[15] * AXIS_UNIT_CUBE_MAX > 0.5f &&
      semMatrixSigValues[16] * AXIS_UNIT_CUBE_MAX > 0.5f &&
      semMatrixSigValues[17] * AXIS_UNIT_CUBE_MAX > 0.5f) {

    // De-normalize the sem_params related to 's1'
    zDenorm[10] = denormalize(semMatrixSigValues[10], -AXIS_UNIT_CUBE_MAX, AXIS_UNIT_CUBE_MAX);
    zDenorm[11] = denormalize(semMatrixSigValues[11], -AXIS_UNIT_CUBE_MAX, AXIS_UNIT_CUBE_MAX);
    zDenorm[12] = denormalize(semMatrixSigValues[12], -AXIS_UNIT_CUBE_MAX, AXIS_UNIT_CUBE_MAX);
    zDenorm[13] = denormalize(semMatrixSigValues[13], EDGINESS_MIN, EDGINESS_MAX);
    zDenorm[14] = denormalize(semMatrixSigValues[14], EDGINESS_MIN, EDGINESS_MAX);
    zDenorm[15] = semMatrixSigValues[15] * AXIS_UNIT_CUBE_MAX;
    zDenorm[16] = semMatrixSigValues[16] * AXIS_UNIT_CUBE_MAX;
    zDenorm[17] = semMatrixSigValues[17] * AXIS_UNIT_CUBE_MAX;

  }

  // If the axes are smaller than what is reasonable (unit cube object), the component is not present
  if (semMatrixSigValues[23] * AXIS_UNIT_CUBE_MAX > 0.5f &&
      semMatrixSigValues[24] * AXIS_UNIT_CUBE_MAX > 0.5f &&
      semMatrixSigValues[25] * AXIS_UNIT_CUBE_MAX > 0.5f) {

    // De-normalize the sem_params related to 's2'
    zDenorm[18] = denormalize(semMatrixSigValues[18], -AXIS_UNIT_CUBE_MAX, AXIS_UNIT_CUBE_MAX);
    zDenorm[19] = denormalize(semMatrixSigValues[19], -AXIS_UNIT_CUBE_MAX, AXIS_UNIT_CUBE_MAX);
    zDenorm[20] = denormalize(semMatrixSigValues[20], -AXIS_UNIT_CUBE_MAX, AXIS_UNIT_CUBE_MAX);
    zDenorm[21] = denormalize(semMatrixSigValues[21], EDGINESS_MIN, EDGINESS_MAX);
    zDenorm[22] = denormalize(semMatrixSigValues[22], EDGINESS_MIN, EDGINESS_MAX);
    zDenorm[23] = semMatrixSigValues[23] * AXIS_UNIT_CUBE_MAX;
    zDenorm[24] = semMatrixSigValues[24] * AXIS_UNIT_CUBE_MAX;
    zDenorm[25] = semMatrixSigValues[25] * AXIS_UNIT_CUBE_MAX;

  }

  return zDenorm;
}


std::vector<float> ComponentValidator::unrotateSemanticParams(
    const cv::Mat& semMatrixSig,
    float rotationAngle) {

  std::vector<float> semMatrixSigDenorm = denormalizeSemanticParams(semMatrixSig);

  // Copy angle values
  semMatrixSigDenorm[26] = semMatrixSigDenorm[8];
  semMatrixSigDenorm[27] = semMatrixSigDenorm[9];
  semMatrixSigDenorm[28] = semMatrixSigDenorm[8];
  semMatrixSigDenorm[29] = semMatrixSigDenorm[9];

  std::vector<float> semMatrixSigDenormUnRot = semMatrixSigDenorm;

  // Adjust angle
  semMatrixSigDenormUnRot[8] += rotationAngle;

  if (semMatrixSigDenormUnRot[8] >= 360.0f)
    semMatrixSigDenormUnRot[8] -= 360.0f;

  if (semMatrixSigDenormUnRot[8] >= 180.0f)
    semMatrixSigDenormUnRot[8] -= 180.0f;

  float angle = semMatrixSigDenormUnRot[8];
  semMatrixSigDenormUnRot[26] = angle;
  semMatrixSigDenormUnRot[27] = semMatrixSigDenormUnRot[9];
  semMatrixSigDenormUnRot[28] = angle;
  semMatrixSigDenormUnRot[29] = semMatrixSigDenormUnRot[9];

  // Rotate points (assumes rotatePointInVector exists)
  Eigen::Vector3f axis(1, 0, 0);  // X-axis
  rotatePointInVector(semMatrixSigDenormUnRot, 0, -rotationAngle, axis);
  rotatePointInVector(semMatrixSigDenormUnRot, 10, -rotationAngle, axis);
  rotatePointInVector(semMatrixSigDenormUnRot, 18, -rotationAngle, axis);
  std::vector<float> sortedVector = semMatrixSigDenormUnRot;

  // Sort based on z desc, y asc
  std::vector<int> seOffsets = {0, 10};
  std::vector<int> sortedOffsets = seOffsets;

  std::sort(seOffsets.begin(), seOffsets.end(), [&](int a, int b) {
    return compareSE(semMatrixSigDenormUnRot, a, b);
  });


  for (size_t i = 0; i < sortedOffsets.size(); ++i) {
    int srcIdx = sortedOffsets[i];
    int dstIdx = seOffsets[i];

    std::copy(
      semMatrixSigDenormUnRot.begin() + srcIdx,
      semMatrixSigDenormUnRot.begin() + srcIdx + SE_SIZE,
      sortedVector.begin() + dstIdx
    );
  }

  return sortedVector;
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
                                                                             float n_1, float n_2,
                                                                             float azimuth, float elevation,
                                                                             int resolution,
                                                                             const Eigen::Vector3f& translation) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

  int number_of_points = (resolution + 1) * (resolution + 1) * 2;
  cloud->width = number_of_points;
  cloud->height = 1;
  cloud->is_dense = false;

  // Combination of superellipsoid sampling leads to dense and well distributed point cloud
  //sampleSuperellipsoidMontecarlo(cloud, a_x, a_y, a_z, n_1, n_2, resolution);
  sampleSuperellipsoidInverseMapping(cloud, a_x, a_y, a_z, n_1, n_2, resolution);
  // Rotate superellipsoid by azimuth and elevation angles
  rotatePointCloud(cloud, -azimuth, Eigen::Vector3f::UnitX());
  rotatePointCloud(cloud, -elevation, Eigen::Vector3f::UnitZ());
  // Translate the superellipsoid to its determined location
  translatePointCloud(cloud, translation);

  return cloud;
}


std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> ComponentValidator::createSuperellipsoidObject(const std::vector<float>& semMatrixSigValues) {
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
  float EPSILON = 1e-6;  // Add small epsilon for the "is-component-present-check"

  int baseResolution = 10;
  float baseVolume = (4.0f / 3.0f) * M_PI; // Unit superellipsoid volume

  float x_0_1 = semMatrixSigValues[0], y_0_1 =semMatrixSigValues[1], z_0_1 = semMatrixSigValues[2];
  float n_1_1 = semMatrixSigValues[3], n_2_1 = semMatrixSigValues[4];
  float a_x1 = semMatrixSigValues[5], a_y1 = semMatrixSigValues[6], a_z1 = semMatrixSigValues[7];
  float azimuth_1 = semMatrixSigValues[8], elev_1 = semMatrixSigValues[9];

  float volume1 = (4.0f / 3.0f) * M_PI * a_x1 * a_y1 * a_z1;
  int resolution1 = static_cast<int>(std::floor(baseResolution * (volume1 / baseVolume)));
  Eigen::Vector3f translation1(x_0_1, y_0_1, z_0_1);
  Eigen::Vector3f axes1(a_x1, a_y1, a_z1);

  // Create base component superellipsoid
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 = createSuperellipsoid(a_x1, a_y1, a_z1, n_1_1, n_2_1, azimuth_1, elev_1, resolution1, translation1);

  // Add the first cloud if it was created successfully
  if (cloud1) {
    clouds.push_back(cloud1);
  }

  float x_0_2 = semMatrixSigValues[10], y_0_2 = semMatrixSigValues[11], z_0_2 = semMatrixSigValues[12];
  float n_1_2 = semMatrixSigValues[13], n_2_2 = semMatrixSigValues[14];
  float a_x2 = semMatrixSigValues[15], a_y2 = semMatrixSigValues[16], a_z2 = semMatrixSigValues[17];
  float azimuth_2 = semMatrixSigValues[26], elev_2 = semMatrixSigValues[27];

  float volume2 = (4.0f / 3.0f) * M_PI * a_x2 * a_y2 * a_z2;
  int resolution2 = static_cast<int>(std::floor(baseResolution * (volume2 / baseVolume)));

  Eigen::Vector3f translation2(x_0_2, y_0_2, z_0_2);
  Eigen::Vector3f axes2(a_x2, a_y2, a_z2);

  if (fabs(x_0_2 + 100) >= EPSILON && fabs(y_0_2 + 100) >= EPSILON && fabs(z_0_2 + 100) >= EPSILON &&
    fabs(n_1_2 + 100) >= EPSILON && fabs(n_2_2 + 100) >= EPSILON &&
    fabs(a_x2 + 100) >= EPSILON && fabs(a_y2 + 100) >= EPSILON && fabs(a_z2 + 100) >= EPSILON) {

    // Create first side component superellipsoid (while denormalizing, its presence is indicated by values != -100)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 = createSuperellipsoid(a_x2, a_y2, a_z2, n_1_2, n_2_2, azimuth_2, elev_2, resolution2, translation2);

    // Add the second cloud if it was created successfully
    if (cloud2) {
      clouds.push_back(cloud2);
    }
  }

  float x_0_3 = semMatrixSigValues[18], y_0_3 = semMatrixSigValues[19], z_0_3 = semMatrixSigValues[20];
  float n_1_3 = semMatrixSigValues[21], n_2_3 = semMatrixSigValues[22];
  float a_x3 = semMatrixSigValues[23], a_y3 = semMatrixSigValues[24], a_z3 = semMatrixSigValues[25];
  float azimuth_3 = semMatrixSigValues[26], elev_3 = semMatrixSigValues[27];

  float volume3 = (4.0f / 3.0f) * M_PI * a_x3 * a_y3 * a_z3;
  int resolution3 = static_cast<int>(std::floor(baseResolution * (volume3 / baseVolume)));

  Eigen::Vector3f translation3(x_0_3, y_0_3, z_0_3);
  Eigen::Vector3f axes3(a_x3, a_y3, a_z3);

  if (fabs(x_0_3 + 100) >= EPSILON && fabs(y_0_3 + 100) >= EPSILON && fabs(z_0_3 + 100) >= EPSILON &&
    fabs(n_1_3 + 100) >= EPSILON && fabs(n_2_3 + 100) >= EPSILON &&
    fabs(a_x3 + 100) >= EPSILON && fabs(a_y3 + 100) >= EPSILON && fabs(a_z3 + 100) >= EPSILON) {

    // Create second side component superellipsoid (while denormalizing, its presence is indicated by values != -100)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3 = createSuperellipsoid(a_x3, a_y3, a_z3, n_1_3, n_2_3, azimuth_3, elev_3, resolution3, translation3);

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

Eigen::Vector4f ComponentValidator::computeBoundingBoxCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D(*cloud, minPt, maxPt);  // Compute bounding box corners

  // Compute geometric centroid (midpoint of bounding box)
  Eigen::Vector4f bboxCentroid;
  bboxCentroid << (minPt.x + maxPt.x) / 2.0f,
                  (minPt.y + maxPt.y) / 2.0f,
                  (minPt.z + maxPt.z) / 2.0f,
                  1.0f; // Homogeneous coordinates

  return bboxCentroid;
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


void ComponentValidator::scalePointCloudToUnitCube(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float cubeHalfSize) {
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*cloud, min_pt, max_pt);

  // Compute center of bounding box
  Eigen::Vector3f center(
      (min_pt.x + max_pt.x) * 0.5f,
      (min_pt.y + max_pt.y) * 0.5f,
      (min_pt.z + max_pt.z) * 0.5f
  );

  // Compute max extent of the bounding box
  float max_extent = std::max({
      max_pt.x - min_pt.x,
      max_pt.y - min_pt.y,
      max_pt.z - min_pt.z
  });

  if (max_extent <= 0.0f) return; // avoid divide-by-zero

  // Desired scale: we want the largest dimension to span 20 units (-10 to 10)
  float scale = (2.0f * cubeHalfSize) / max_extent;

  // Apply translation and uniform scale
  for (auto& point : cloud->points) {
      point.x = (point.x - center.x()) * scale;
      point.y = (point.y - center.y()) * scale;
      point.z = (point.z - center.z()) * scale;
  }

  pcl::getMinMax3D(*cloud, min_pt, max_pt);

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
  float iou = static_cast<float>(intersection_voxels.size()) / (static_cast<float>(union_voxels.size()));

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


void ComponentValidator::applyRotationToImage(const cv::Mat &objectDepthImageNorm, cv::Mat &objectDepthImageNormRot,
                                                 const float angle) {

  // Background depth values are 0
  int borderType = cv::BORDER_CONSTANT;
  cv::Scalar borderColor(0, 0, 0);

  cv::Point2f center(objectDepthImageNorm.cols / 2.0, objectDepthImageNorm.rows / 2.0);

  // Create the rotation matrix
  cv::Mat rotationMat = cv::getRotationMatrix2D(center, angle, 1.0);

  // Apply the transformation
  cv::warpAffine(objectDepthImageNorm, objectDepthImageNormRot, rotationMat, objectDepthImageNorm.size(),
                 cv::INTER_LINEAR, borderType, borderColor);
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

  objectDepthImageNorm.setTo(cv::Scalar(0), ~nanBackgroundMask);

  //cv::imwrite("/home/schneiders/Desktop/nanBackgroundMask.tiff", nanBackgroundMask);
  //cv::imwrite("/home/schneiders/Desktop/nanInsideMask.tiff", nanInsideMask);

  // Inpaint NaN values inside object
  int impaintingRadius = 30;
  cv::inpaint(objectDepthImageNorm, nanInsideMask, objectDepthImageNorm, impaintingRadius, cv::INPAINT_TELEA);

  cutOutObject(objectDepthImageNorm, nanBackgroundMask);
  //cv::patchNaNs(objectDepthImageNorm, 0.0f);
  //cv::imwrite("/home/schneiders/Desktop/cutOutObject.tiff", objectDepthImageNorm);

  if (objectDepthImageNorm.type() != CV_32FC1) {
    objectDepthImageNorm.convertTo(objectDepthImageNorm, CV_32FC1);
  }

  //cv::Mat filtered;
  //cv::bilateralFilter(objectDepthImageNorm, filtered, 19, 10.0, 19);
  //objectDepthImageNorm = filtered;

  //cv::GaussianBlur(objectDepthImageNorm, objectDepthImageNorm, cv::Size(kernelSize, kernelSize), 0);
  //cv::boxFilter(objectDepthImageNorm, objectDepthImageNorm, -1, cv::Size(kernelSize, kernelSize));

  //centerObject(objectDepthImageNorm, nanBackgroundMask);

  //objectDepthImageNorm.convertTo(objectDepthImageNorm, CV_8U);
  // Apply median blur to the entire image
  //cv::medianBlur(objectDepthImageNorm, objectDepthImageNorm, kernelSize);
  //objectDepthImageNorm.convertTo(objectDepthImageNorm, CV_32F);
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


void ComponentValidator::cutOutObject(cv::Mat &objectDepthImageNorm, const cv::Mat &objectMask) {
  // Find contours in the mask (object outline)
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(objectMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // Return early if no contours are found
  if (contours.empty()) {
    LOG4CXX_TRACE(logger, "[ComponentValidation] No valid object contour found in depth image.");
    return;
  }

  // Find the bounding box of the largest contour
  cv::Rect objectBoundingBox = cv::boundingRect(contours[0]);

  // Ensure the bounding box stays within the image boundaries
  objectBoundingBox &= cv::Rect(0, 0, objectDepthImageNorm.cols, objectDepthImageNorm.rows);

  // Extract the object from the depth image
  cv::Mat cutOut = objectDepthImageNorm(objectBoundingBox).clone();

  int cutOutTargetHeight, cutOutTargetWidth;
  int predefinedPadding;

  // Determine target dimensions while maintaining aspect ratio
  if (objectBoundingBox.height > objectBoundingBox.width) {
    cutOutTargetHeight = depthImageHeight - 2 * paddingTopBottom;
    cutOutTargetWidth = (cutOutTargetHeight * cutOut.cols) / cutOut.rows;
    predefinedPadding = paddingTopBottom;
  } else {
    cutOutTargetWidth = depthImageWidth - 2 * paddingLeftRight;
    cutOutTargetHeight = (cutOutTargetWidth * cutOut.rows) / cutOut.cols;
    predefinedPadding = paddingLeftRight;
  }

  // Resize cut-out image to fit the target dimensions
  cv::resize(cutOut, cutOut, cv::Size(cutOutTargetWidth, cutOutTargetHeight));

  // Create a new depth image filled with the background value
  objectDepthImageNorm = cv::Mat(depthImageHeight, depthImageWidth, cutOut.type(), cv::Scalar(0));

  // Compute placement coordinates to center the cut-out
  int xStart = (depthImageWidth - cutOutTargetWidth) / 2;
  int yStart = (depthImageHeight - cutOutTargetHeight) / 2;

  // Define the target region of interest (ROI) for placing the cut-out
  cv::Rect targetROI(xStart, yStart, cutOutTargetWidth, cutOutTargetHeight);

  // Copy the cut-out to the centered location in the new depth image
  cutOut.copyTo(objectDepthImageNorm(targetROI));

}


bool ComponentValidator::isSideComponentPresent(const int compIndex, const cv::Mat &semMat) {

//  float dx = semMat.at<float>(0, sideCompIdcs[compIndex - 1][0]);
//  float dy = semMat.at<float>(0, sideCompIdcs[compIndex - 1][1]);
//  float dz = semMat.at<float>(0, sideCompIdcs[compIndex - 1][2]);
//  float n1 = semMat.at<float>(0, sideCompIdcs[compIndex - 1][3]);
//  float n2 = semMat.at<float>(0, sideCompIdcs[compIndex - 1][4]);
  float ax = semMat.at<float>(0, sideCompIdcs[compIndex - 1][5]);
  float ay = semMat.at<float>(0, sideCompIdcs[compIndex - 1][6]);
  float az = semMat.at<float>(0, sideCompIdcs[compIndex - 1][7]);

//  float sum = dx + dy + dz + n1 + n2 + ax + ay + az;
  float sum = ax + ay + az;
  LOG4CXX_DEBUG(logger, boost::format("[isSideComponentPresent] compIdx: %d Sum: %f Threshold: %f") % compIndex % sum % existenceThreshold);

  if (sum > existenceThreshold) {
    return true;
  } else {
    return false;
  }
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
  bool orientationPredicate = false;

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
    case 9:
    case 10:
    case 11:
      compIndex = 1;
      shapePredicate = true;
      break;
    //we look for side component 2 shape predicates
    case 27:
    case 28:
    case 29:
      compIndex = 2;
      shapePredicate = true;
      break;
    //we look for side component 1 position predicates
    case 12:
    case 13:
    case 14:
    case 15:
    case 16:
    case 17:
    case 18:
    case 19:
    case 20:
      compIndex = 1;
      positionPredicate = true;
      break;
    //we look for side component 2 position predicates
    case 30:
    case 31:
    case 32:
    case 33:
    case 34:
    case 35:
    case 36:
    case 37:
    case 38:
      compIndex = 2;
      positionPredicate = true;
      break;
    //we look for side component 1 size predicates
    case 21:
    case 22:
    case 23:
      compIndex = 1;
      sizePredicate = true;
      break;
    //we look for side component 2 size predicates
    case 39:
    case 40:
    case 41:
      compIndex = 2;
      sizePredicate = true;
      break;
    case 6:
    case 24:
    case 42:
      eccentricityPredicate = true;
      break;
    //we look for base component orientation predicates
    case 7:
    case 8:
      compIndex = 0;
      orientationPredicate = true;
      break;
    //we look for side component 1 orientation predicates
    case 25:
    case 26:
      compIndex = 1;
      orientationPredicate = true;
      break;
    //we look for side component 2 orientation predicates
    case 43:
    case 44:
      compIndex = 2;
      orientationPredicate = true;
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

  if (orientationPredicate) {
    semMatch = processOrientation(compIndex, semMat, semPredIdx);
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
  float maxAxis = std::max(ay, az);
  float minAxis = std::min(ay, az);
  bool isElongated = (maxAxis / minAxis) > 1.5;

  return edginessDifferent && isElongated;
}


bool ComponentValidator::isDisk(const float n1, const float n2, const float ax, const float ay, const float az) {

  bool edginessDifferent = std::abs(n2 - n1) > edginessSimilarityThreshold;
  float maxAxis = std::max(ay, az);
  float minAxis = std::min(ay, az);
  bool isNotElongated = (maxAxis / minAxis) < 1.1;

  return edginessDifferent && isNotElongated;
}


// Check discrete position
bool ComponentValidator::processPosition(const int compIndex, const cv::Mat &semMat, const int semPredAxis) {

  // Initialize semMatch to false
  bool semPositionMatch = false;
  float distance;
  double positionTolerance;

  if (semPredAxis == 0 || semPredAxis == 1 || semPredAxis == 6) {  // x-axis distance
    distance = semMat.at<float>(0, positionIdcs[compIndex][0]) - semMat.at<float>(0, positionIdcs[0][0]);
    positionTolerance = positionToleranceDepth;
  } else if (semPredAxis == 2 || semPredAxis == 3 || semPredAxis == 7) {  // y-axis distance
    distance = semMat.at<float>(0, positionIdcs[compIndex][1]) - semMat.at<float>(0, positionIdcs[0][1]);
    positionTolerance = positionToleranceHorizontal;
  } else if (semPredAxis == 4 || semPredAxis == 5 || semPredAxis == 8) {  // z-axis distance
    distance = semMat.at<float>(0, positionIdcs[compIndex][2]) - semMat.at<float>(0, positionIdcs[0][2]);
    positionTolerance = positionToleranceVertical;
  } else {
    std::cerr << "Axis " << semPredAxis << " is not part of the defined groups." << std::endl;
  }

  if ((semPredAxis == 0 || semPredAxis == 2 || semPredAxis == 4) && (distance > (0 + positionTolerance))) {
    // in front (x-axis), to the right (y-axis), on top (z-axis)
    semPositionMatch = true;

  } else if ((semPredAxis == 1 || semPredAxis == 3 || semPredAxis == 5) && (distance < (0 - positionTolerance))) {
    // behind (x-axis), to the left (y-axis), at bottom (z-axis),
    semPositionMatch = true;

  } else if ((semPredAxis == 6 || semPredAxis == 7 || semPredAxis == 8) && (distance > (0 - positionTolerance)) &&
             (distance < (0 + positionTolerance))) {
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

  float volumeRatio = volumeSide / volumeBase;

  if ((semPredIdx == 21 || semPredIdx == 39) && (volumeRatio > 1.5)) {
    semSizeMatch = true;  // component is larger
  } else if ((semPredIdx == 22 || semPredIdx == 40) && (volumeRatio < 0.5)) {
    semSizeMatch = true;  // component is smaller
  } else if ((semPredIdx == 23 || semPredIdx == 41) && (volumeRatio < 1.2) &&
             (volumeRatio > 0.8)) {
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
    case 24:
      compIndex = 1;
      break;
      //we look for side component 2 eccentricity
    case 42:
      compIndex = 2;
      break;
    default:
      std::cerr << "semPredIdx is not supposed to appear at the Eccentricity check" << semPredIdx << std::endl;
      return false;
  }

  float ay = semMat.at<float>(0, sizeIdcs[compIndex][1]);
  float az = semMat.at<float>(0, sizeIdcs[compIndex][2]);

  float maxAxis = std::max(ay, az);
  float minAxis = std::min(ay, az);

  // If main axes differ "enough", we consider the component as elongated
  bool semEccentricityMatch = (maxAxis / minAxis) > eccentricityTolerance;

  // Return the result
  return semEccentricityMatch;

}


bool ComponentValidator::processOrientation(const int compIndex, const cv::Mat &semMat, const int semPredIdx) {

  float ay = semMat.at<float>(0, sizeIdcs[compIndex][1]);
  float az = semMat.at<float>(0, sizeIdcs[compIndex][2]);
  float azimuth = semMat.at<float>(0, rotationIdcs[compIndex][0]);
  float maxAxis = std::max(ay, az);
  float minAxis = std::min(ay, az);
  bool isElongated = (maxAxis / minAxis) > 1.5;

  bool semOrientationMatch = false;

  if (isElongated) {
    // Group 1: Oriented along Y-axis if elongated (ay > az)
    if (semPredIdx == 7 || semPredIdx == 25 || semPredIdx == 43) {
      bool yAxisDominant = ay > az;
      bool zAxisDominant = az > ay;

      if (yAxisDominant) {
        // Looking along Y-axis → azimuth ≈ 0° or 180°
        if ((azimuth >= -30 && azimuth <= 30) || (azimuth >= 150 && azimuth <= 210)) {
          semOrientationMatch = true;
        }
      } else if (zAxisDominant) {
        // Looking along Z-axis → azimuth ≈ 90° or 270°
        if ((azimuth >= 60 && azimuth <= 120) || (azimuth >= 240 && azimuth <= 300)) {
          semOrientationMatch = true;
        }
      }
    }

    // Group 2: Oriented along Z-axis if elongated (az > ay)
    else if (semPredIdx == 8 || semPredIdx == 26 || semPredIdx == 44) {
      bool zAxisDominant = az > ay;
      bool yAxisDominant = ay > az;

      if (zAxisDominant) {
        // Looking along Z-axis → azimuth ≈ 0° or 180°
        if ((azimuth >= -30 && azimuth <= 30) || (azimuth >= 150 && azimuth <= 210)) {
          semOrientationMatch = true;
        }
      } else if (yAxisDominant) {
        // Looking along Y-axis → azimuth ≈ 90° or 270°
        if ((azimuth >= 60 && azimuth <= 120) || (azimuth >= 240 && azimuth <= 300)) {
          semOrientationMatch = true;
        }
        }
    }
  }

  return semOrientationMatch;
}


// Function to apply sigmoid to matrix and check dimensions
cv::Mat ComponentValidator::applySigmoidToMatrix(const cv::Mat &matrix) {

  // Check if the matrix is not empty and has the correct dimensions
  if (!matrix.empty() && matrix.cols == 26 && matrix.rows == 1) {
    // Create a new matrix with 3 additional columns (24 + 3 = 27)
    cv::Mat matrixWithZeros = cv::Mat::zeros(1, matrix.cols, CV_32F);

    // Copy original matrix values to the new matrix, starting after the 3 initial zeros
    for (int i = 0; i < matrix.cols; ++i) {
      matrixWithZeros.at<float>(0, i) = sigmoid(matrix.at<float>(0, i));
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

  auto getEccentricity = [&](int longIndex) -> std::string {
    return foundPredicatesVector[longIndex] ? "long" : "";
  };

  auto getOrientation = [&](int horziontalIndex, int verticalIndex) -> std::string {
    if (foundPredicatesVector[horziontalIndex]) return "horizontal";
    if (foundPredicatesVector[verticalIndex]) return "vertical";
    return "";
  };

  auto getSize = [&](int largerIndex, int smallerIndex, int similarSizeIndex) -> std::string {
    if (foundPredicatesVector[largerIndex]) return "larger";
    if (foundPredicatesVector[smallerIndex]) return "smaller";
    if (foundPredicatesVector[similarSizeIndex]) return "similar sized";
    return "";
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
  std::string eccentricityCentral = getEccentricity(6);
  std::string orientationCentral = getOrientation(7, 8);
  std::string shapeCentral = getShape(3, 4);

  // Object with one component
  if (foundPredicatesVector[0]) {
    mo->addValidationResult(0.9f, PredicateHelper(eccentricityCentral+ "(" + mo->getVariableName() + ")"));
    mo->addValidationResult(0.9f, PredicateHelper(orientationCentral+ "(" + mo->getVariableName() + ")"));
    mo->addValidationResult(0.9f, PredicateHelper(shapeCentral+ "(" + mo->getVariableName() + ")"));

    // "The object is a " + eccentricityCentral + " " + orientationCentral + " " + shapeCentral + ".";
    return;
  }

  // Object with two components
  if (foundPredicatesVector[1]) {
    std::string sizeSecond = getSize(21, 22, 23);
    std::string eccentricitySecond = getEccentricity(24);
    std::string orientationSecond = getOrientation(25, 26);
    std::string shapeSecond = getShape(9, 10);
    std::string positionsSecond = getPosition({12, 13, 14, 15, 16, 17});

    // central component
    MemoryObject::Ptr component1 = MemoryObject::Ptr(new MemoryObject(mo->getId(), "Y", mo->getCaptureData(), mo->getDetectionMask()));
    component1->addValidationResult(0.9f, PredicateHelper(eccentricityCentral+ "(" + component1->getVariableName()  + ")"));
    component1->addValidationResult(0.9f, PredicateHelper(orientationCentral+ "(" + component1->getVariableName()  + ")"));
    component1->addValidationResult(0.9f, PredicateHelper(shapeCentral+ "(" + component1->getVariableName()  + ")"));
    component1->addRelation(0.9f, PredicateHelper("has_a(" + mo->getVariableName() + "," + component1->getVariableName() + ")"), mo);

    // second component
    MemoryObject::Ptr component2 = MemoryObject::Ptr(new MemoryObject(mo->getId(), "Z", mo->getCaptureData(), mo->getDetectionMask()));
    component2->addValidationResult(0.9f, PredicateHelper(sizeSecond+ "(" + component2->getVariableName()  + ")"));
    component2->addValidationResult(0.9f, PredicateHelper(eccentricitySecond+ "(" + component2->getVariableName()  + ")"));
    component2->addValidationResult(0.9f, PredicateHelper(orientationSecond+ "(" + component2->getVariableName()  + ")"));
    component2->addValidationResult(0.9f, PredicateHelper(shapeSecond+ "(" + component2->getVariableName()  + ")"));
    component2->addRelation(0.9f, PredicateHelper(positionsSecond + "(" + component2->getVariableName() + "," + component1->getVariableName() + ")"), component1);

    // "The object has a " + eccentricityCentral + " " + orientationCentral + " " + shapeCentral +
    // " with a " + sizeSecond + " " + eccentricitySecond + " " + orientationSecond + " " + shapeSecond + " " + positionsSecond + ".";
    return;
  }

  // Object with three components
  if (foundPredicatesVector[2]) {
    std::string sizeSecond = getSize(21, 22, 23);
    std::string eccentricitySecond = getEccentricity(24);
    std::string orientationSecond = getOrientation(25, 26);
    std::string shapeSecond = getShape(9, 10);
    std::string positionsSecond = getPosition({12, 13, 14, 15, 16, 17});

    std::string sizeThird = getSize(39, 40, 41);
    std::string eccentricityThird = getEccentricity(42);
    std::string orientationThird = getOrientation(43, 44);
    std::string shapeThird = getShape(27, 28);
    std::string positionsThird = getPosition({30, 31, 32, 33, 34, 35});

    // central component
    MemoryObject::Ptr component1 = MemoryObject::Ptr(new MemoryObject(mo->getId(), "Y", mo->getCaptureData(), mo->getDetectionMask()));
    component1->addValidationResult(0.9f, PredicateHelper(eccentricityCentral+ "(" + component1->getVariableName()  + ")"));
    component1->addValidationResult(0.9f, PredicateHelper(orientationCentral+ "(" + component1->getVariableName()  + ")"));
    component1->addValidationResult(0.9f, PredicateHelper(shapeCentral+ "(" + component1->getVariableName()  + ")"));
    component1->addRelation(0.9f, PredicateHelper("has_a(" + mo->getVariableName() + "," + component1->getVariableName() + ")"), mo);

    // second component
    MemoryObject::Ptr component2 = MemoryObject::Ptr(new MemoryObject(mo->getId(), "Z", mo->getCaptureData(), mo->getDetectionMask()));
    component2->addValidationResult(0.9f, PredicateHelper(sizeSecond+ "(" + component2->getVariableName()  + ")"));
    component2->addValidationResult(0.9f, PredicateHelper(eccentricitySecond+ "(" + component2->getVariableName()  + ")"));
    component2->addValidationResult(0.9f, PredicateHelper(orientationSecond+ "(" + component2->getVariableName()  + ")"));
    component2->addValidationResult(0.9f, PredicateHelper(shapeSecond+ "(" + component2->getVariableName()  + ")"));
    component2->addRelation(0.9f, PredicateHelper(positionsSecond + "(" + component2->getVariableName() + "," + component1->getVariableName() + ")"), component1);

    // third component
    MemoryObject::Ptr component3 = MemoryObject::Ptr(new MemoryObject(mo->getId(), "W", mo->getCaptureData(), mo->getDetectionMask()));
    component3->addValidationResult(0.9f, PredicateHelper(sizeThird+ "(" + component3->getVariableName()  + ")"));
    component3->addValidationResult(0.9f, PredicateHelper(eccentricityThird+ "(" + component3->getVariableName()  + ")"));
    component3->addValidationResult(0.9f, PredicateHelper(orientationThird+ "(" + component3->getVariableName()  + ")"));
    component3->addValidationResult(0.9f, PredicateHelper(shapeThird+ "(" + component3->getVariableName()  + ")"));
    component3->addRelation(0.9f, PredicateHelper(positionsThird + "(" + component3->getVariableName() + "," + component1->getVariableName() + ")"), component1);

    // "The object has a " + eccentricityCentral + " " + orientationCentral + " central " + shapeCentral +
    // " with one " + sizeSecond + " " + eccentricitySecond + " " + orientationSecond + " " + shapeSecond + " " + positionsSecond +
    // " and a " + sizeThird + " " + eccentricityThird + " " + orientationThird + " " + shapeThird + " " + positionsThird + ".";
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

  auto getEccentricity = [&](int longIndex) -> std::string {
    return foundPredicatesVector[longIndex] ? "long" : "";
  };

  auto getOrientation = [&](int horziontalIndex, int verticalIndex) -> std::string {
//    if (foundPredicatesVector[horziontalIndex]) return "horizontal";
//    if (foundPredicatesVector[verticalIndex]) return "vertical";
    return "";
  };

  auto getSize = [&](int largerIndex, int smallerIndex, int similarSizeIndex) -> std::string {
    if (foundPredicatesVector[largerIndex]) return "larger";
    if (foundPredicatesVector[smallerIndex]) return "smaller";
    if (foundPredicatesVector[similarSizeIndex]) return "similar sized";
    return "";
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
  std::string eccentricityCentral = getEccentricity(6);
  std::string orientationCentral = getOrientation(7, 8);

  // Object with one component
  if (foundPredicatesVector[0]) {
    LOG4CXX_DEBUG(logger, boost::format("Generating 1-component object description"));
    return eccentricityCentral + "_" + orientationCentral + "_" + shapeCentral;
  }

  // Object with two components
  if (foundPredicatesVector[1]) {
    LOG4CXX_DEBUG(logger, boost::format("Generating 2-component object description"));

    std::string shapeSecond = getShape(9, 10);
    std::string sizeSecond = getSize(21, 22, 23);
    std::string eccentricitySecond = getEccentricity(24);
    std::string positionsSecond = getPosition({12, 13, 14, 15, 16, 17});
    std::string orientationSecond = getOrientation(25, 26);

    return eccentricityCentral + "_" + orientationCentral + "_" + shapeCentral +
           "_with_a_" + sizeSecond + "_" + eccentricitySecond + "_" + orientationSecond + "_" + shapeSecond + "_" + positionsSecond;
  }

  // Object with three components
  if (foundPredicatesVector[2]) {
    LOG4CXX_DEBUG(logger, boost::format("Generating 3-component object description"));

    std::string shapeSecond = getShape(9, 19);
    std::string sizeSecond = getSize(21, 22, 23);
    std::string eccentricitySecond = getEccentricity(24);
    std::string positionsSecond = getPosition({12, 13, 14, 15, 16, 17});
    std::string orientationSecond = getOrientation(25, 26);

    std::string shapeThird = getShape(27, 28);
    std::string sizeThird = getSize(39, 40, 41);
    std::string eccentricityThird = getEccentricity(42);
    std::string positionsThird = getPosition({30, 31, 32, 33, 34, 35});
    std::string orientationThird = getOrientation(43, 44);

    return eccentricityCentral + "_" + orientationCentral + "_central_" + shapeCentral +
           "_with_one_" + sizeSecond + "_" + eccentricitySecond + "_" + orientationSecond + "_" + shapeSecond + "_" + positionsSecond +
           "_and_a_" + sizeThird + "_" + eccentricityThird + "_" + orientationThird + "_" + shapeThird + "_" + positionsThird;
  }

  // If no proper description
  return "No proper description generated";
}
