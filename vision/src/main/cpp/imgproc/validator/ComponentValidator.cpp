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
#include <regex>
#include <filesystem>

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

constexpr int SE_SIZE = 10; // Superellipsoid param count


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

void rotatePointInVector(std::vector<float>& vec, int idx, float angle_degrees, const Eigen::Vector3f& axis) {
  float angle_radians = angle_degrees * M_PI / 180.0f;

  Eigen::Vector3f point(vec[idx], vec[idx + 1], vec[idx + 2]);
  Eigen::Matrix3f rotation = Eigen::AngleAxisf(angle_radians, axis.normalized()).toRotationMatrix();
  Eigen::Vector3f rotated = rotation * point;

  vec[idx]     = rotated.x();
  vec[idx + 1] = rotated.y();
  vec[idx + 2] = rotated.z();
}

cv::Mat visualizePointClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds1,
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds2) {

  // Print min/max for first set of clouds
  for (size_t i = 0; i < clouds1.size(); ++i) {
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*clouds1[i], minPt, maxPt);
  }

  // Print min/max for second set of clouds
  for (size_t i = 0; i < clouds2.size(); ++i) {
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*clouds2[i], minPt, maxPt);
  }

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

  return image;
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
  std::string type;
  std::string descriptorName;
  std::string modelFile;
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

  modelFile = dir + config["model"].asString();
  ComponentValidator::initModels(modelFile);

  identifyShapes = config["identify_shapes"].asBool();
  visualizeReconstruction = config["visualize_reconstruction"].asBool();
  edginessSimilarityThreshold = config["edginess_similarity_threshold"].asDouble();
  positionToleranceDepth = config["position_tolerance_front_behind"].asDouble();
  positionToleranceHorizontal = config["position_tolerance_left_right"].asDouble();
  positionToleranceVertical = config["position_tolerance_bottom_top"].asDouble();
  existenceThreshold = config["existence_threshold"].asDouble();
  volumeThreshold = config["volume_threshold"].asDouble();
  eccentricityTolerance = config["eccentricity_tolerance"].asDouble();
  chamferTolerance = config["chamfer_tolerance"].asDouble();
}

void ComponentValidator::initModels(const std::string &model_path) {
  nets.push_back(cv::dnn::readNetFromONNX(model_path + ".onnx"));
}

bool ComponentValidator::handleLearningNotification(const std::string& notification) {
  LOG4CXX_DEBUG(logger, boost::format("[ComponentValidation] learning: %s") % notification.c_str());
  // NOTE: loadConfig and init might not have been called yet

  int component_count = 0;
  std::unordered_map<int, std::string> shapes;
  parseComponentShapes(notification, component_count, shapes, objectType);

  // Generate args for Blender
  int numberOfSamples = 50;
  
  // Default to generic "part" if shapes are not explicitely given by user
  std::string shape1 = shapes.count(1) ? shapes[1] : "part";
  std::string shape2 = shapes.count(2) ? shapes[2] : "part";
  std::string shape3 = shapes.count(3) ? shapes[3] : "part";

  int numOfSideComponents = component_count > 1 ? component_count - 1 : 0;

  // We do not use component positioning yet, but will probably add that later
  std::vector<std::string> args = {
    "--background", 
    "--python-use-system-env",
    "--python",
    "../diarc/vision/src/main/python/superellipsoid_obj_gen/render_supereggs_unit_cube_rotated.py",
    "--",
    "--object_id", objectType,
    "--number_of_samples", std::to_string(numberOfSamples),
    "--num_of_side_components", std::to_string(numOfSideComponents),
    "--shape_type1", "firstshape_" + shape1
  };

  if (component_count >= 2) {
    args.push_back("--shape_type2");
    args.push_back("secondshape_" + shape2);
  }

  if (component_count >= 3) {
    args.push_back("--shape_type3");
    args.push_back("thirdshape_" + shape3);
  }

  // Build the message string
  std::ostringstream blenderArgs;
  blenderArgs << "\nGenerated Blender args:\n";
  for (const auto& arg : args) {
    blenderArgs << arg << " ";
  }
  // Log with log4cxx
  LOG4CXX_DEBUG(logger, boost::format("[ComponentValidation] blender call: %s") % blenderArgs.str());
  
  // Launch Blender process
  int ret = bp::system(
    "/usr/bin/blender",
    bp::args(args),
    bp::std_out > "blender_output.log",
    bp::std_err > "blender_error.log"
  );

  if (ret != 0) {
    LOG4CXX_ERROR(logger, boost::format("Rendering script returned with exit code != 0"));
  }
  
  PythonHelper* pyInstance = PythonHelper::getInstance();
  LOG4CXX_DEBUG(logger, "Training is started.");
  try {
    LOG4CXX_DEBUG(logger, "[learn] waiting on GIL...");
    pyInstance->acquireGIL();
    LOG4CXX_DEBUG(logger, "[learn] GIL acquired.");
    py::module_ sys = py::module_::import("sys");
    py::module_ io = py::module_::import("io");
    // Create a Python file object that writes to a file
    py::object file = io.attr("open")("superellipsoid_training.log", "a");
    sys.attr("stdout") = file;
    sys.attr("stderr") = file;
    py::module_ trainmodule = pyInstance->importModule("superellipsoid_obj_training.train_supereggs");
    py::object trainerWrapper = trainmodule.attr("TrainerWrapper");
    // 80% of objects (number_of_samples) are used for training, 20% for validation
    // --> Make sure that there is a meaningful number of objects (i.e. not too low)?
    // 50 seem to work okay enough for now --> it depends on the complexity fo the object structure
    py::object trainer = trainerWrapper(objectType, numberOfSamples);  
    // Call the run method to start training
    trainer.attr("run")();
    LOG4CXX_DEBUG(logger, "Training finished.");
  } catch (py::error_already_set& e) {
    LOG4CXX_ERROR(logger, boost::format("[init] %s") % e.what());
  }

  pyInstance->releaseGIL();
  
  std::string newModelName =
    std::string("../vision/src/main/resources/config/edu/tufts/hrilab/vision/imgProcs/componentvalidator/")
    + "model_" + objectType;
  // The model path of the newly trained model will be written to newestModelPathFile (a txt file), 
  // the findBestReconstruction method will use the path given in that file when called after model training
  std::ofstream outFile(newestModelPathFile);
  if (!outFile) {
    std::cerr << "Error: could not open file for writing\n";
    return 1;
  }

  outFile << newModelName << std::endl;
  
  return true;
  
}

void ComponentValidator::handleFrameCompletionNotification(FrameCompletionNotification::ConstPtr notification) {

  // keep track of when a frame has been fully processed for "known" and "unknown" objects
  DescriptorsByTypeConstPtr types = getTypes();
  for (const auto& typeId : notification->typeIds) {
    LOG4CXX_DEBUG(logger, "[handleFrameCompletionNotification] processing notification for typeId: " << typeId << " FrameNum: " << notification->frameNumber);

    if (types->find(typeId) != types->end()) {
      // "unknown" typeId
      unknownMOs.erase(mostRecentUnknownFrameNum); // erase old results
      mostRecentUnknownFrameNum = notification->frameNumber;
    } else {
      // "known" typeId
      if (notification->frameNumber <= mostRecentKnownFrameNum) {
        return;
      }
      knownMOs.erase(mostRecentKnownFrameNum); // erase old results
      mostRecentKnownFrameNum = notification->frameNumber;
    }
  }

  if (mostRecentKnownFrameNum != 0 && mostRecentUnknownFrameNum != 0) {
    // have results from "known" and "unknown" for at least one frame
    LOG4CXX_DEBUG(logger, boost::format("[handleFrameCompletionNotification] have both MO types. Knowns: %d. Unknowns: %d. Filtering knowns ...") % knownMOs[mostRecentKnownFrameNum].size() % unknownMOs[mostRecentUnknownFrameNum].size());
    auto filteredUnknownMOs = unknownMOs[mostRecentUnknownFrameNum];
    removeKnownObjects(knownMOs[mostRecentKnownFrameNum], filteredUnknownMOs);
    LOG4CXX_DEBUG(logger, "[handleFrameCompletionNotification] after filtering. Num of unknowns: " << filteredUnknownMOs.size());
    for (auto& unknownMO : filteredUnknownMOs) {
      processUnknownObject(unknownMO);
      unknownMO->addValidationResult(0.9f, PredicateHelper("unknown(X)"));
      sendValidationNotifications(unknownMO);
    }

    // reset unknownMOs container to avoid duplicate processing
    unknownMOs.erase(mostRecentUnknownFrameNum);
    mostRecentUnknownFrameNum = 0;
  }

  // call base class method to forward notifications
  ObjectValidator::handleFrameCompletionNotification(notification);
}

void ComponentValidator::handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification) {
  LOG4CXX_TRACE(logger, "[handleMemoryObjectNotification] method entered.");

  MemoryObject::Ptr object = notification->object;

  // check if this typeId should currently be processed
  DescriptorsByTypeConstPtr types = getTypes();
  DescriptorsByType::const_iterator types_itr = types->find(object->getTypeId());

  // separate the notification objects into "known" and "unknown" containers
  if (types_itr != types->end()) {
    // "unknown" MO
    if (unknownMOs.find(notification->frameNumber) == unknownMOs.end()) {
      unknownMOs[notification->frameNumber] = std::vector<MemoryObject::Ptr>();
    }
    unknownMOs[notification->frameNumber].push_back(object);
    LOG4CXX_DEBUG(logger, "[handleMemoryObjectNotification] Received \"unknown\" object. Count: " << unknownMOs[notification->frameNumber].size() << " FrameNum: " << notification->frameNumber);
  } else {
    // "known" MO
    if (notification->frameNumber <= mostRecentKnownFrameNum) {
      LOG4CXX_DEBUG(logger, "[handleMemoryObjectNotification] Received old \"known\" object notification. Ignoring. FrameNum: " << notification->frameNumber);
      return;
    }
    if (knownMOs.find(notification->frameNumber) == knownMOs.end()) {
      knownMOs[notification->frameNumber] = std::vector<MemoryObject::Ptr>();
    }
    knownMOs[notification->frameNumber].push_back(object);
    LOG4CXX_DEBUG(logger, "[handleMemoryObjectNotification] Received \"known\" object. Count: " << knownMOs[notification->frameNumber].size() << " FrameNum: " << notification->frameNumber);
  }

}

void ComponentValidator::removeKnownObjects(const std::vector<MemoryObject::Ptr>& knownObjects, std::vector<MemoryObject::Ptr>& unknownObjects) {
  for (auto unknown_itr = unknownObjects.begin(); unknown_itr != unknownObjects.end();) {
    bool matchFound = false;
    for (auto known_itr = knownObjects.begin(); known_itr != knownObjects.end(); ++known_itr) {
      if (calculateBBoxIoU((*known_itr)->getTrackingMask()->getBoundingBox(), (*unknown_itr)->getDetectionMask()->getBoundingBox()) > 0.5) {
        matchFound = true;
        break;
      }
    }

    // remove "unknown" object if it matches with any "known" objects
    if (matchFound) {
      unknown_itr = unknownObjects.erase(unknown_itr);
    } else {
      ++unknown_itr;
    }
  }
}

float ComponentValidator::calculateBBoxIoU(const cv::Rect& rect1, const cv::Rect& rect2) {
  cv::Rect intersection = rect1 & rect2;
  float intersection_area = intersection.area();
  float bbox1_area = rect1.area();
  float bbox2_area = rect2.area();
  float union_area = bbox1_area + bbox2_area - intersection_area;
  return intersection_area / union_area;
}

void ComponentValidator::processUnknownObject(MemoryObject::Ptr object) {
  LOG4CXX_TRACE(logger, "[processUnknownObject] method entered.");

  const auto& types = getTypes();
  DescriptorsByType::const_iterator types_itr = types->find(object->getTypeId());
  if (types_itr == types->end()) {
    LOG4CXX_INFO(logger, "TypeId not being processed by Validator: " << object->getTypeId());;
    return;
  }

  cv::Mat bestSemMatrixSig = cv::Mat::zeros(1, 24, CV_32F); // Track the model output with the highest IoU

  // get MemoryObject cloud, denoise and upsample it
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr objectCloudOrig = object->getDetectionMask()->getObjectPointCloud();
  pcl::PointCloud<pcl::PointXYZ>::Ptr denoisedCloud(new pcl::PointCloud<pcl::PointXYZ>());
  *denoisedCloud = *objectCloudOrig;
  float refCubeScaleFactor = scalePointCloudToUnitCube(denoisedCloud, AXIS_UNIT_CUBE_MAX);
  cv::Mat objectDepthImage(denoisedCloud->height, denoisedCloud->width, CV_32FC1);
  diarc::pc::util::pointCloudToDepthWithinReferenceCube(objectDepthImage, denoisedCloud,
                                                        AXIS_UNIT_CUBE_MAX);

  // align object point cloud with superellipsoid coordinate system
  pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloud = denoisedCloud;
  rotatePointCloud(objectCloud, 180.0f, Eigen::Vector3f::UnitX());
  rotatePointCloud(objectCloud, 90.0f, Eigen::Vector3f::UnitY());

  // reconstruct superellipsoids for different rotations, use best (lowest chamfer distance) reconstruction
  BestDescription bestDescriptionCandidate = findBestReconstruction(objectCloud, objectDepthImage);
  printDimensions(bestDescriptionCandidate, refCubeScaleFactor);

  if (visualizeReconstruction == true) {
    cv::Mat displayFrame = visualizePointClouds({bestDescriptionCandidate.objectUnRot},
                                                bestDescriptionCandidate.bestSuperellipsoidsUnRot);

    std::string savePath = "projected_pointcloud.png";
    cv::imwrite(savePath, displayFrame);
    cv::namedWindow("Visual Object Representation", cv::WINDOW_AUTOSIZE);
    cv::moveWindow("Visual Object Representation", 1340, 50);
    cv::imshow("Visual Object Representation", displayFrame);
    cv::waitKey(2000);
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
    foundPredicatesVector[predIdx] = calculateSemProperty(bestDescriptionCandidate.bestSemMatrixSigUnRot, predShape,
                                                          predAxis, predIdx);

    // If side component 1 is not found, we do not need to go further
    if (sideComp1IsPresent == false && predIdx >= 10) {
      foundPredicatesVector[0] = true;
      break;
    }

    // If side component 2 is not found, we do not need to go further
    if (sideComp2IsPresent == false && predIdx >= 29) {
      foundPredicatesVector[1] = true;
      break;
    }

    // set shape to part if we couldn't assign a shape to the existing component
    if (identifyShapes) {
      if (predIdx == 6) {
        if (foundPredicatesVector[3] == false
          && foundPredicatesVector[4] == false
          && foundPredicatesVector[5] == false) {
          foundPredicatesVector[6] = true;
        }
      }
      if (sideComp1IsPresent == true && predIdx == 13) {
        if (foundPredicatesVector[10] == false
          && foundPredicatesVector[11] == false
          && foundPredicatesVector[12] == false) {
          foundPredicatesVector[13] = true;
        }
      }
      if (sideComp2IsPresent == true && predIdx == 32) {
        if (foundPredicatesVector[29] == false
          && foundPredicatesVector[30] == false
          && foundPredicatesVector[31] == false) {
          foundPredicatesVector[32] = true;
        }
      }
    }
    else {
      std::vector<int> shapeIndices = {3, 4, 5, 6, 9, 10, 11, 12, 13, 29, 30, 31, 32};
      for (int idx : shapeIndices) {
        foundPredicatesVector[idx] = false;
      }
    }
  }

  if (sideComp1IsPresent && sideComp2IsPresent) {
    foundPredicatesVector[2] = true;
  }

  for (const auto& descriptor : types_itr->second) {
    std::string keyToFind = descriptor.getName();

    // Add each identified predicate (shape, size, position, ...) as validation result
    if (keyToFind == "any" || keyToFind == "unknown") {
      //populateMemoryObjectDescription(foundPredicatesVector, object);
      std::string descriptor = generateFullDescription(foundPredicatesVector,
                                                       bestDescriptionCandidate.minChamferDist);
      LOG4CXX_INFO(logger, boost::format("Description: %s") % descriptor);
      object->addValidationResult(0.9f, PredicateHelper("unknown(" + object->getVariableName() + ")"));
      object->addValidationResult(0.9f, PredicateHelper(descriptor + "(" + object->getVariableName() + ")"));

      sendValidationNotifications(object);

      // Add full text description as validation result
    }
    else if (keyToFind == "full_description") {
      std::string fullTextDescription = generateFullDescription(foundPredicatesVector,
                                                                bestDescriptionCandidate.minChamferDist);

      object->addValidationResult(confidence, PredicateHelper(descriptor));
      object->addValidationResult(confidence, PredicateHelper(
                                    fullTextDescription + "(" + descriptor.getArg(0) + ")"));
      sendValidationNotifications(object);
      std::cout << "Full description: " << fullTextDescription << std::endl;

      // Add validation result for specific predicates we are looking for
    }
    else {
      auto it = std::find_if(semanticProperties.begin(), semanticProperties.end(),
                             [&keyToFind](const PredicateFeatures& pf) {
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
      }
      else {
        object->addValidationResult(confidence, descriptor);
        sendValidationNotifications(object);
      }
    }
  }

}


ComponentValidator::BestDescription ComponentValidator::findBestReconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr& objectCloudOrig,
                                                                               cv::Mat& objectDepthImage) {

  cv::Mat objectDepthImageNorm = objectDepthImage.clone();
  cv::Mat objectDepthImageNormRot = objectDepthImage.clone();

  // If user defined object structure before, we'll use that newly model
  // Otherwise, we will use the default model given in the config file
  std::string modelFile;  
  if (std::filesystem::exists(newestModelPathFile)) {
    std::ifstream pathFile(newestModelPathFile);
    if (pathFile.is_open()) {
      std::getline(pathFile, modelFile);
      pathFile.close();
      ComponentValidator::initModels(modelFile);
    }
  }

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
  for (int angle = 0; angle < 3; angle += 10) {
      rotationAnglesToTest.push_back(static_cast<float>(angle));
  }

  for (float rotationAngle : rotationAnglesToTest) {

    //rotationAngle = rotationAnglePCA + rotationAngle; // Add PCA rotation to the angle
    pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    objectCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*objectCloudOrig);

    rotatePointCloud(objectCloud, rotationAngle, Eigen::Vector3f::UnitX());
    applyRotationToImage(objectDepthImageNorm, objectDepthImageNormRot, rotationAngle);
    
    // Convert opencv Mat into CNN input format
    double scalefactor = 1.0;
    cv::Scalar noMean(0.0, 0.0, 0.0);
    cv::Size depthImageSize(depthImageWidth, depthImageHeight);

    // Use most recently added model (the user defined one will be the last one in the list)
    for (size_t netIdx = nets.size() - 1; netIdx < nets.size(); ++netIdx) {

      cv::Mat blob;
      cv::dnn::blobFromImage(objectDepthImageNormRot, blob, scalefactor, depthImageSize, noMean, true, CV_32F);

      // Apply Semantic Encoder
      nets[netIdx].setInput(blob);
      cv::Mat semMatrix = nets[netIdx].forward();

      cv::Mat semMatrixSig = applySigmoidToMatrix(semMatrix);

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

      pcl::PointCloud<pcl::PointXYZ>::Ptr combinedSupereggProjected = combinePointClouds(projectedSupereggs);

      float chamferDistance = computeChamferDistance(combinedSupereggProjected, projectedObject);
      
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
  return descriptionCandidate;
}

void ComponentValidator::parseComponentShapes(const std::string& notification,
                                              int& component_count,
                                              std::unordered_map<int, std::string>& shapes,
                                              std::string& type_name) {
  // Parse component count (rendering code supports 1, 2, or 3 for now)
  std::regex component_pattern(R"(components\((1|2|3)\))");
  std::smatch match;
  component_count = 0;

  if (std::regex_search(notification, match, component_pattern)) {
    component_count = std::stoi(match[1]);
  }

  // Parse shape(i, shapeName)
  std::regex shape_pattern(R"(shape\((\d+),(cylinder|sphere|disk|cuboid|capsule|plate)\))");
  auto begin = std::sregex_iterator(notification.begin(), notification.end(), shape_pattern);
  auto end = std::sregex_iterator();

  for (auto it = begin; it != end; ++it) {
    int index = std::stoi((*it)[1]);
    std::string shape = (*it)[2];
    shapes[index] = shape;
  }

  // Parse type name (final argument)
  std::regex name_pattern(R"(\),([a-zA-Z0-9_]+)\)$)");
  if (std::regex_search(notification, match, name_pattern)) {
    type_name = match[1];
  } else {
    type_name = "unknown";
  }
}

std::vector<float> ComponentValidator::denormalizeSemanticParams(const cv::Mat& semMatrixSigValuesMat) {
  // Convert cv::Mat to std::vector<float>
  std::vector<float> semMatrixSigValues(semMatrixSigValuesMat.begin<float>(), semMatrixSigValuesMat.end<float>());

  // Initialize with -100 to indicate missing components
  std::vector<float> zDenorm(30, -100.0f);

  auto denormalizeBlock = [&](int startIdx, int presenceIdx) {
    // Skip presence check for 'b' (presenceIdx < 0)
    if (presenceIdx >= 0) {
      if (!(semMatrixSigValues[presenceIdx]     * AXIS_UNIT_CUBE_MAX > 0.5f &&
          semMatrixSigValues[presenceIdx + 1] * AXIS_UNIT_CUBE_MAX > 0.5f &&
          semMatrixSigValues[presenceIdx + 2] * AXIS_UNIT_CUBE_MAX > 0.5f)) {
            return; // Component not present
      }
    }

    // Common mapping pattern for each block (10 values)
    // Position (x, y, z), edginess (e1, e2), main axes (ax, ay, az), angle (a1, a2)
    zDenorm[startIdx + 0] = denormalize(semMatrixSigValues[startIdx + 0], -AXIS_UNIT_CUBE_MAX/2, AXIS_UNIT_CUBE_MAX/2);
    zDenorm[startIdx + 1] = denormalize(semMatrixSigValues[startIdx + 1], -AXIS_UNIT_CUBE_MAX/2, AXIS_UNIT_CUBE_MAX/2);
    zDenorm[startIdx + 2] = denormalize(semMatrixSigValues[startIdx + 2], -AXIS_UNIT_CUBE_MAX/2, AXIS_UNIT_CUBE_MAX/2);
    zDenorm[startIdx + 3] = denormalize(semMatrixSigValues[startIdx + 3], EDGINESS_MIN, EDGINESS_MAX);
    zDenorm[startIdx + 4] = denormalize(semMatrixSigValues[startIdx + 4], EDGINESS_MIN, EDGINESS_MAX);
    zDenorm[startIdx + 5] = semMatrixSigValues[startIdx + 5] * AXIS_UNIT_CUBE_MAX;
    zDenorm[startIdx + 6] = semMatrixSigValues[startIdx + 6] * AXIS_UNIT_CUBE_MAX;
    zDenorm[startIdx + 7] = semMatrixSigValues[startIdx + 7] * AXIS_UNIT_CUBE_MAX;
    // Denormalize angles
    zDenorm[startIdx + 8] = denormalize(semMatrixSigValues[startIdx + 8], ANGLE_MIN, ANGLE_MAX);
    zDenorm[startIdx + 9] = denormalize(semMatrixSigValues[startIdx + 9], ANGLE_MIN, ANGLE_MAX/2);
    
  };

  // Apply to each block
  denormalizeBlock(0,  -1);  // 'b' (no presence check)
  denormalizeBlock(10, 15);  // 's1'
  denormalizeBlock(20, 25);  // 's2'
  
  /*
  auto printBlock = [&](const std::string& name, int idx) {
    std::cout << name << ": ";
    for (int i = 0; i < 10; ++i) {
      std::cout << zDenorm[idx + i] << (i < 9 ? ", " : "\n");
    }
  };
  printBlock("b", 0);
  printBlock("s1", 10);
  printBlock("s2", 20);
  */
 
  return zDenorm;
}

std::vector<float> ComponentValidator::unrotateSemanticParams(
    const cv::Mat& semMatrixSig,
    float rotationAngle) {

  std::vector<float> semMatrixSigDenorm = denormalizeSemanticParams(semMatrixSig);
  std::vector<float> semMatrixSigDenormUnRot = semMatrixSigDenorm;

  // Adjust angle for symmetries
  auto wrapAngle = [](float& angle) {
      if (angle >= 360.0f) angle -= 360.0f;
      if (angle >= 180.0f) angle -= 180.0f;
  };

  const int rotationIndices[] = {8, 18, 28};
  for (int idx : rotationIndices) {
      semMatrixSigDenormUnRot[idx] += rotationAngle;
      wrapAngle(semMatrixSigDenormUnRot[idx]);
  }

  // Rotate points (assumes rotatePointInVector exists)
  Eigen::Vector3f axis(1, 0, 0);  // X-axis
  rotatePointInVector(semMatrixSigDenormUnRot, 0, -rotationAngle, axis);
  rotatePointInVector(semMatrixSigDenormUnRot, 10, -rotationAngle, axis);
  rotatePointInVector(semMatrixSigDenormUnRot, 20, -rotationAngle, axis);
  std::vector<float> sortedVector = semMatrixSigDenormUnRot;
  
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
  sampleSuperellipsoidMontecarlo(cloud, a_x, a_y, a_z, n_1, n_2, resolution);
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

  int baseResolution = 20;
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
  float azimuth_2 = semMatrixSigValues[18], elev_2 = semMatrixSigValues[19];

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

  float x_0_3 = semMatrixSigValues[20], y_0_3 = semMatrixSigValues[21], z_0_3 = semMatrixSigValues[22];
  float n_1_3 = semMatrixSigValues[23], n_2_3 = semMatrixSigValues[24];
  float a_x3 = semMatrixSigValues[25], a_y3 = semMatrixSigValues[26], a_z3 = semMatrixSigValues[27];
  float azimuth_3 = semMatrixSigValues[28], elev_3 = semMatrixSigValues[29];

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
  // Remove NaNs first since organized neighbor search needs finite values
  pcl::PointCloud<pcl::PointXYZ>::Ptr finiteCloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> indices;
  
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

float ComponentValidator::scalePointCloudToUnitCube(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float cubeHalfSize) {

  for (auto& pt : cloud->points) {
    if (!pcl::isFinite(pt)) {
      pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
    }
  }
  
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*cloud, min_pt, max_pt);

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

  if (max_extent <= 0.0f) return 1.0f; // avoid divide-by-zero

  // Desired scale: we want the largest dimension to span 20 units (-10 to 10)
  float scale = (2.0f * cubeHalfSize) / max_extent;

  // Apply translation and uniform scale
  for (auto& point : cloud->points) {
    point.x = (point.x - center.x()) * scale;
    point.y = (point.y - center.y()) * scale;
    point.z = (point.z - center.z()) * scale;
  }

  return scale;  // Return scale factor for dimension estimation
}

void ComponentValidator::printDimensions(
  BestDescription& bestDescriptionCandidate,
  float refCubeScaleFactor)
{

  pcl::PointXYZ min_point;
  pcl::PointXYZ max_point;
  Eigen::Vector3f mass_center_base_tmp;
  
  // Rescale point clouds back to original size for dimension estimation
  scalePointCloud(
    bestDescriptionCandidate.bestSuperellipsoidsUnRot[0], 1/refCubeScaleFactor);
  scalePointCloud(
    bestDescriptionCandidate.objectUnRot, 1/refCubeScaleFactor);

  diarc::pc::util::calculateBoundingBox(bestDescriptionCandidate.objectUnRot, 
                                        min_point, max_point, 
                                        mass_center_base_tmp); 

  diarc::pc::util::calculateBoundingBox(bestDescriptionCandidate.bestSuperellipsoidsUnRot[0], 
                                        min_point, max_point, 
                                        mass_center_base_tmp);
  
  // Encoder COS differs from DIARC COS (x, y, z) -> (z, y, x)
  const char* axes[3] = {"z", "y", "x"};

  // Iteration over all components
  for (size_t compIdx = 0; compIdx < sizeIdcs.size(); ++compIdx) {
       
    const auto& idxGroup = sizeIdcs[compIdx];
    std::vector<float> scaledAxes(3, 0.0f);
    bool componentPresent = false;
    float a_x = 0.0f, a_y = 0.0f, a_z = 0.0f;
    float azimuth = bestDescriptionCandidate.bestSemMatrixSigUnRot.at<float>(0, rotationIdcs[compIdx][0]);
    float elevation = bestDescriptionCandidate.bestSemMatrixSigUnRot.at<float>(0, rotationIdcs[compIdx][1]);

    // Iterate over the three principal axes 
    for (size_t axisIdx = 0; axisIdx < idxGroup.size(); ++axisIdx) {
      int idx = idxGroup[axisIdx];
      if (idx >= 0 && idx < bestDescriptionCandidate.bestSemMatrixSigUnRot.cols) {
        float axisRef = bestDescriptionCandidate.bestSemMatrixSigUnRot.at<float>(0, idx);

        if (axisRef == -100.0f) {
          return; // No more compoonents to process
        }

        float axisAbs = axisRef * 2;  // Axis length is double principal axis value

        // Assign to explicit variables
        if (axisIdx == 0) a_z = axisRef;
        else if (axisIdx == 1) a_y = axisRef;
        else if (axisIdx == 2) a_x = axisRef;
        
        LOG4CXX_DEBUG(logger, boost::format(
          "Component %1% | Axis %2% (idx %3%): %4% m")
          % (compIdx + 1)
          % axes[axisIdx]
          % idx
          % (axisAbs));
      }
    }

    // Define the 8 corners of the box in local coordinates
    pcl::PointXYZ corners_local[8] = {
        { a_x,  a_y,  a_z}, { a_x,  a_y, -a_z},
        { a_x, -a_y,  a_z}, { a_x, -a_y, -a_z},
        {-a_x,  a_y,  a_z}, {-a_x,  a_y, -a_z},
        {-a_x, -a_y,  a_z}, {-a_x, -a_y, -a_z}
    };

    // Rotation matrices (object orientation)
    Eigen::AngleAxisf R1(-azimuth,   Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf R2(-elevation, Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f R = (R2 * R1).toRotationMatrix();

    // Pair axes with extents
    std::vector<std::pair<Eigen::Vector3f, float>> axes = {
        {R.col(0), 2*a_x/refCubeScaleFactor},
        {R.col(1), 2*a_y/refCubeScaleFactor},
        {R.col(2), 2*a_z/refCubeScaleFactor},
    };

    // Sort axes by descending length (major → middle → minor)
    std::sort(axes.begin(), axes.end(), [](auto &a, auto &b){
        return a.second > b.second;
    });

    // Enforce right-handed system: minor = major × middle
    axes[2].first = axes[0].first.cross(axes[1].first);

    // Rebuild rotation matrix following PCL convention
    Eigen::Matrix3f R_OBB;
    R_OBB.col(0) = axes[0].first; // major
    R_OBB.col(1) = axes[1].first; // middle
    R_OBB.col(2) = axes[2].first; // minor

    // Rotate corners to world space (OBB)
    std::vector<pcl::PointXYZ> obb_corners;
    obb_corners.reserve(8);

    pcl::PointXYZ min_point(FLT_MAX, FLT_MAX, FLT_MAX);
    pcl::PointXYZ max_point(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    for (int i = 0; i < 8; ++i) {
        Eigen::Vector3f p_world = R_OBB * Eigen::Vector3f(
            corners_local[i].x,
            corners_local[i].y,
            corners_local[i].z
        );
        pcl::PointXYZ corner_world(p_world.x(), p_world.y(), p_world.z());
        obb_corners.push_back(corner_world);

        // Update AABB min/max
        min_point.x = std::min(min_point.x, corner_world.x);
        min_point.y = std::min(min_point.y, corner_world.y);
        min_point.z = std::min(min_point.z, corner_world.z);

        max_point.x = std::max(max_point.x, corner_world.x);
        max_point.y = std::max(max_point.y, corner_world.y);
        max_point.z = std::max(max_point.z, corner_world.z);
    }

    // Compute AABB dimensions
    float aabb_x  = (max_point.x - min_point.x) / refCubeScaleFactor;
    float aabb_y = (max_point.y - min_point.y) / refCubeScaleFactor;
    float aabb_z  = (max_point.z - min_point.z) / refCubeScaleFactor;

    // Print OBB corners
    //std::cout << "OBB corners (world coordinates):" << std::endl;
    //for (int i = 0; i < 8; ++i) {
    //    std::cout << i << ": "
    //              << obb_corners[i].x << " "
    //              << obb_corners[i].y << " "
    //              << obb_corners[i].z << std::endl;
    //}

    LOG4CXX_INFO(logger, boost::format(
        "Component %1% dimensions (x->y->z)): %2% x %3% x %4% m")
        % (compIdx + 1)
        % aabb_x
        % aabb_y
        % aabb_z);

    LOG4CXX_INFO(logger, boost::format(
        "Component %1% dimensions (major->middle->minor): %2% x %3% x %4% m")
        % (compIdx + 1)
        % axes[0].second
        % axes[1].second
        % axes[2].second);
  }
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
  cv::max(objectDepthImageNorm, 0, objectDepthImageNorm);

  //cv::imwrite("/home/schneiders/Desktop/cutOutObject.tiff", objectDepthImageNorm);

  if (objectDepthImageNorm.type() != CV_32FC1) {
    objectDepthImageNorm.convertTo(objectDepthImageNorm, CV_32FC1);
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

  // Sort countours by area and get the largest one
  size_t largestContourIdx = 0;
  double maxArea = 0.0;
  for (size_t i = 0; i < contours.size(); ++i) {
    double area = cv::contourArea(contours[i]);
    if (area > maxArea) {
      maxArea = area;
      largestContourIdx = i;
    }
  }
  cv::Rect objectBoundingBox = cv::boundingRect(contours[largestContourIdx]);

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

  float ax = semMat.at<float>(0, sideCompIdcs[compIndex - 1][5]);
  float ay = semMat.at<float>(0, sideCompIdcs[compIndex - 1][6]);
  float az = semMat.at<float>(0, sideCompIdcs[compIndex - 1][7]);

  float sum = ax + ay + az;
  LOG4CXX_DEBUG(logger, boost::format("[isSideComponentPresent] compIdx: %d Sum: %f Threshold: %f") % compIndex % sum % existenceThreshold);
  // Component must have a large enough volume in order to be considered present
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
    case 0:
    case 1:
    case 2:
      break;
    //we look for base component shape predicates
    case 3:
    case 4:
    case 5:
    case 6:
      compIndex = 0;
      shapePredicate = true;
      break;
    //we look for side component 1 shape predicates
    case 10:
    case 11:
    case 12:
    case 13:
      compIndex = 1;
      shapePredicate = true;
      break;
    //we look for side component 2 shape predicates
    case 29:
    case 30:
    case 31:
    case 32:
      compIndex = 2;
      shapePredicate = true;
      break;
    //we look for side component 1 position predicates
    case 14:
    case 15:
    case 16:
    case 17:
    case 18:
    case 19:
    case 20:
    case 21:
    case 22:
      compIndex = 1;
      positionPredicate = true;
      break;
    //we look for side component 2 position predicates
    case 33:
    case 34:
    case 35:
    case 36:
    case 37:
    case 38:
    case 39:
    case 40:
    case 41:
      compIndex = 2;
      positionPredicate = true;
      break;
    //we look for side component 1 size predicates
    case 23:
    case 24:
    case 25:
      compIndex = 1;
      sizePredicate = true;
      break;
    //we look for side component 2 size predicates
    case 42:
    case 43:
    case 44:
      compIndex = 2;
      sizePredicate = true;
      break;
    //we look for eccentricity predicates
    case 7:
    case 26:
    case 45:
      eccentricityPredicate = true;
      break;
    //we look for base component orientation predicates
    case 8:
    case 9:
      compIndex = 0;
      orientationPredicate = true;
      break;
    //we look for side component 1 orientation predicates
    case 27:
    case 28:
      compIndex = 1;
      orientationPredicate = true;
      break;
    //we look for side component 2 orientation predicates
    case 46:
    case 47:
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
    case 2:
      return isBlock(n1, n2);
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

bool ComponentValidator::isBlock(const float n1, const float n2) {

  bool isn1Low = n1 < 0.4;
  bool isn2Low = n2 < 0.4;
  
  return isn1Low && isn2Low;
}

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

  return semPositionMatch;
}

bool ComponentValidator::processSize(const int compIndex, const cv::Mat &semMat, const int semPredIdx) {

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

  return semSizeMatch;
}

bool ComponentValidator::processEccentricity(const cv::Mat &semMat, const int semPredIdx) {

  int compIndex;

  switch (semPredIdx) {
    //we look for base component eccentricity
    case 7:
      compIndex = 0;
      break;
      //we look for side component 1 eccentricity
    case 26:
      compIndex = 1;
      break;
      //we look for side component 2 eccentricity
    case 45:
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
    if (semPredIdx == 8 || semPredIdx == 27 || semPredIdx == 46) {
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
    else if (semPredIdx == 9 || semPredIdx == 28 || semPredIdx == 47) {
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

cv::Mat ComponentValidator::applySigmoidToMatrix(const cv::Mat &matrix) {
  // Check if the matrix is not empty and has the correct dimensions
  if (!matrix.empty() && matrix.cols == 30 && matrix.rows == 1) {
    // Create a new matrix with 3 additional columns (24 + 3 = 27)
    cv::Mat matrixSig = cv::Mat::zeros(1, matrix.cols, CV_32F);
    // Copy original matrix values to the new matrix, starting after the 3 initial zeros
    for (int i = 0; i < matrix.cols; ++i) {
      matrixSig.at<float>(0, i) = sigmoid(matrix.at<float>(0, i));
    }
    return matrixSig; // Return the new matrix with prepended zeros and sigmoid applied
  } else {
    LOG4CXX_DEBUG(logger, "[ComponentValidation] Invalid semantic matrix dimensions ");
    return cv::Mat(); // Return an empty matrix to indicate an error
  }
}

// Generate a full text description based on the identified predicates
void ComponentValidator::populateMemoryObjectDescription(std::vector<bool> foundPredicatesVector, MemoryObject::Ptr mo) {

  // Helper lambdas to determine shape and size
  auto getShape = [&](int rodIndex, int diskIndex, int blockIndex) -> std::string {
    if (foundPredicatesVector[rodIndex]) return "rod";
    if (foundPredicatesVector[diskIndex]) return "disk";
    if (foundPredicatesVector[blockIndex]) return "block";
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
  std::string eccentricityCentral = getEccentricity(7);
  std::string orientationCentral = getOrientation(8, 9);
  std::string shapeCentral = getShape(3, 4, 5);

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
    std::string sizeSecond = getSize(23, 24, 25);
    std::string eccentricitySecond = getEccentricity(26);
    std::string orientationSecond = getOrientation(27, 28);
    std::string shapeSecond = getShape(10, 11, 12);
    std::string positionsSecond = getPosition({14, 15, 16, 17, 18, 19});

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
    std::string sizeSecond = getSize(23, 24, 25);
    std::string eccentricitySecond = getEccentricity(26);
    std::string orientationSecond = getOrientation(27, 28);
    std::string shapeSecond = getShape(10, 11, 12);
    std::string positionsSecond = getPosition({14, 15, 16, 17, 18, 19});

    std::string sizeThird = getSize(42, 43, 44);
    std::string eccentricityThird = getEccentricity(45);
    std::string orientationThird = getOrientation(46, 47);
    std::string shapeThird = getShape(29, 30, 31);
    std::string positionsThird = getPosition({33, 34, 35, 36, 37, 38});

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
std::string ComponentValidator::generateFullDescription(std::vector<bool> foundPredicatesVector, float chamferDistance) {
  
  std::string lowConfidenceStatement = ", but I have low confidence in my description.";
  
  // Helper lambdas to determine shape and size
  auto getShape = [&](int rodIndex, int diskIndex, int blockIndex) -> std::string {
    if (foundPredicatesVector[rodIndex]) return "rod";
    if (foundPredicatesVector[diskIndex]) return "disk";
    if (foundPredicatesVector[blockIndex]) return "block";
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

  // Central component
  std::string eccentricityCentral = getEccentricity(7);
  std::string orientationCentral = getOrientation(8, 9);
  std::string shapeCentral = getShape(3, 4, 5);

  // Object with one component
  if (foundPredicatesVector[0]) {
    LOG4CXX_DEBUG(logger, boost::format("Generating 1-component object description"));
    std::string description = ", which is a " + eccentricityCentral + " " + orientationCentral + " " + shapeCentral;

    // Add low confidence statement only if Chamfer distance is below threshold
    if (chamferDistance > chamferTolerance) {
      description += lowConfidenceStatement;
    } else {
      description += ". ";
    }

    return description;
  }

  // Object with two components
  if (foundPredicatesVector[1]) {
    LOG4CXX_DEBUG(logger, boost::format("Generating 2-component object description"));

    std::string sizeSecond = getSize(23, 24, 25);
    std::string eccentricitySecond = getEccentricity(26);
    std::string orientationSecond = getOrientation(27, 28);
    std::string shapeSecond = getShape(10, 11, 12);
    std::string positionsSecond = getPosition({14, 15, 16, 17, 18, 19});

    std::string description = ", which consists of a " + eccentricityCentral + " " + orientationCentral + " " + shapeCentral +
           " and a" + sizeSecond + " " + eccentricitySecond + " " + orientationSecond + " " + shapeSecond + " " + positionsSecond;

    if (chamferDistance > chamferTolerance) {
      description += lowConfidenceStatement;
    } else {
      description += ". ";
    }
   
    return description;
  }
  
  // Object with three components
  if (foundPredicatesVector[2]) {
    LOG4CXX_DEBUG(logger, boost::format("Generating 3-component object description"));

    std::string sizeSecond = getSize(23, 24, 25);
    std::string eccentricitySecond = getEccentricity(26);
    std::string orientationSecond = getOrientation(27, 28);
    std::string shapeSecond = getShape(10, 11, 12);
    std::string positionsSecond = getPosition({14, 15, 16, 17, 18, 19});

    std::string sizeThird = getSize(42, 43, 44);
    std::string eccentricityThird = getEccentricity(45);
    std::string orientationThird = getOrientation(46, 47);
    std::string shapeThird = getShape(29, 30, 31);
    std::string positionsThird = getPosition({33, 34, 35, 36, 37, 38});

    std::string description = " , which consists of a " + eccentricityCentral + " " + orientationCentral + " central " + shapeCentral +
           " with one " + sizeSecond + " " + eccentricitySecond + " " + orientationSecond + " " + shapeSecond + " " + positionsSecond +
           " and a " + sizeThird + " " + eccentricityThird + " " + orientationThird + " " + shapeThird + " " + positionsThird;

  // Add low confidence statement only if Chamfer distance is below threshold
    if (chamferDistance > chamferTolerance) {
      description += lowConfidenceStatement;
    } else {
      description += " . ";
    }
    return description;
  }
  
  // If no proper description
  return "No proper description generated";
}
