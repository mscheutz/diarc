/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "NeuralDetector.hpp"

#include <jsoncpp/json/reader.h>

#include "capture/util/CaptureUtilities.hpp"
#include "common/notification/CaptureNotification.hpp"
#include "display/Display.hpp"
#include "stm/util/StmUtilities.hpp"
#include <experimental/filesystem>
#include <fstream>
#include <iostream>

namespace fs = std::experimental::filesystem;
using namespace ade::stm;

void NeuralDetector::loadConfig(const std::string &configFile) {
  //get directory
  fs::path homedir(getenv("HOME"));
  fs::path diarc(".diarc");
  fs::path vision("vision");
  fs::path detectors("detectors");
  fs::path p(configFile);
  fs::path dir = homedir / diarc / vision / detectors / p.stem();
  std::string backEnd;
  std::string type;
  uint64_t totalClasses;

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

  // dnn configuration options
  Json::Value config = root["config"];
  fs::path model_file(config["model"].asString());
  fs::path model_path = dir / model_file;
  inputSize = config["input_size"].asInt64();
  backEnd = config["backend"].asString();
  confidence_thresh = config["conf_thresh"].asDouble();
  score_thresh = config["score_thresh"].asDouble();
  nms_thresh = config["nms_thresh"].asDouble();
  type = config["type"].asString();
  totalClasses = config["classes"].asInt64();

  fillClassNames(totalClasses);

  NeuralDetector::initModel(model_path.string(), backEnd, type, configFile);
}

NeuralDetector::NeuralDetector(const long long &processorId, const int imgWidth, const int imgHeight)
        : ObjectDetector(processorId, imgWidth, imgHeight) {
  visionProcessName = "NeuralDetector";
  logger = log4cxx::Logger::getLogger("ade.detector.NeuralDetector");
}

NeuralDetector::~NeuralDetector() {}

void NeuralDetector::initModel(const std::string &model_path, const std::string &backEnd, const std::string &type, const std::string &configFile ) {
  if (type == "darknet") {
#if defined(OPENCV_DNN) || defined(OPENCV3_DNN)
    fs::path dn_model_abs = fs::canonical(model_path + ".weights");

    fs::path dn_config_file(configFile);
    fs::path dn_config_fn(dn_model_abs.filename().stem().string() + ".cfg");
    fs::path dn_config_dir(dn_config_file.parent_path().string());
    fs::path dn_config = dn_config_dir / dn_config_fn;
    fs::path dn_config_abs = fs::canonical(dn_config);

    LOG4CXX_INFO(logger, boost::format("Using darknet config: %s") % dn_config_abs.string())
    LOG4CXX_INFO(logger, boost::format("Loading darknet model: %s") % dn_model_abs.string());
    net = cv::dnn::readNetFromDarknet(dn_config_abs.string(), dn_model_abs.string());
#else
    LOG4CXX_ERROR(logger, "Cannot load darknet model with this OpenCV version. Must be 3.4+");
#endif
  } else if (type == "onnx") {
#if defined(OPENCV_DNN)
    fs::path onnx_model_abs = fs::canonical(model_path + ".onnx");
    LOG4CXX_INFO(logger, boost::format("Loading ONNX model: %s") % onnx_model_abs.string());
    net = cv::dnn::readNetFromONNX(onnx_model_abs.string());
#else
    LOG4CXX_ERROR(logger, "Cannot load ONNX model with this OpenCV version. Must be 4.5+");
#endif
  }
  LOG4CXX_INFO(logger, "Model loaded.");

  setBackendAndTarget();
  
  out_layer_names = net.getUnconnectedOutLayersNames();
}

//determine if cuDNN is installed and use CUDA, otherwise default to CPU
void NeuralDetector::setBackendAndTarget () {
  int backend = cv::dnn::DNN_BACKEND_OPENCV;
  int target = cv::dnn::DNN_TARGET_CPU;

#if defined(OPENCV_DNN) && defined(CUDNN)
  auto listBackends = cv::dnn::getAvailableBackends();
  for (const auto& be : listBackends) {
    if (backend == cv::dnn::DNN_BACKEND_OPENCV && namesBackend[be.first] == "DNN_BACKEND_CUDA") {
      backend = cv::dnn::DNN_BACKEND_CUDA;
      LOG4CXX_INFO(logger, "Backend: DNN_BACKEND_CUDA");
    }
    if (target == cv::dnn::DNN_TARGET_CPU && namesTarget[be.second] == "DNN_TARGET_CUDA") {
      target = cv::dnn::DNN_TARGET_CUDA;
      LOG4CXX_INFO(logger, "Target: DNN_TARGET_CUDA");
    }
  }

  if (backend != cv::dnn::DNN_BACKEND_OPENCV && target != cv::dnn::DNN_TARGET_CPU) {
    int numDevices = cv::cuda::getCudaEnabledDeviceCount();
    LOG4CXX_INFO(logger, boost::format("Number of CUDA devices: %s") %  numDevices);
    if (numDevices == 0) {
      backend = cv::dnn::DNN_BACKEND_OPENCV;
      target = cv::dnn::DNN_TARGET_CPU;
      LOG4CXX_INFO(logger, "No CUDA devices found, reverting to CPU");
    }
  }
#endif

  net.setPreferableBackend(backend);
  net.setPreferableTarget(target);
}

// get detections from current frame and return as a vector of detectedObjects
std::vector<NeuralDetector::DetectedObject> NeuralDetector::getDetections(CaptureNotification::ConstPtr notification) {
  LOG4CXX_DEBUG(logger, "called getDetections");
  std::vector<DetectedObject> objects;
  const cv::Mat &img = notification->captureData->frame;
  int origRows = img.rows;
  int origCols = img.cols;
  std::vector<int> class_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  cv::Mat blob;
  float x_factor = origCols / inputSize;
  float y_factor = origRows / inputSize;

  //necessary?
  cv::Mat currFrame = prepFrame(img);

  // forward pass
  std::vector<cv::Mat> detections;
  cv::dnn::blobFromImage(currFrame, blob, 1./255., cv::Size(inputSize, inputSize), cv::Scalar(), true, false, CV_32F);
  net.setInput(blob);
  static std::vector<std::string> outLayers = net.getUnconnectedOutLayersNames(); 
  static std::string outLayerType = net.getLayer(outLayers[0])->type;
  net.forward(detections, outLayers);

  LOG4CXX_DEBUG(logger, boost::format("Output layer type: %s") % outLayerType);
  if (outLayerType == "DetectionOutput") {
    // Network produces output blob with a shape 1x1xNx7 where N is a number of
    // detections and an every detection is a vector of values
    // [batchId, classId, confidence, left, top, right, bottom]]
    
    for (size_t k = 0; k < detections.size(); k++) {

      float* data = (float*) detections[k].data;
      uint64_t found = 0;
      uint64_t total = 0;
      for (int i = 0; i < detections[k].total(); i+=7) {
        float confidence = data[i + 2];
        total++;
        //LOG4CXX_INFO(logger, "looping through detections"); // never gets here
        if (confidence < confidence_thresh) {
          continue;
        }
        found++;
        //LOG4CXX_INFO(logger, confidence);

        int classIdx = (int)(data[i + 1]) - 1;
        std::string class_id = idToClass[classIdx];
        //double max_class_score;
        //cv::minMaxLoc(class_scores, 0, &max_class_score, 0, &class_id);
        //if (max_class_score < score_thresh) {
        //  continue;
        //}

        LOG4CXX_DEBUG(logger, boost::format("Box num: %d. Confidence: %f. ClassId: %s.") % i % confidence % class_id);
        int left   = (int)data[i + 3];
        int top    = (int)data[i + 4];
        int right  = (int)data[i + 5];
        int bottom = (int)data[i + 6];
        int width  = right - left + 1;
        int height = bottom - top + 1;
        if (width <= 2 || height <= 2) {
            left   = (int)(data[i + 3] * img.cols);
            top    = (int)(data[i + 4] * img.rows);
            right  = (int)(data[i + 5] * img.cols);
            bottom = (int)(data[i + 6] * img.rows);
            width  = right - left + 1;
            height = bottom - top + 1;
        }
        cv::Rect rect(left, top, width, height);

        DetectedObject currObj;
        currObj.rect = rect;
        currObj.confidence = confidence;
        currObj.name = class_id;
        objects.push_back(currObj);
      }

      LOG4CXX_INFO(logger, boost::format("Found: %d / %d") % found % total);
    }
  } else if (outLayerType == "Region") {
    for (size_t i = 0; i < detections.size(); i++) {
      // Network produces output blob with a shape NxC where N is a number of
      // detected objects and C is a number of classes + 4 where the first 4
      // numbers are [center_x, center_y, width, height]
      float* data = (float*)detections[i].data;
      for (int j = 0; j < detections[i].rows; j++, data += detections[i].cols) {
        cv::Mat scores = detections[i].row(j).colRange(5, detections[i].cols);
        cv::Point classIdPoint;
        double confidence;

        minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
        if (confidence >= confidence_thresh) {
          int centerX = (int)(data[0] * img.cols);
          int centerY = (int)(data[1] * img.rows);
          int width = (int)(data[2] * img.cols);
          int height = (int)(data[3] * img.rows);
          int left = centerX - width / 2;
          int top = centerY - height / 2;
          
          std::string class_id = idToClass[classIdPoint.x];
          cv::Rect rect(left, top, width, height);
          LOG4CXX_DEBUG(logger, boost::format("Box num: %d. Confidence: %f. ClassId: %s.") % j % confidence % class_id);
          DetectedObject currObj;
          currObj.rect = rect;
          currObj.confidence = confidence;
          currObj.name = class_id;
          objects.push_back(currObj);
        }
      }
    }
  } else if (outLayerType == "Identity") {
    int net_height = detections[0].size[1];
    int net_width = classNames.size() + 5;
    int net_out_width = detections[0].size[2];
    float score_thresh = 0.2;
    float *data = (float *) detections[0].data;
    float x_factor = img.cols / 640.0;
    float y_factor = img.rows / 640.0;
    int num_classes = classNames.size() + 5;
    LOG4CXX_DEBUG(logger, boost::format("Net width: %d. Net output width: %d.") % net_width % net_out_width);
    for (int i = 0; i < net_height; ++i) {
      float confidence = data[4];
      if (confidence >= confidence_thresh) {
        cv::Mat scores(1, classNames.size(), CV_32FC1, data + 5);
        cv::Point classIdPoint;
        double max_class_score;
        minMaxLoc(scores, 0, &max_class_score, 0, &classIdPoint);
        max_class_score = max_class_score * confidence;
        if (max_class_score > confidence_thresh) {
          float x = data[0];
          float y = data[1];
          float w = data[2];
          float h = data[3];
          int left = int((x - 0.5 * w) * x_factor);
          int top = int((y - 0.5 * h) * y_factor);
          int width = int(w * x_factor);
          int height = int(h * y_factor);
          std::string class_id = idToClass[classIdPoint.x];
          cv::Rect rect(left, top, width, height);

          LOG4CXX_DEBUG(logger, boost::format("Box num: %d. Confidence: %f. ClassId: %d.") % i % confidence % classIdPoint.x);
          DetectedObject currObj;
          currObj.rect = rect;
          currObj.confidence = confidence;
          currObj.name = class_id;
          objects.push_back(currObj);
        }
      }
      data += net_width;
    }
  }

  return objects;
}

// removes duplicates detections from detections vector
// detection qualifies as a duplicate if threshold% of its bounding box is within another bounding box
void NeuralDetector::removeDuplicates(std::vector<DetectedObject> &objects, float threshold) {
  bool finished = false;

  if (objects.size() <= 1) return;
  if (objects.size() == 2) {
    cv::Rect rectA = objects[0].rect;
    cv::Rect rectB = objects[1].rect;
    if (ade::stm::util::calculateBoundBoxOverlap(rectA, rectB) >= threshold) {
      if (objects[0].confidence > objects[1].confidence) {
        objects.erase(objects.begin()+1);
      } else {
        objects.erase(objects.begin()+0);
      }
    }
    return;
  }

  while (!finished) {
    for (int i = 0; i < objects.size() - 1; i++) {
      cv::Rect rectA = objects[i].rect;
      bool shouldBreak = false;
      for (int j = 0; j < objects.size(); j++) {
        if (i == j) continue;
        cv::Rect rectB = objects[j].rect;
        if (ade::stm::util::calculateBoundBoxOverlap(rectA, rectB) >= threshold) {
          if (objects[i].confidence > objects[j].confidence) {
            objects.erase(objects.begin()+j);
          } else {
            objects.erase(objects.begin()+i);
          }
          shouldBreak = true;
          break;
        }
      }
      if (i == objects.size() - 2) {
        finished = true;
      } else if (shouldBreak) {
        break;
      }
    }
  }
  if (objects.size() == 2) { // TODO: actually fix this case
    removeDuplicates(objects, threshold);
  }
}

void NeuralDetector::handleCaptureNotification(CaptureNotification::ConstPtr notification) {
  //get captured frames info
  const cv::Mat &inputFrame = notification->captureData->frame;
  long min_size = std::min(inputFrame.cols, inputFrame.rows);
  cv::Mat currFrame;
  cv::resize(inputFrame, currFrame, cv::Size((long) (inputFrame.cols * inputSize / min_size) + 1,
                                             (long) (inputFrame.rows * inputSize / min_size) + 1));
  // frame: 320 by 240
  cv::Mat blob;
  cv::dnn::blobFromImage(currFrame, blob, 0.00392, cv::Size(inputSize, inputSize), cv::Scalar(), true, false, CV_32F);

  //Perform Yolo Detection
  net.setInput(blob);
  std::vector<cv::Mat> detections;
  net.forward(detections, out_layer_names);

  std::vector<cv::Rect> boxes;
  std::vector<float> confidences;
  std::vector<int> class_ids;
  for (auto &output: detections) {
    // each row contains results for a single bounding box [x,y,w,h,conf,<score for each class>]
    for (int i = 0; i < output.rows; i++) {
      auto confidence = output.at<float>(i, 4);
      if (confidence < confidence_thresh) {
        continue;
      }

      // find class with max score (for this row)
      // Create a 1xN(num classes) Mat and store class scores of classes.
      cv::Mat class_scores(1, output.cols - 5, CV_32FC1, output.ptr<float>(i, 5));
      // Perform minMaxLoc and acquire the index of best class  score.
      cv::Point class_id;
      double max_class_score;
      cv::minMaxLoc(class_scores, 0, &max_class_score, 0, &class_id);
      if (max_class_score < score_thresh) {
        continue;
      }

      LOG4CXX_DEBUG(logger,
                   boost::format("Box num: %d. Confidence: %f. Class score: %f. ClassId: %d.") % i % confidence %
                   max_class_score % class_id.x);

      float x = output.at<float>(i, 0) * inputFrame.cols;
      float y = output.at<float>(i, 1) * inputFrame.rows;
      float width = output.at<float>(i, 2) * inputFrame.cols;
      float height = output.at<float>(i, 3) * inputFrame.rows;
      cv::Rect rect(x - width / 2, y - height / 2, width, height);
      boxes.push_back(rect);
      confidences.push_back(confidence);
      class_ids.push_back(class_id.x);
    }
  }

  // Non-maximum suppression
  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, 0.0, nms_thresh, indices);
  LOG4CXX_DEBUG(logger, boost::format("Num boxes after non-max suppression: %d") % indices.size());

  // get current descriptors to process
  TypesByDescriptorConstPtr descriptors = getDescriptors();
  TypesByDescriptor::const_iterator descriptor_iter;

  std::tr1::unordered_set<long long>::const_iterator typeIds_itr;
  MemoryObject::VecPtr newObjs(new MemoryObject::Vec());
  for (descriptor_iter = descriptors->begin(); descriptor_iter != descriptors->end(); ++descriptor_iter) {
    bool useAllResults = (descriptor_iter->first.getName() == parentType);
    int target_class_id = classToId[descriptor_iter->first.getName()];

    for (int idx: indices) {
      if (class_ids[idx] == target_class_id || useAllResults) {
        std::string class_label = idToClass[class_ids[idx]];

        // create MemoryObject for each relevant typeId
        for (typeIds_itr = descriptor_iter->second.begin(); typeIds_itr != descriptor_iter->second.end(); ++typeIds_itr) {
          MemoryObject::Ptr newObj(
                  new MemoryObject(*typeIds_itr, descriptor_iter->first.getArg(0), notification->captureData,
                                   boxes[idx]));
          newObj->addValidationResult(confidences[idx], descriptor_iter->first);
          if (useAllResults) {
            // also add "class_label(VAR)"
            newObj->addValidationResult(confidences[idx], PredicateHelper(
                    class_label + "(" + descriptor_iter->first.getArg(0) + ")"));
          }
          newObjs->push_back(newObj);

        }
      }
    }
  }

  sendDetectionNotifications(newObjs);

  // draw face bounding boxes
  if (getDisplayFlag()) {
    inputFrame.copyTo(displayFrame);
    MemoryObject::Vec::const_iterator newObjItr;
    for (auto newObjIter = newObjs->begin(); newObjIter != newObjs->end(); ++newObjIter) {
      const cv::Rect &objRect = (*newObjIter)->getDetectionMask()->getBoundingBox();
      std::string label = (*newObjIter)->getValidationResults().getDescriptorsString();
      cv::rectangle(displayFrame, cv::Point(objRect.x, objRect.y),
                    cv::Point(objRect.x + objRect.width, objRect.y + objRect.height),
                    CV_RGB(255, 0, 0), 2, 8, 0);
      cv::putText(displayFrame, label, cv::Point(objRect.x, objRect.y), cv::FONT_HERSHEY_SIMPLEX,
                  0.8, CV_RGB(0, 255, 0), 1);
    }

    ade::Display::displayFrame(displayFrame, getDisplayName());
  }
}

// alters current frame so that it can be processed by the model
cv::Mat NeuralDetector::prepFrame(cv::Mat img) {
  cv::Mat currFrame;
  cv::resize(img, currFrame, cv::Size(640, 640));
  cv::cvtColor(currFrame, currFrame, cv::COLOR_BGR2RGB);

  return currFrame;
}

size_t NeuralDetector::vectorProduct(std::vector<int64_t> vector) {
  if (vector.size() == 0) {
    return 0;
  }

  size_t product = 1;
  for (int64_t i : vector) {
    product *= i;
  }
  return product;
}

// calculates top left and bottom right given iterator with
// center coordinates and dimensions
std::vector<cv::Point> NeuralDetector::getTLandBR(std::vector<float>::iterator it) {
  int centerX = (int) (it[0]);
  int centerY = (int) (it[1]);
  int width = (int) (it[2]);
  int height = (int) (it[3]);
  int left = centerX - width / 2;
  int top = centerY - height / 2;

  cv::Point topLeft = cv::Point(left, top);
  cv::Point bottomRight = cv::Point(left + width, top + height);

  std::vector<cv::Point> points;
  points.push_back(topLeft);
  points.push_back(bottomRight);

  return points;
}

void NeuralDetector::displayDetectedObjects(cv::Mat& imgToDraw, const std::vector<NeuralDetector::DetectedObject>& detectedObjects, const std::string& displayName) {
  // display detections
  for (auto itr = detectedObjects.begin(); itr != detectedObjects.end(); ++itr) {
    const cv::Rect &objRect = (*itr).rect;
    cv::rectangle(imgToDraw, cv::Point(objRect.x, objRect.y),
    cv::Point(objRect.x + objRect.width, objRect.y + objRect.height),
    CV_RGB(255, 0, 0), 2, 8, 0);
    cv::putText(imgToDraw, (*itr).name, cv::Point(objRect.x + objRect.width, objRect.y + objRect.height),
    cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 0, 0));
  }
  ade::Display::createWindowIfDoesNotExist(displayName);
  ade::Display::displayFrame(imgToDraw, displayName);
}

// fills classNames vector first with placeholder names
// then at every known id fill with relevant classname
void NeuralDetector::fillClassNames(int totalClasses) {
  for (int i = 0; i < totalClasses; i++) {
    classNames.push_back("invalid");
  }

  for (auto idThenClass : idToClass) {
    classNames[idThenClass.first] = idThenClass.second;
  }
}
