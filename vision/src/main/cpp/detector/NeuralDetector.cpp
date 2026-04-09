/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "NeuralDetector.hpp"

#include <jsoncpp/json/reader.h>

#include "common/notification/CaptureNotification.hpp"
#include "display/Display.hpp"
#include "stm/util/StmUtilities.hpp"
#include <experimental/filesystem>
#include <fstream>

namespace fs = std::experimental::filesystem;
using namespace diarc::stm;

void NeuralDetector::loadConfig(const std::string &configFile) {
  LOG4CXX_INFO(logger, boost::format("Loading config file: %s") % configFile.c_str());
  //get directory
  fs::path homedir(getenv("HOME"));
  fs::path diarc(".diarc");
  fs::path vision("vision");
  fs::path detectors("detectors");

  unsigned found = configFile.find_last_of("/\\");
  std::string path = configFile.substr(0, found);
  found = path.find_last_of("/\\");
  fs::path p(path.substr(found+1, path.length()));

  fs::path dir = homedir / diarc / vision / detectors / p;
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
  int arity = root["processor"]["arity"].asInt();
  for (auto predicateValue: root["predicates"]) {
    std::string name = predicateValue["name"].asString();
    int classID = predicateValue["classID"].asInt();
    if (classID < 0) {
      continue;
    }
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
  logger = log4cxx::Logger::getLogger("diarc.detector.NeuralDetector");
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

    LOG4CXX_INFO(logger, boost::format("Using darknet config: %s") % dn_config_abs.string());
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
  const cv::Mat &img = notification->captureData->frame;
  std::vector<int> class_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  cv::Mat blob;

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

        boxes.push_back(rect);
        confidences.push_back(confidence);
        class_ids.push_back(classIdx);
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
          boxes.push_back(rect);
          confidences.push_back(confidence);
          class_ids.push_back(classIdPoint.x);
        }
      }
    }
  } else if (outLayerType == "Identity" || outLayerType == "Concat") {
    int net_height = detections[0].size[1];
    int net_width = classNames.size() + 5;
    int net_out_width = detections[0].size[2];
    float *data = (float *) detections[0].data;
    float x_factor = img.cols / 640.0;
    float y_factor = img.rows / 640.0;
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

          boxes.push_back(rect);
          confidences.push_back(confidence);
          class_ids.push_back(classIdPoint.x);
        }
      }
      data += net_width;
    }
  }


  // // Non-maximum suppression
  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, 0.0, nms_thresh, indices);
  LOG4CXX_DEBUG(logger, boost::format("Num boxes after non-max suppression: %d") % indices.size());

  std::vector<DetectedObject> objects;
  for (int idx: indices) {
    DetectedObject currObj;
    currObj.rect = boxes[idx];
    currObj.confidence = confidences[idx];
    currObj.name = idToClass[class_ids[idx]];
    objects.push_back(currObj);
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
    if (diarc::stm::util::calculateBoundBoxOverlap(rectA, rectB) >= threshold) {
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
        if (diarc::stm::util::calculateBoundBoxOverlap(rectA, rectB) >= threshold) {
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
  std::vector<DetectedObject> detections = getDetections(notification);

  // display all class detections
  if (getDisplayFlag()) {
    const cv::Mat &inputFrame = notification->captureData->frame;
    inputFrame.copyTo(displayFrame);
    for (const auto &detected_object : detections) {
        const cv::Rect &rect = detected_object.rect;
        std::string label = detected_object.name + " " + std::to_string(detected_object.confidence);
        cv::rectangle(displayFrame, cv::Point(rect.x, rect.y),
                      cv::Point(rect.x + rect.width, rect.y + rect.height),
                      CV_RGB(255, 0, 0), 2, 8, 0);
        cv::putText(displayFrame, label, cv::Point(rect.x, rect.y+rect.height), cv::FONT_HERSHEY_SIMPLEX,
                    0.8, CV_RGB(255, 255, 0), 1);
      }

    diarc::Display::displayFrame(displayFrame, getDisplayName());
  }

  // get current descriptors to process
  TypesByDescriptorConstPtr descriptors = getDescriptors();
  TypesByDescriptor::const_iterator descriptor_iter;

  std::tr1::unordered_set<long long>::const_iterator typeIds_itr;
  MemoryObject::VecPtr newObjs(new MemoryObject::Vec());
  for (descriptor_iter = descriptors->begin(); descriptor_iter != descriptors->end(); ++descriptor_iter) {
    bool useAllResults = descriptor_iter->first.getName() == "all";
    int target_class_id = classToId[descriptor_iter->first.getName()];

    for (const auto &detected_object: detections) {
      int class_id = classToId[detected_object.name];
      if (class_id == target_class_id || useAllResults) {

        // create MemoryObject for each relevant typeId
        for (typeIds_itr = descriptor_iter->second.begin(); typeIds_itr != descriptor_iter->second.end(); ++typeIds_itr) {

          MemoryObject::Ptr newObj(
                  new MemoryObject(*typeIds_itr, descriptor_iter->first.getArg(0), notification->captureData,
                                   detected_object.rect));
          if (useAllResults) {
            // add "class_label(VAR)" and "all(VAR)"
            newObj->addValidationResult(detected_object.confidence, PredicateHelper(
                    detected_object.name + "(" + descriptor_iter->first.getArg(0) + ")"));
            newObj->addValidationResult(detected_object.confidence, descriptor_iter->first);
          } else {
            // add "class_label(VAR)"
            newObj->addValidationResult(detected_object.confidence, descriptor_iter->first);
          }
          newObjs->push_back(newObj);

        }
      }
    }
  }

  sendDetectionNotifications(newObjs);

  // draw detection bounding boxes
  if (getDisplayFlag()) {
    // inputFrame.copyTo(displayFrame);
    for (auto newObjIter = newObjs->begin(); newObjIter != newObjs->end(); ++newObjIter) {
      const cv::Rect &objRect = (*newObjIter)->getDetectionMask()->getBoundingBox();
      std::string label = (*newObjIter)->getValidationResults().getDescriptorsString();
      int lineThickness = 2;
      cv::rectangle(displayFrame, cv::Point(objRect.x+lineThickness, objRect.y+lineThickness),
                    cv::Point(objRect.x + objRect.width - lineThickness, objRect.y + objRect.height - lineThickness),
                    CV_RGB(0, 0, 255), lineThickness, 8, 0);
    }

  diarc::Display::displayFrame(displayFrame, getDisplayName());
  }
}

// alters current frame so that it can be processed by the model
cv::Mat NeuralDetector::prepFrame(const cv::Mat& img) {
  cv::Mat currFrame;
  cv::resize(img, currFrame, cv::Size(640, 640));
  cv::cvtColor(currFrame, currFrame, cv::COLOR_BGR2RGB);

  return currFrame;
}

size_t NeuralDetector::vectorProduct(std::vector<int64_t> vector) {
  if (vector.empty()) {
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
  for (const auto & detectedObject : detectedObjects) {
    const cv::Rect &objRect = detectedObject.rect;
    cv::rectangle(imgToDraw, cv::Point(objRect.x, objRect.y),
    cv::Point(objRect.x + objRect.width, objRect.y + objRect.height),
    CV_RGB(255, 0, 0), 2, 8, 0);
    cv::putText(imgToDraw, detectedObject.name, cv::Point(objRect.x + objRect.width, objRect.y + objRect.height),
    cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 0, 0));
  }
  diarc::Display::createWindowIfDoesNotExist(displayName);
  diarc::Display::displayFrame(imgToDraw, displayName);
}

// fills classNames vector first with placeholder names
// then at every known id fill with relevant classname
void NeuralDetector::fillClassNames(int totalClasses) {
  for (int i = 0; i < totalClasses; i++) {
    classNames.push_back("invalid");
  }

  for (const auto& idThenClass : idToClass) {
    classNames[idThenClass.first] = idThenClass.second;
  }
}
