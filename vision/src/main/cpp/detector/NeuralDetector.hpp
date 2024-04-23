/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef NEURALDETECTOR_HPP
#define NEURALDETECTOR_HPP

#include "ObjectDetector.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>
#include <unordered_map>

class NeuralDetector : public ObjectDetector {
public:
  typedef boost::shared_ptr<NeuralDetector> Ptr;
  typedef boost::shared_ptr<const NeuralDetector> ConstPtr;

  struct DetectedObject {
    cv::Rect rect;
    float confidence;
    std::string name;
  };
  std::vector<DetectedObject> getDetections(CaptureNotification::ConstPtr notification);
  void removeDuplicates(std::vector<DetectedObject> &objects, float threshold);

  NeuralDetector(const long long& processorId, const int imgWidth, const int imgHeight);
  virtual ~NeuralDetector();
  virtual void loadConfig(const std::string& configFile);

protected:
  void displayDetectedObjects(cv::Mat& imgToDraw, const std::vector<DetectedObject>& detectedObjects, const std::string& displayName);
  virtual void handleCaptureNotification(CaptureNotification::ConstPtr notification);
  virtual void initModel(const std::string& model_path, const std::string& backEnd, const std::string &type);
  virtual void setBackendAndTarget();

  // semantic parent type for this detector's descriptors (isA hierarchy)
  std::string parentType;

  cv::dnn::Net net;
  double confidence_thresh=0.1;
  double score_thresh=0.1;
  double nms_thresh=0.4;
  long inputSize;
  std::vector<cv::String> out_layer_names;
  std::unordered_map<std::string, int> classToId;
  std::unordered_map<int, std::string> idToClass;
  std::vector<std::string> classNames;
  void fillClassNames(int totalClasses);
  cv::Mat prepFrame(cv::Mat img);
  size_t vectorProduct(std::vector<int64_t> vector);
  std::vector<cv::Point> getTLandBR(std::vector<float>::iterator it);
  std::vector<std::string> namesBackend = {
    "DNN_BACKEND_DEFAULT",
    "DNN_BACKEND_HALIDE",
    "DNN_BACKEND_INFERENCE_ENGINE",
    "DNN_BACKEND_OPENCV",
    "DNN_BACKEND_VKCOM",
    "DNN_BACKEND_CUDA",
    "DNN_BACKEND_WEBNN",
    "DNN_BACKEND_TIMVX"
  };
  std::vector<std::string> namesTarget = {
    "DNN_TARGET_CPU",
    "DNN_TARGET_OPENCL",
    "DNN_TARGET_OPENCL_FP16",
    "DNN_TARGET_MYRIAD",
    "DNN_TARGET_VULKAN",
    "DNN_TARGET_FPGA",
    "DNN_TARGET_CUDA",
    "DNN_TARGET_CUDA_FP16",
    "DNN_TARGET_HDDL",
    "DNN_TARGET_NPU"
  };

};

#endif  //NEURALDETECTOR_HPP