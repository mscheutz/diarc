/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include <cfloat>
#include <boost/lexical_cast.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "capture/util/CaptureUtilities.hpp"
#include "FastSamDetector.hpp"
#include "display/Display.hpp"

 #include <pybind11/embed.h> // everything needed for embedding
 namespace py = pybind11;
#include <dlfcn.h>
#include <Python.h>
//#include <boost/python/import.hpp>
//namespace py = boost::python;
//namespace np = boost::python::numpy;

using namespace diarc::stm;

FastSamDetector::FastSamDetector(const long long& processorId, const int imgWidth, const int imgHeight)
: ObjectDetector(processorId, imgWidth, imgHeight) {
  salmap = cv::Mat_<float>::ones(imgHeight, imgWidth);
  visionProcessName = "FastSamDetector";
  logger = log4cxx::Logger::getLogger("diarc.detector.FastSamDetector");

  // When you load a NumPy-dependent plugin, do the following just before that.
  //dlopen("libpython3.10.so", RTLD_LAZY | RTLD_GLOBAL);
}

FastSamDetector::~FastSamDetector() {
}

void FastSamDetector::init() {
   py::initialize_interpreter();
  //Py_Initialize();
  //boost::python::numpy::initialize();
}

void FastSamDetector::cleanup() {
   py::finalize_interpreter();
  //Py_Finalize();
}

void FastSamDetector::handleSaliencyNotification(SaliencyNotification::ConstPtr notification) {
  notification->saliencyMap.copyTo(salmap);
}

void FastSamDetector::handleCaptureNotification(CaptureNotification::ConstPtr notification) {
  cv::Mat frame = notification->captureData->frame;

  // PYBIND11

   py::print("Hello, World!"); // use the Python API

   // Equivalent to "from vector_perception.segmentation import Sam2DSegmenter"
   py::object Sam2DSegmenter = py::module_::import("vector_perception.segmentation").attr("Sam2DSegmenter");

   // Construct a Python object of class Decimal
   py::object segmenter = Sam2DSegmenter(
       py::arg("model_path") = "models/FastSAM-s.pt",
       py::arg("device") = "cpu",
       py::arg("min_analysis_interval") = 4.0,
       py::arg("use_tracker") = false,
       py::arg("use_analyzer") = false
       );


  // process the image
  // results[0] --> [mask0, mask1, ...], each mask is binary image mask?
  // results[1] --> [bbox1, bbox2, ...], each box is [x1, y1, x2, y2]
  // results[2] --> [track_id1, track_id2, ...]
  // results[3] --> [conf1, conf2, ...]
  // results[4] --> [name1, name2, ...]
  //np::ndarray pyFrame = convertMatToNDArray(frame);
  //py::object results = segmenter.attr("process_image")(pyFrame);

  // Visualize results
  //py::object overlay = segmenter.attr("visualize_results")(frame, results[0], results[1], results[2], results[3], results[4]);
  //displayFrame = convertNDArrayToMat(py::extract<np::ndarray>(overlay));

  // BOOST PYTHON

  //try {
  //
  //  // Equivalent to "from vector_perception.segmentation import Sam2DSegmenter"
  //  py::object Sam2DSegmenter = py::import("vector_perception.segmentation").attr("Sam2DSegmenter");
  //
  //  // Create an instance of the class
  //  py::object segmenter = Sam2DSegmenter(
  //      "models/FastSAM-s.pt",
  //      "cpu",
  //      4.0,
  //      false,
  //      false,
  //      false
  //      );
  //
  //  // process the image
  //  // results[0] --> [mask0, mask1, ...], each mask is binary image mask?
  //  // results[1] --> [bbox1, bbox2, ...], each box is [x1, y1, x2, y2]
  //  // results[2] --> [track_id1, track_id2, ...]
  //  // results[3] --> [conf1, conf2, ...]
  //  // results[4] --> [name1, name2, ...]
  //  np::ndarray pyFrame = convertMatToNDArray(frame);
  //  py::object results = segmenter.attr("process_image")(pyFrame);
  //
  // // Visualize results
  //  py::object overlay = segmenter.attr("visualize_results")(frame, results[0], results[1], results[2], results[3], results[4]);
  //  displayFrame = convertNDArrayToMat(py::extract<np::ndarray>(overlay));
  //
  //
  //  // for (int i = 0; i < boost::python::len(results[0]); ++i) {
  //  //   py::object mask = results[0][i];
  //  // }
  //
  //  // Access a method of the instance
  //  // py::object get_value_method = instance.attr("get_value");
  //  // int value = py::extract<int>(get_value_method());
  //  //
  //  // std::cout << "Value: " << value << std::endl;
  //
  //} catch (const py::error_already_set&) {
  //  PyErr_Print();
  //  return;
  //}

  //send notifications
  // MemoryObject::VecPtr newClusterObjects = createMemoryObjects(clusters, notification->captureData);
  // sendDetectionNotifications(newClusterObjects);

  if (getDisplayFlag()) {
    diarc::Display::displayFrame(displayFrame, getDisplayName());
  }
}

// py::object FastSamDetector::convertCvMatToPyMat(cv::Mat& cvMat) {
//
// }
//
// cv::Mat FastSamDetector::convertPyMatToCvMat(py::object& pyMat) {
//
// }

//np::ndarray FastSamDetector::convertMatToNDArray(const cv::Mat& mat) {
//  py::tuple shape = py::make_tuple(mat.rows, mat.cols, mat.channels());
//  py::tuple stride = py::make_tuple(mat.channels() * mat.cols * sizeof(uchar), mat.channels() * sizeof(uchar), sizeof(uchar));
//  np::dtype dt = np::dtype::get_builtin<uchar>();
//  np::ndarray ndImg = np::from_data(mat.data, dt, shape, stride, py::object());
//
//  return ndImg;
//}
//
//cv::Mat FastSamDetector::convertNDArrayToMat(const np::ndarray& ndarr) {
//  //int length = ndarr.get_nd(); // get_nd() returns num of dimensions. this is used as a length, but we don't need to use in this case. because we know that image has 3 dimensions.
//  const Py_intptr_t* shape = ndarr.get_shape(); // get_shape() returns Py_intptr_t* which we can get the size of n-th dimension of the ndarray.
//  char* dtype_str = py::extract<char *>(py::str(ndarr.get_dtype()));
//
//  // variables for creating Mat object
//  int rows = shape[0];
//  int cols = shape[1];
//  int channel = shape[2];
//  int depth;
//
//  // you should find proper type for c++. in this case we use 'CV_8UC3' image, so we need to create 'uchar' type Mat.
//  if (!strcmp(dtype_str, "uint8")) {
//    depth = CV_8U;
//  }
//  else {
//    std::cout << "wrong dtype error" << std::endl;
//    return cv::Mat();
//  }
//
//  int type = CV_MAKETYPE(depth, channel); // CV_8UC3
//
//  cv::Mat mat = cv::Mat(rows, cols, type);
//  memcpy(mat.data, ndarr.get_data(), sizeof(uchar) * rows * cols * channel);
//
//  return mat;
//}