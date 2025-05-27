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

using namespace diarc::stm;

FastSamDetector::FastSamDetector(const long long& processorId, const int imgWidth, const int imgHeight)
: ObjectDetector(processorId, imgWidth, imgHeight) {
  salmap = cv::Mat_<float>::ones(imgHeight, imgWidth);
  visionProcessName = "FastSamDetector";
  logger = log4cxx::Logger::getLogger("diarc.detector.FastSamDetector");

  // TODO: how to handle this in general?
  // When you load a NumPy-dependent plugin, do the following just before that.
  dlopen("libpython3.8.so", RTLD_LAZY | RTLD_GLOBAL);
}

FastSamDetector::~FastSamDetector() {
  LOG4CXX_INFO(logger, "[destructor]");
}

void FastSamDetector::init() {
  LOG4CXX_INFO(logger, "[init]");

  try {
    py::initialize_interpreter();

    LOG4CXX_INFO(logger, "[init] 2");
    // Equivalent to "from vector_perception.segmentation import Sam2DSegmenter"
    py::module_::import("vector_perception.segmentation");
    //Sam2DSegmenter = py::module_::import("vector_perception.segmentation").attr("Sam2DSegmenter");
//
    //LOG4CXX_INFO(logger, "[init] 3");
    //// Construct a Python object of class Sam2DSegmenter
    //segmenter = Sam2DSegmenter(
    //  py::arg("model_path") = "models/FastSAM-s.pt",
    //  py::arg("device") = "cpu",
    //  py::arg("min_analysis_interval") = 4.0,
    //  py::arg("use_tracker") = false,
    //  py::arg("use_analyzer") = false
    //);

  } catch (py::error_already_set &e) {
    LOG4CXX_ERROR(logger, boost::format("[init] %s") % e.what());
  }

  LOG4CXX_INFO(logger, "[init] done");
}

void FastSamDetector::cleanup() {
  LOG4CXX_INFO(logger, "[cleanup]");
  py::finalize_interpreter();
  LOG4CXX_INFO(logger, "[cleanup] done");
}

void FastSamDetector::handleSaliencyNotification(SaliencyNotification::ConstPtr notification) {
  notification->saliencyMap.copyTo(salmap);
}

void FastSamDetector::handleCaptureNotification(CaptureNotification::ConstPtr notification) {
  cv::Mat frame = notification->captureData->frame;

  // process the image
  // results[0] --> [mask0, mask1, ...], each mask is binary image mask?
  // results[1] --> [bbox1, bbox2, ...], each box is [x1, y1, x2, y2]
  // results[2] --> [track_id1, track_id2, ...]
  // results[3] --> [conf1, conf2, ...]
  // results[4] --> [name1, name2, ...]
  //py::array_t<uchar> pyFrame = convertMatToNumpy(frame);
  //py::object tmpResults = segmenter.attr("process_image")(pyFrame);
  //py::tuple results = py::cast<py::tuple>(tmpResults);
//
  //// Visualize results
  //py::object overlay = segmenter.attr("visualize_results")(
  //  pyFrame,
  //  results[0].cast<py::list>(),
  //  results[1].cast<py::list>(),
  //  results[2].cast<py::list>(),
  //  results[3].cast<py::list>(),
  //  results[4].cast<py::list>()
  //);
  //displayFrame = convertNumpyToMat(overlay.cast<py::array_t<uchar>>());

  //send notifications
  // MemoryObject::VecPtr newClusterObjects = createMemoryObjects(clusters, notification->captureData);
  // sendDetectionNotifications(newClusterObjects);

  if (getDisplayFlag()) {
    diarc::Display::displayFrame(displayFrame, getDisplayName());
  }
}

py::array_t<uchar> FastSamDetector::convertMatToNumpy(const cv::Mat& mat) {
    cv::Mat temp_mat;
    if (mat.type() != CV_8UC1 && mat.type() != CV_8UC3) {
        mat.convertTo(temp_mat, CV_8UC3);
    } else {
        temp_mat = mat;
    }

    std::vector<size_t> shape = { static_cast<size_t>(temp_mat.rows), static_cast<size_t>(temp_mat.cols), static_cast<size_t>(temp_mat.channels()) };
    std::vector<size_t> strides = { temp_mat.cols * temp_mat.channels(), temp_mat.channels(), 1 };

    return py::array_t<uchar>(py::buffer_info(
        temp_mat.data,
        sizeof(uchar),
        py::format_descriptor<uchar>::format(),
        3,
        shape,
        strides
    ));
}

cv::Mat FastSamDetector::convertNumpyToMat(const py::array_t<uchar>& array) {
    py::buffer_info buf = array.request();
    cv::Mat mat(buf.shape[0], buf.shape[1], CV_8UC3, (uchar*)buf.ptr);
    return mat;
}