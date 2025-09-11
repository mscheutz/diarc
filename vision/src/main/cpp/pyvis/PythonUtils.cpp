/*
Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
*/

#include "PythonUtils.hpp"

#include <boost/format/format_class.hpp>
namespace py = pybind11;

log4cxx::LoggerPtr PythonUtils::logger = log4cxx::Logger::getLogger("diarc.pyvis.PythonUtils");

py::array_t<uchar> PythonUtils::convertMatToNumpy(const cv::Mat& mat) {
  cv::Mat temp_mat;
  if (mat.type() != CV_8UC1 && mat.type() != CV_8UC3) {
    mat.convertTo(temp_mat, CV_8UC3);
  }
  else {
    temp_mat = mat;
  }

  std::vector<size_t> shape = {
    static_cast<size_t>(temp_mat.rows), static_cast<size_t>(temp_mat.cols), static_cast<size_t>(temp_mat.channels())
  };
  std::vector<size_t> strides = {
    static_cast<unsigned long>(temp_mat.cols * temp_mat.channels()), static_cast<unsigned long>(temp_mat.channels()), 1
  };

  return py::array_t<uchar>(py::buffer_info(
    temp_mat.data,
    sizeof(uchar),
    py::format_descriptor<uchar>::format(),
    3,
    shape,
    strides
  ));
}

cv::Mat PythonUtils::convertNumpyToMat(const py::array_t<uchar>& array) {
  py::buffer_info buf = array.request();

  if (logger->isDebugEnabled()) {
    std::string message = "[convertNumpyToMat] format: " + buf.format + " ndims: " + std::to_string(buf.ndim);
    if (buf.ndim == 2) {
      message += " shape: " + std::to_string(buf.shape[0]) + " " + std::to_string(buf.shape[1]);
      message += " strides: " + std::to_string(buf.strides[0]) + " " + std::to_string(buf.strides[1]);
    } else {
      message += " shape: " + std::to_string(buf.shape[0]) + " " + std::to_string(buf.shape[1]) + " " + std::to_string(buf.shape[2]);
      message += " strides: " + std::to_string(buf.strides[0]) + " " + std::to_string(buf.strides[1]) + " " + std::to_string(buf.strides[2]);
    }
    LOG4CXX_DEBUG(logger, message);
  }

  int cvType = CV_8UC3;
  if (buf.ndim == 2) {
    cvType = CV_8UC1;
  }

  cv::Mat tmp(buf.shape[0], buf.shape[1], cvType, buf.ptr, buf.strides[0]);
  cv::Mat mat = tmp.clone();
  return mat;
}
