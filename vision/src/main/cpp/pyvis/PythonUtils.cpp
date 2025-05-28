/*
Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
*/

#include "PythonUtils.hpp"
namespace py = pybind11;


py::array_t<uchar> PythonUtils::convertMatToNumpy(const cv::Mat& mat) {
    cv::Mat temp_mat;
    if (mat.type() != CV_8UC1 && mat.type() != CV_8UC3) {
        mat.convertTo(temp_mat, CV_8UC3);
    } else {
        temp_mat = mat;
    }

    std::vector<size_t> shape = { static_cast<size_t>(temp_mat.rows), static_cast<size_t>(temp_mat.cols), static_cast<size_t>(temp_mat.channels()) };
    std::vector<size_t> strides = { static_cast<unsigned long>(temp_mat.cols * temp_mat.channels()), static_cast<unsigned long>(temp_mat.channels()), 1 };

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
    cv::Mat mat(buf.shape[0], buf.shape[1], CV_8UC3, (uchar*)buf.ptr);
    return mat;
}