/*
Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
*/

#ifndef PYTHONUTILS_HPP
#define PYTHONUTILS_HPP


#include <log4cxx/logger.h>
#include <pybind11/numpy.h>
#include <opencv2/opencv.hpp>

class PYBIND11_EXPORT PythonUtils {
public:

    static log4cxx::LoggerPtr logger;
    static pybind11::array_t<uchar> convertMatToNumpy(const cv::Mat& mat);
    static cv::Mat convertNumpyToMat(const pybind11::array_t<uchar>& array);
};



#endif //PYTHONUTILS_HPP
