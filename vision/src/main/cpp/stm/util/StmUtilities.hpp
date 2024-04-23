/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   StmUtilities.hpp
 * Author: Evan Krause
 *
 * Created on August 13, 2013, 6:24 PM
 */

#ifndef STMUTILITIES_HPP
#define	STMUTILITIES_HPP

#include <opencv2/opencv.hpp>
#include <log4cxx/logger.h>

namespace ade {
  namespace stm {
    namespace util {
      
      static log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger("ade.stm.util.Utilities");

      float calculateBoundBoxOverlap(const cv::Rect& bb1, const cv::Rect& bb2);
      float calculateBoundBoxSizeSimilarity(const cv::Rect& bb1, const cv::Rect& bb2);
    }
  }
}

#endif	/* STMUTILITIES_HPP */

