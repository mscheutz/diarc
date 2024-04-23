/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include <boost/format.hpp>
#include "StmUtilities.hpp"

namespace ade {
  namespace stm {
    namespace util {

      //log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger("ade.stm.util.Utilities");

      float calculateBoundBoxOverlap(const cv::Rect &bb1, const cv::Rect &bb2) {
        LOG4CXX_DEBUG(logger, boost::format("[calculateBoundBoxOverlap] b1=(%d,%d,%d,%d) b2=(%d,%d,%d,%d)")
                              % bb1.x % bb1.y % bb1.width % bb1.height
                              % bb2.x % bb2.y % bb2.width % bb2.height);
        //compare two bounding boxes
        int x1 = bb1.x;
        int y1 = bb1.y;
        int x2 = bb1.x + bb1.width;
        int y2 = bb1.y + bb1.height;
        float area = (x2 - x1 + 1) * (y2 - y1 + 1);

        int x1_other = bb2.x;
        int y1_other = bb2.y;
        int x2_other = bb2.x + bb2.width;
        int y2_other = bb2.y + bb2.height;
        float area_other = (x2_other - x1_other + 1) * (y2_other - y1_other + 1);

        int x1_overlap = std::max(x1, x1_other);
        int y1_overlap = std::max(y1, y1_other);
        int x2_overlap = std::min(x2, x2_other);
        int y2_overlap = std::min(y2, y2_other);
        if ((x1_overlap >= x2_overlap) || (y1_overlap >= y2_overlap)) {
          return 0.0;
        }

        float area_overlap = (x2_overlap - x1_overlap + 1) * (y2_overlap - y1_overlap + 1);

        return area_overlap / (std::min(area, area_other));
      }

      float calculateBoundBoxSizeSimilarity(const cv::Rect &bb1, const cv::Rect &bb2) {
        //compare overall size
        float widthDiff = std::abs(bb1.width - bb2.width);
        float heightDiff = std::abs(bb1.height - bb2.height);
        float sizeScore = (1.0 - widthDiff / std::max(bb1.width, bb2.width))
                          * (1.0 - heightDiff / std::max(bb1.height, bb2.height));

        return sizeScore;
      }

    } //namespace
  }
}