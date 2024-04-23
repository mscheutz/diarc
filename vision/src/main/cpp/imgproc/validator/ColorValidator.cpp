/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "ColorValidator.hpp"
#include "capture/calibration/Cameras.hpp"
#include "display/Display.hpp"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>

using namespace ade::stm;

ColorValidator::ColorValidator(const long long& processorId, const unsigned int imgWidth,
        const unsigned int imgHeight, const bool isStereo)
: ObjectValidator(processorId, imgWidth, imgHeight, isStereo) {
  visionProcessName = "ColorValidator";
  logger = log4cxx::Logger::getLogger("ade.imgproc.validator.ColorValidator");
}

ColorValidator::~ColorValidator() {
}

void ColorValidator::handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification) {
  LOG4CXX_TRACE(logger, "[handleMemoryObjectNotification] method entered.");
  MemoryObject::Ptr object = notification->object;
  //get captured frames info
  std::vector<int> indices = object->getDetectionMask()->getIndicesMask();
  const cv::Mat image = object->getCaptureData()->frame;

  // get current descriptors to process
  TypesByDescriptorConstPtr descriptors = getDescriptors();

  int result;
  float confidence = 0.0f;
  cv::Mat_<float> imageMask;
  TypesByDescriptor::const_iterator descriptors_itr;
  for (descriptors_itr = descriptors->begin(); descriptors_itr != descriptors->end(); ++descriptors_itr) {
    std::map<std::string, cv::Scalar>::iterator it = colorMap.find(descriptors_itr->first.getName());
    if (it == colorMap.end()) {
      continue;
    }
    result = calculateColor(image, indices, it->second, confidence, imageMask);
    if (!result) {
      LOG4CXX_DEBUG(logger, boost::format("%s color confidence: %f. Not above conf thresh.") % it->first % confidence);
      continue;
    }

    LOG4CXX_DEBUG(logger, boost::format("%s color confidence: %f.") % it->first % confidence);
    object->addValidationResult(confidence, descriptors_itr->first, imageMask);
  }

  sendValidationNotifications(object);

  if (getDisplayFlag()) {
    displayMemoryObjectValidation(object, true);
  }
}

void ColorValidator::loadConfig(const std::string& config) {
  // populate tree structure pt
  using boost::property_tree::ptree;
  ptree pt;
  read_xml(config, pt);

  std::string predicateName;

  // parse tree

  BOOST_FOREACH(ptree::value_type const& predicateNode, pt.get_child("processor")) {
    if (predicateNode.first.compare("predicate") == 0) {
      predicateName = predicateNode.second.get<std::string> ("<xmlattr>.name", "unknown");
      LOG4CXX_DEBUG(logger, boost::format("[loadConfig] predicateName: %s.") % predicateName);

      float r = predicateNode.second.get("r", 0);
      float g = predicateNode.second.get("g", 0);
      float b = predicateNode.second.get("b", 0);

      std::pair < std::map<std::string, cv::Scalar>::iterator, bool> ret;
      ret = colorMap.insert(std::pair<std::string, cv::Scalar > (predicateName, cv::Scalar_<double>(r, g, b)));
    }
  }
}

bool ColorValidator::calculateColor(const cv::Mat& image, const std::vector<int>& indices, const cv::Scalar& color, float &confidence, cv::Mat_<float>& imageMask) {

  float r_color = color(0);
  float g_color = color(1);
  float b_color = color(2);
  r_color = r_color / 255;
  g_color = g_color / 255;
  b_color = b_color / 255;

  // red color (1,0,0)
  float max2red = sqrt((r_color - 1)*(r_color - 1) + (g_color - 0)*(g_color - 0) + (b_color - 0)*(b_color - 0));
  // green color (0,1,0)
  float max2green = sqrt((r_color - 0)*(r_color - 0) + (g_color - 1)*(g_color - 1) + (b_color - 0)*(b_color - 0));
  // blue color (0,0,1)
  float max2blue = sqrt((r_color - 0)*(r_color - 0) + (g_color - 0)*(g_color - 0) + (b_color - 1)*(b_color - 1));
  // red-green color (1,1,0)
  float max2red_green = sqrt((r_color - 1)*(r_color - 1) + (g_color - 1)*(g_color - 1) + (b_color - 0)*(b_color - 0));
  // red-blue color (1,0,1)
  float max2red_blue = sqrt((r_color - 1)*(r_color - 1) + (g_color - 0)*(g_color - 0) + (b_color - 1)*(b_color - 1));
  // green-blue color (0,1,1)
  float max2green_blue = sqrt((r_color - 0)*(r_color - 0) + (g_color - 1)*(g_color - 1) + (b_color - 1)*(b_color - 1));
  // black color (0,0,0)
  float max2black = sqrt((r_color - 0)*(r_color - 0) + (g_color - 0)*(g_color - 0) + (b_color - 0)*(b_color - 0));
  // white color (1,1,1)
  float max2white = sqrt((r_color - 1)*(r_color - 1) + (g_color - 1)*(g_color - 1) + (b_color - 1)*(b_color - 1));

  float max_dist;
  max_dist = std::max(max2red, max2green);
  max_dist = std::max(max_dist, max2blue);
  max_dist = std::max(max_dist, max2red_green);
  max_dist = std::max(max_dist, max2red_blue);
  max_dist = std::max(max_dist, max2green_blue);
  max_dist = std::max(max_dist, max2black);
  max_dist = std::max(max_dist, max2white);

  //initialize resulting image mask with zeros
  imageMask = cv::Mat_<float>::zeros(image.size());

  long numPoints = 0;
  confidence = 0;
  int imgWidth = image.cols;
  int imgHeight = image.rows;
  float confidence_thresh = 0.5f;
  float pixel_confidence_thresh = 0.4f;

  std::vector<int>::const_iterator indices_itr;
  for (indices_itr = indices.begin(); indices_itr != indices.end(); ++indices_itr) {

    int x = (*indices_itr) % imgWidth;
    int y = (*indices_itr) / imgWidth;

    float rr = image.at<uchar>(y, 3 * x + 2);
    float gg = image.at<uchar>(y, 3 * x + 1);
    float bb = image.at<uchar>(y, 3 * x + 0);
    rr = rr / 255;
    gg = gg / 255;
    bb = bb / 255;

    float dist = (rr - r_color)*(rr - r_color) + (gg - g_color)*(gg - g_color) + (bb - b_color)*(bb - b_color);
    dist = sqrt(dist);
    float pixel_confidence = 1.0 - dist / max_dist;
    numPoints++;
    if (pixel_confidence > pixel_confidence_thresh) {
      imageMask(y, x) = pixel_confidence;
      confidence += pixel_confidence;
    }
  }

  confidence /= numPoints;

  if (numPoints > 0 && confidence > confidence_thresh) {
    return true;
  }

  return false;
}
