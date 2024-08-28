/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "ColorProcessor.hpp"
#include <capture/calibration/Cameras.hpp>
#include "display/Display.hpp"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>

ColorProcessor::ColorProcessor(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight, const bool isStereo)
: SaliencyProcessor(processorId, imgWidth, imgHeight, isStereo) {
  colorSaliencyMap.setWidth(imgWidth);
  colorSaliencyMap.setHeight(imgHeight);
  colorSaliencyMap.setUseLAB(false);
  visionProcessName = "ColorProcessor";
  logger = log4cxx::Logger::getLogger("diarc.imgproc.saliency.ColorProcessor");
}

ColorProcessor::~ColorProcessor() {
}

void ColorProcessor::handleCaptureNotification(CaptureNotification::ConstPtr notification) {
  LOG4CXX_TRACE(logger, "[handleCaptureNotification] method entered.");
  const cv::Mat image = notification->captureData->frame;

  // get current descriptors to process
  TypesByDescriptorConstPtr descriptors = getDescriptors();

  int numDescriptors = descriptors->size();
  if (!numDescriptors) {
    LOG4CXX_ERROR(logger, "No descriptors set, returning.");
    return;
  }

  cv::Mat tmpImage = image; //bc setImage doesn't take **const** ref like it should 
  colorSaliencyMap.setImage(tmpImage);

  // maps to combine
//  int mapsIndex = 0;
  std::vector<cv::Mat> maps;
  maps.reserve(numDescriptors);
  cv::Mat temp;

  std::map<std::string, cv::Scalar>::iterator it;
  TypesByDescriptor::const_iterator descriptors_itr = descriptors->begin();
  for (descriptors_itr = descriptors->begin(); descriptors_itr != descriptors->end(); ++descriptors_itr) {
    it = colorMap.find(descriptors_itr->first.getName());
    if (it == colorMap.end()) {
      LOG4CXX_ERROR(logger, boost::format("Unknown saliency color: %s.") % descriptors_itr->first.getName());
      continue;
    }
    colorSaliencyMap.setColor(it->second);
    if (colorSaliencyMap.calculateColorMap(temp)) {
      LOG4CXX_ERROR(logger, "Unable to calculate color saliency.");
      return;
    }

    maps.push_back(temp);
    temp.release();
//    temp.copyTo(maps.at(mapsIndex++));
  }
  
  if (maps.size() == 0) {
    return;
  }

  cv::Mat resultImage;
  AttentionModule::CombineMaps(maps, resultImage, AttentionModule::AM_SUM, EPUtils::NT_EMPTY);

  Notification::Ptr n(new SaliencyNotification(shared_from_this(), notification->captureData->frameNumber, notification->captureData, resultImage));
  sendNotifications(n);

  if (getDisplayFlag()) {
    diarc::Display::displayFrame(resultImage, getDisplayName());
  }
}

void ColorProcessor::loadConfig(const std::string & config) {
  // populate tree structure pt
  using boost::property_tree::ptree;
  ptree pt;
  read_xml(config, pt);

  std::string predicateName;

  // traverse pt

  BOOST_FOREACH(ptree::value_type const& predicateNode, pt.get_child("processor")) {
    if (predicateNode.first.compare("predicate") == 0) {

      predicateName = predicateNode.second.get<std::string> ("<xmlattr>.name", "unknown");
      float r = predicateNode.second.get("r", 0);
      float g = predicateNode.second.get("g", 0);
      float b = predicateNode.second.get("b", 0);

      LOG4CXX_DEBUG(logger, boost::format("[loadConfig] %s: (r,g,b) = (%f,%f,%f).") % predicateName % r % g % b);
      std::pair < std::map<std::string, cv::Scalar>::iterator, bool> ret;
      ret = colorMap.insert(std::pair<std::string, cv::Scalar > (predicateName, cv::Scalar_<double>(r, g, b)));
    }
  }

  return;
}
