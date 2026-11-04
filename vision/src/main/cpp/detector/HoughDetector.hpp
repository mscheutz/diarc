/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   HoughDetector.hpp
 * Author: evan
 *
 * Created on April 1, 2014, 3:48 PM
 */

#ifndef HOUGHDETECTOR_HPP
#define	HOUGHDETECTOR_HPP

#include "ObjectDetector.hpp"
#include "stm/MemoryObject.hpp"
#include "hough/GHT.hpp"

class HoughDetector : public ObjectDetector {
public:
  typedef boost::shared_ptr<HoughDetector> Ptr;
  typedef boost::shared_ptr<const HoughDetector> ConstPtr;

  HoughDetector(const long long& processorId, const int imgWidth, const int imgHeight);
  ~HoughDetector();

  virtual void loadConfig(const std::string& configFile);

protected:
  virtual void handleCaptureNotification(CaptureNotification::ConstPtr notification);
  
private:
  diarc::stm::MemoryObject::VecPtr createMemoryObjects(const std::string& variableName,
        const std::tr1::unordered_set<long long>& typeIds,
        const std::vector<cv::Rect>& rects,
        CaptureNotification::ConstPtr capture);

  boost::unordered_map<std::string, std::string> imgFilenameMap;
  boost::unordered_map<std::string, boost::shared_ptr<GenHoughTrnf> > genHoughTrnfMap;
};

#endif	/* HOUGHDETECTOR_HPP */

