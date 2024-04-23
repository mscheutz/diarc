/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   PeopleDetector.hpp
 * Author: evan krause
 *
 * Created on November 26, 2012, 10:51 AM
 */

#ifndef PEOPLEDETECTOR_HPP
#define PEOPLEDETECTOR_HPP

#include "ObjectDetector.hpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/gpu/people/people_detector.h>

class PeopleDetector : public ObjectDetector {
public:
    typedef boost::shared_ptr<PeopleDetector> Ptr;
    typedef boost::shared_ptr<const PeopleDetector> ConstPtr;

    PeopleDetector(const long long& moTypeId, const long long& processorId, const int imgWidth, const int imgHeight);
    ~PeopleDetector();

    void loadConfig(const std::string& config);

protected:
  virtual void handleCaptureNotification(CaptureNotification::ConstPtr notification);
  
private:
    pcl::gpu::people::PeopleDetector::Ptr peopleDetector;
    pcl::gpu::people::RDFBodyPartsDetector::Ptr rdf;
    pcl::visualization::PCLVisualizer::Ptr vis;	//TODO: remove -- debug only
};

#endif  //PEOPLEDETECTOR_HPP
