/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * Class for object shape validation.
 *
 * This uses the PCL object classification system. Classes there can be anything (mugs,
 * chairs, or the class of all containeres, all furniture to sit on etc.)
 * We will use it to recognise object shapes, i.e. rather "narrow" classes, such as
 * mugs, boxes, chairs etc.
 *
 * @author Michael Zillich (MZ)
 * @date Dec 2012
 */

#ifndef SHAPEVALIDATOR_HPP
#define SHAPEVALIDATOR_HPP

#include "ObjectValidator.hpp"

#include <pcl/segmentation/sac_segmentation.h>
#include "shape_match_3d/PCLFeatureKNNClassifier.hpp"
#include <pcl/features/usc.h>
#include <pcl/features/feature.h>

class ShapeValidator : public ObjectValidator {
public:
  typedef boost::shared_ptr<ShapeValidator> Ptr;
  typedef boost::shared_ptr<const ShapeValidator> ConstPtr;

  ShapeValidator(const long long& processorId, const unsigned int imgWidth,
          const unsigned int imgHeight, const bool isStereo);
  ~ShapeValidator();

  virtual void loadConfig(const std::string& config);

protected:
  virtual void handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification);
  virtual void handleMemoryObjectsNotification(MemoryObjectsNotification::ConstPtr notification);

private:
  bool haveNewObject(diarc::stm::MemoryObject::Ptr object);
  bool haveNewObjects(diarc::stm::MemoryObject::VecPtr objects);
  bool checkForShapes(diarc::stm::MemoryObject::Ptr object);
  float checkForBox(diarc::stm::MemoryObject::Ptr object);
  float checkForCylinder(diarc::stm::MemoryObject::Ptr object);
  void display(diarc::stm::MemoryObject::VecPtr objects);

  //for cylinder detection
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> cylinderSeg;

  // EXPERIMENTAL //
//  pcl::UniqueShapeContext< pcl::PointXYZRGB> usc;
//  PCLFeatureKNNClassifier<pcl::ShapeContext1980> classifier;
//  bool checkShape(diarc::stm::MemoryObject::Ptr object);
//  void filterPointCloud(const cv::Mat& image, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr inputCloud,
//          pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ouputCloud);
//  void removeBorderEdges(const cv::Mat& image, pcl::PointIndicesPtr& cloudIndices);

};

#endif  //SHAPEVALIDATOR_HPP
