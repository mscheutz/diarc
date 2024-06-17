/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   OrientationValidator.hpp
 * Author: evan
 *
 * Created on March 27, 2014, 5:23 PM
 */

#ifndef COMPONENTVALIDATOR_HPP
#define	COMPONENTVALIDATOR_HPP

#include "ObjectValidator.hpp"

class ComponentValidator : public ObjectValidator {
public:
  typedef boost::shared_ptr<ComponentValidator> Ptr;
  typedef boost::shared_ptr<const ComponentValidator> ConstPtr;

  ComponentValidator(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight);
  virtual ~ComponentValidator();
  virtual void loadConfig(const std::string& configFile);

  // indices of shape and spatial parameters in the Semantic Vector
  std::vector<std::vector<int>> shapeIdcs;
  std::vector<std::vector<int>> spatialIdcs;
  std::vector<std::string> shapeNames;

  typedef struct {
      int x;
      int y;
      int z;
    } LocationCoords;

  typedef struct {
    std::string shape;
    std::string relativeLocationX;
    std::string relativeLocationY;
    std::string relativeLocationZ;
  } ComponentCandidate;

protected:
  cv::dnn::Net net;

  // semantic parent type for this detector's descriptors (isA hierarchy)
  std::string parentType;
  std::unordered_map<std::string, int> classToId;
  std::unordered_map<int, std::string> idToClass;

  virtual void handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification);
  virtual void initModel(const std::string& model_path);

private:
  std::vector<std::string> semanticProperties;

  bool calculateSemProperty(const cv::Mat& semMat, const std::string& semPred);

};



#endif	/* COMPONENTVALIDATOR_HPP */

