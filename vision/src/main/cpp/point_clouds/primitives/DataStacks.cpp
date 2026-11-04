/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "DataStacks.hpp"

template<class T>
SavedPrimitives<T>::SavedPrimitives(const std::string &_ClassName) :
ClassName(_ClassName),
currentIndex(-1) 
{
  stack.reserve(VisionConstants::maxFrameHistoryLength);
  capturedFrames = CapturedFrames::getInstance();
}

template<class T>
bool SavedPrimitives<T>::getElement(T &_element, unsigned long int _frameNumber) 
{
  // block with mutex
  boost::lock_guard<boost::mutex> lock(get_element_mutex);
  int index = stack.size() - 1;
  if (!stack.size() || (/*(motion_detected) &&*/ !findElement(_frameNumber, index))) 
  {
    if(!computeElement(_frameNumber))
      return (false);
    index = currentIndex;
  }
    
  _element = (stack.at(index).element);
  return (true);
}

template<class T>
bool SavedPrimitives<T>::findElement(unsigned long int _frameNumber, int &index) 
{
  for (int i = 0; i < stack.size(); ++i) {
    if (stack.at(i).frameNumber == _frameNumber) 
    {
      index = i;
      return (true);
    }
  }
  return (false);
}

bool FilteredCloudsStack::computeElement(unsigned long int _frameNumber) 
{
  const unsigned long int currFrameNumber = _frameNumber;
  DataElement<FilteredCloud> currentElement;
  currentElement.element.cloud = capturedFrames->getPointCloud(currFrameNumber);
   
  if (!FilterPointCloud<pcl::PointXYZ>(currentElement.element.cloud, currentElement.element.indices, "z", 0.0, 10.0))
  {
    return (false);
  }
  
  currentElement.frameNumber = currFrameNumber;
  currentIndex = (currentIndex + 1) % VisionConstants::maxFrameHistoryLength;
  if (stack.size() < VisionConstants::maxFrameHistoryLength)
  {
    stack.push_back(currentElement);
  }
  else 
  {
    stack.at(currentIndex) = currentElement;
  }
  return (true);
}

FilteredCloudsStack* FilteredCloudsStack::instance = NULL;

FilteredCloudsStack* FilteredCloudsStack::getInstance() 
{
  if(instance == NULL) 
  {
    FilteredCloudsStack* instance2 = new FilteredCloudsStack(std::string("FilteredCloudsStack"));
  }
  return instance;
}

bool ExtractedPlanesStack::computeElement(unsigned long int _frameNumber) 
{
  const unsigned long int currFrameNumber = _frameNumber;
  const cv::Mat transform = capturedFrames->getTransform(_frameNumber);
  DataElement<ExtractedPlane> currentElement;

  FilteredCloudsStack *filteredCloudsStackPtr = FilteredCloudsStack::getInstance();
  FilteredCloud filteredCloud;
  if(!filteredCloudsStackPtr->getElement(filteredCloud,currFrameNumber))
  {
    return (false);
  }
  currentElement.element.cloud = filteredCloud.cloud;
  currentElement.element.indices = filteredCloud.indices;

//std::cerr << "[filtercloud] cloud size: " << currentElement.element.indices->indices.size() << std::endl;
//for (int i = 0; i < currentElement.element.indices->indices.size(); ++i) {
//int index = currentElement.element.indices->indices[i];
//	if (std::isnan(currentElement.element.cloud->points[index].x) || std::isnan(currentElement.element.cloud->points[index].y) || std::isnan(currentElement.element.cloud->points[index].z)) {
//
//		std::cerr << index << ": (" << currentElement.element.cloud->points[index].x << ", " <<
//			currentElement.element.cloud->points[index].y << ", " <<
//			currentElement.element.cloud->points[index].z << ")" <<
//			std::endl;
//	}
//}

  if(!SegmentPlane<pcl::PointXYZ>(currentElement.element.cloud,currentElement.element.indices,
                                  currentElement.element.indices_plane,currentElement.element.indices_objects,
                                  currentElement.element.coefficients, transform))
  {
    return (false);
  }

//std::cerr << "[segmentplane] plane size: " << currentElement.element.indices_plane->indices.size() << std::endl;
//std::cerr << "[segmentplane] object candidate size: " << currentElement.element.indices_objects->indices.size() << std::endl;
//for (int i = 0; i < currentElement.element.indices_plane->indices.size(); ++i) {
//int index = currentElement.element.indices_plane->indices[i];
//	if (std::isnan(currentElement.element.cloud->points[index].x) || std::isnan(currentElement.element.cloud->points[index].y) || std::isnan(currentElement.element.cloud->points[index].z)) {
//
//		std::cerr << index << ": (" << currentElement.element.cloud->points[index].x << ", " <<
//			currentElement.element.cloud->points[index].y << ", " <<
//			currentElement.element.cloud->points[index].z << ")" <<
//			std::endl;
//	}
//}
  
  if(!filterPointsOnPlane<pcl::PointXYZ>(currentElement.element.cloud,currentElement.element.indices_plane,
                                         currentElement.element.indices_objects,currentElement.element.coefficients,
                                         currentElement.element.indices_filtered_objects))
  {
    return (false);
  }
  //currentElement.element.indices_filtered_objects = currentElement.element.indices_objects;

//std::cerr << "[filterpointsonplane] object candidate size: " << currentElement.element.indices_objects->indices.size() << std::endl;
//std::cerr << "[filterpointsonplane] filtered candidate size: " << currentElement.element.indices_filtered_objects->indices.size() << std::endl;
//for (int i = 0; i < currentElement.element.indices_filtered_objects->indices.size(); ++i) {
//int index = currentElement.element.indices_filtered_objects->indices[i];
//	if (std::isnan(currentElement.element.cloud->points[index].x) || std::isnan(currentElement.element.cloud->points[index].y) || std::isnan(currentElement.element.cloud->points[index].z)) {
//
//		std::cerr << index << ": (" << currentElement.element.cloud->points[index].x << ", " <<
//			currentElement.element.cloud->points[index].y << ", " <<
//			currentElement.element.cloud->points[index].z << ")" <<
//			std::endl;
//	}
//}

  currentElement.frameNumber = currFrameNumber;
  currentIndex = (currentIndex + 1) % VisionConstants::maxFrameHistoryLength;
  if(stack.size() < VisionConstants::maxFrameHistoryLength) 
  {
    stack.push_back(currentElement);
  } else 
  {
    stack.at(currentIndex) = currentElement;
  }
  return (true);
}

ExtractedPlanesStack* ExtractedPlanesStack::instance = NULL;

ExtractedPlanesStack* ExtractedPlanesStack::getInstance() 
{
  if(instance == NULL) 
  {
    ExtractedPlanesStack* instance2 = new ExtractedPlanesStack(std::string("ExtractedPlanesStack"));
  }
  return instance;
}

bool PointNormalsStack::computeElement(unsigned long int _frameNumber) 
{
  const unsigned long int currFrameNumber = _frameNumber;

  ExtractedPlanesStack *extractedPlanesStackPtr = ExtractedPlanesStack::getInstance();
  ExtractedPlane extractedPlane;

  if(!extractedPlanesStackPtr->getElement(extractedPlane,currFrameNumber))
    return (false);

  DataElement<NormalsCloud> currentElement;
  currentElement.element.cloud = extractedPlane.cloud;
  currentElement.element.indices = extractedPlane.indices;
  currentElement.element.indices_objects = extractedPlane.indices_objects;
  currentElement.element.coefficients = extractedPlane.coefficients;
  
  if(!ComputePointNormals<pcl::PointXYZ>(currentElement.element.cloud,
                                         currentElement.element.indices_objects,
                                         currentElement.element.normals))
    return (false);

  currentElement.frameNumber = currFrameNumber;
  currentIndex = (currentIndex + 1) % VisionConstants::maxFrameHistoryLength;
  if(stack.size() < VisionConstants::maxFrameHistoryLength) 
  {
    stack.push_back(currentElement);
  } 
  else 
  {
    stack.at(currentIndex) = currentElement;
  }
  return (true);
}

PointNormalsStack* PointNormalsStack::instance = NULL;

PointNormalsStack* PointNormalsStack::getInstance() 
{
  if(instance == NULL) 
  {
    PointNormalsStack* instance2 = new PointNormalsStack(std::string("PointNormalsStack"));
  }
  return instance;
}
