/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

// class to calculate and save necessary features
// author: ep

#ifndef DATA_STACKS_HPP
#define DATA_STACKS_HPP

#include <vector>
#include <string>
#include "imgproc/ImageProcessor.hpp"
#include "point_clouds/PCLFunctions.hpp"
#include "capture/CapturedFrames.hpp"

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/pthread/mutex.hpp>

//#include <v4r/EPUtils/EPUtils.hpp>

template<class T>
struct DataElement {
                    T element;
    unsigned long int frameNumber;
};

struct ExtractedPlane {

    ExtractedPlane()
    : cloud(new pcl::PointCloud<pcl::PointXYZ>()),
      indices(new pcl::PointIndices()),
      indices_plane(new pcl::PointIndices()),
      indices_objects(new pcl::PointIndices()),
      indices_filtered_objects(new pcl::PointIndices()),
      coefficients(new pcl::ModelCoefficients()) {
    }

    void reset() {
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        indices.reset(new pcl::PointIndices());
        indices_plane.reset(new pcl::PointIndices());
        indices_objects.reset(new pcl::PointIndices());
        indices_filtered_objects.reset(new pcl::PointIndices());
        coefficients.reset(new pcl::ModelCoefficients());
    }
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud;
    pcl::PointIndices::Ptr indices;
    pcl::PointIndices::Ptr indices_plane;
    pcl::PointIndices::Ptr indices_objects;
    pcl::PointIndices::Ptr indices_filtered_objects;
    pcl::ModelCoefficients::Ptr coefficients;
};

template<class T>
class SavedPrimitives {
protected:
    std::vector<DataElement<T> > stack;
    int currentIndex;
    mutable boost::mutex get_element_mutex;
    bool findElement(unsigned long int _frameNumber, int &index);
    std::string ClassName;
    //bool motion_detected;
    CapturedFrames* capturedFrames;

public:
    SavedPrimitives(const std::string &_ClassName = std::string("Empty"));
    ~SavedPrimitives() {};
    virtual bool getElement(T &_element, unsigned long int _frameNumber);
    virtual bool computeElement(unsigned long int _frameNumber) = 0;
};

class ExtractedPlanesStack : public SavedPrimitives<ExtractedPlane> {
public:

    ~ExtractedPlanesStack() {};
    virtual bool computeElement(unsigned long int _frameNumber);
    static ExtractedPlanesStack* getInstance();

private:
    ExtractedPlanesStack(const std::string &_ClassName = std::string("ExtractedPlanesStack")) :
    SavedPrimitives<ExtractedPlane>(_ClassName) 
    {
        instance = this;
    };
    static ExtractedPlanesStack* instance;
};

struct FilteredCloud {

    FilteredCloud()
    : cloud(new pcl::PointCloud<pcl::PointXYZ>()),
      indices(new pcl::PointIndices()){
    }

    void reset() {
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        indices.reset(new pcl::PointIndices());
    }
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud;
    pcl::PointIndices::Ptr indices;
};

class FilteredCloudsStack : public SavedPrimitives<FilteredCloud> {
public:
    ~FilteredCloudsStack() {};
    virtual bool computeElement(unsigned long int _frameNumber);
    static FilteredCloudsStack* getInstance();

private:
    FilteredCloudsStack(const std::string &_ClassName = std::string("FilteredCloudsStack")) :
    SavedPrimitives<FilteredCloud>(_ClassName) {
        instance = this;
    };
    static FilteredCloudsStack* instance;
};

struct NormalsCloud {

    NormalsCloud()
    : cloud(new pcl::PointCloud<pcl::PointXYZ>()),
      indices(new pcl::PointIndices()),
      indices_objects(new pcl::PointIndices()),
      normals(new pcl::PointCloud<pcl::Normal>()),
      coefficients(new pcl::ModelCoefficients()){
    }

    void reset() {
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        indices.reset(new pcl::PointIndices());
        indices_objects.reset(new pcl::PointIndices());
        normals.reset(new pcl::PointCloud<pcl::Normal>());
        coefficients.reset(new pcl::ModelCoefficients());
    }
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud;
    pcl::PointIndices::Ptr indices;
    pcl::PointIndices::Ptr indices_objects;
    pcl::PointCloud<pcl::Normal>::Ptr normals;
    pcl::ModelCoefficients::Ptr coefficients;
};

class PointNormalsStack : public SavedPrimitives<NormalsCloud> {
public:

    ~PointNormalsStack() {
    };
    virtual bool computeElement(unsigned long int _frameNumber);
    static PointNormalsStack* getInstance();
private:

    PointNormalsStack(const std::string &_ClassName = std::string("PointNormalsStack")) :
    SavedPrimitives<NormalsCloud>(_ClassName) {
        instance = this;
    };
    static PointNormalsStack* instance;
};

#endif //DATA_STACKS_HPP
