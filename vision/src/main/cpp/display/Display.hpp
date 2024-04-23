/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef DISPLAY_HPP
#define DISPLAY_HPP

#include <opencv2/opencv.hpp>
#include <common/VisionConstants.hpp>
#include <log4cxx/logger.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <pthread.h>
#include <boost/thread/mutex.hpp>
#include <boost/unordered_map.hpp>
#include <boost/thread/locks.hpp>
//#include <tr1/unordered_map> //<hash_map>     //get segfault when using unordered_map<string, >
#include <tr1/unordered_set> //<hash_set>

#include <pcl/visualization/pcl_visualizer.h>

namespace ade {

  class DisplayWindow2D; //forward declaration
  class DisplayWindow3D; //forward declaration

  //TODO: make this a Singleton class?
  //TODO: make sure all the DisplayWindows are cleaned up properly!!

  class Display {
  public:
    Display();
    ~Display();

    void run();

    static bool windowExists(const std::string& windowName);
    static void createWindowIfDoesNotExist(const std::string& name);
    static void destroyWindowIfItExists(const std::string& name);

    static void displayFrame(const cv::Mat frameToDraw, const std::string& windowName);
    typedef boost::unordered_map<std::string, DisplayWindow2D*> Display2D_Map;
    typedef boost::unordered_map<std::string, DisplayWindow3D*> Display3D_Map;

    static void displayPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr, const std::string& cloudName, const std::string& windowName);
    static void displayPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr, const std::string& cloudName, const std::string& windowName);
    static void displayPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::ConstPtr handler, const std::string& cloudName, const std::string& windowName);
    static void displayPolygonMesh(pcl::PolygonMesh::ConstPtr, const std::string& meshName, const std::string& windowName);
    static void displaySphere(const pcl::PointXYZ center, const double& radius, const std::string& sphereName, const std::string& windowName);
    static void displayPlane(const pcl::ModelCoefficients& coeffs, const std::string& planeName, const std::string& windowName);

    static void removePointCloud(const std::string& cloudName, const std::string& windowName);
    static void removePolygonMesh(const std::string& meshName, const std::string& windowName);
    static void removeSphere(const std::string& sphereName, const std::string& windowName);
    static void removePlane(const std::string& planeName, const std::string& windowName);

  static log4cxx::LoggerPtr logger;

  private:
    static void cleanupDisplayWindows();

    static Display2D_Map displays2d;
    static Display3D_Map displays3d;
    static boost::mutex display_mutex;

  };

  class DisplayWindow2D {
  public:
    DisplayWindow2D(const std::string& name);
    ~DisplayWindow2D();

    void setDisplayFrame(const cv::Mat& newFrameToDraw);
    bool display();
    std::string getName() const;
    
  private:
    const std::string windowName;
    bool initialized; //if cvNamedWindow has been called yet
    cv::Mat frameToDraw;
  };

  class DisplayWindow3D {
  public:
    DisplayWindow3D(const std::string& name);
    ~DisplayWindow3D();

    std::string getName() const;
    void setPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudToDisplay, const std::string& cloudName);
    void setPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudToDisplay, const std::string& cloudName);
    void setPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudToDisplay, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::ConstPtr handler, const std::string& cloudName);
    void setPolygonMesh(pcl::PolygonMesh::ConstPtr meshToDisplay, const std::string& meshName);
    void setSphere(const pcl::PointXYZ center, const double& radius, const std::string& sphereName);
    void setPlane(const pcl::ModelCoefficients& coeffs, const std::string& planeName);
    void removePointCloud(const std::string& cloudName);
    void removePolygonMesh(const std::string& meshName);
    void removeSphere(const std::string& sphereName);
    void removePlane(const std::string& planeName);
    bool display();


  private:
    typedef boost::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>::ConstPtr> Cloud_Map;
    typedef boost::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> ColorCloud_Map;
    typedef boost::unordered_map<std::string, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::ConstPtr> CloudHandler_Map;
    typedef boost::unordered_map<std::string, pcl::PolygonMesh::ConstPtr> Mesh_Map;
    typedef boost::unordered_map<std::string, pcl::ModelCoefficients::ConstPtr> Shape_Map;

    const std::string windowName;
    bool initialized; //
    pcl::visualization::PCLVisualizer* viewer;
    Cloud_Map clouds_add;
    Cloud_Map clouds_update;
    std::vector<std::string> clouds_remove;
    ColorCloud_Map colorClouds_add;
    ColorCloud_Map colorClouds_update;
    CloudHandler_Map cloud_handlers;
    Mesh_Map meshes_add;
    Mesh_Map meshes_update;
    std::vector<std::string> meshes_remove;
    Shape_Map spheres_add;
    Shape_Map spheres_update;
    std::vector<std::string> spheres_remove;
    Shape_Map planes_add;
    Shape_Map planes_update;
    std::vector<std::string> planes_remove;
  };

} //namespace ade

#endif  //DISPLAY_HPP
