/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef MEMORYOBJECTINTERFACE_HPP
#define MEMORYOBJECTINTERFACE_HPP

#include <log4cxx/logger.h>
#include <jni.h>
#include <stdio.h>
#include <map>
#include <unistd.h>
#include <string>
#include <vector>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/Vertices.h>

#include "MemoryObject.hpp"

//interface used to pass native MemoryObjects to Java
//Changes made here MUST be reflected in MemoryObject.java and MemoryObject.hpp (addMemoryObjectToSend(...))

//NOTE: to generate java method signatures: $ javap -s <class>

namespace ade {
  namespace stm {

    class MemoryObjectInterface {
    public:
      MemoryObjectInterface();
      void initialize(JNIEnv* newEnv, jobject obj);
      void initialize(JNIEnv* newEnv);
      jobject getJavaObject() const;

      //creates Java Memory Object and sets appropriate data fields
      void fillJavaMemoryObject(const MemoryObject::ConstPtr& current);
      void fillJavaMemoryObject(const MemoryObject::ConstPtr& current,
              std::tr1::unordered_map<long long, MemoryObjectInterface>& sceneGraphNodes);

    private:
      //set methods
      void setTypeId(const long long& typeId);
      void setTokenId(const long long& tokenId);
      void setFrameNum(const long long& fNum);
      void setVariable(const std::string& variableName);
      void setDetectionConfidence(const double& conf);
      void setTrackingConfidence(const double& conf);
      void setLocation(const double& x, const double& y, const double& z);
      void setDirection(const double& x, const double& y, const double& z);
      void setBoundingBox(const int x, const int y, const int width, const int height);
      void setDimensions(const double& x, const double& y, const double& z);
      void setImageMask(const std::vector<int>& imageMask);
      void setPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr points);
      void setMesh(pcl::PointCloud<pcl::PointXYZ>::ConstPtr points, const std::vector<pcl::Vertices>& polygons);
      void setNumOrientations(const int size);
      void setOrientation(const int index, const double& x, const double& y, const double& z, const double& w);
      void setBaseTransform(const cv::Mat& transform);
      void addDescriptor(const std::string& descriptor, const float& confidence);
      void addChild(const MemoryObject::ConstPtr& mo, std::tr1::unordered_map<long long, MemoryObjectInterface>& sceneGraphNodes);
      void addRelation(const float& confidence, const std::string& descriptor, const MemoryObject::ConstPtr& mo,
              std::tr1::unordered_map<long long, MemoryObjectInterface>& sceneGraphNodes);

      //get methods
      long long getTypeId();
      long long getTokenId();
      long long getFrameNum();
      double getDetectionConfidence();
      double getTrackingConfidence();

      static log4cxx::LoggerPtr logger;

      JNIEnv *env;
      jobject memoryObject;
      //TODO: figure out how to make all these static so class and methodIds don't have to be looked up for every MemoryObjectInterface!
      jclass memoryObjectClass;
      jmethodID memoryObject_constructor;
      //set methods
      jmethodID memoryObject_settypeid;
      jmethodID memoryObject_settokenid;
      jmethodID memoryObject_setframenum;
      jmethodID memoryObject_setvariable;
      jmethodID memoryObject_setdetectionconfidence;
      jmethodID memoryObject_settrackingconfidence;
      jmethodID memoryObject_setlocation;
      jmethodID memoryObject_setdirection;
      jmethodID memoryObject_setbb;
      jmethodID memoryObject_setdimensions;
      jmethodID memoryObject_setbasetransform;
      jmethodID memoryObject_setNumPoints;
      jmethodID memoryObject_addPoint;
      jmethodID memoryObject_setNumFaces;
      jmethodID memoryObject_addFace;
      jmethodID memoryObject_setNumOrientations;
      jmethodID memoryObject_addOrientation;
      jmethodID memoryObject_setNumMaskIndices;
      jmethodID memoryObject_addMaskIndex;
      jmethodID memoryObject_addDescriptor;
      jmethodID memoryObject_addChild;
      jmethodID memoryObject_addRelation;
      //get methods
      jmethodID memoryObject_gettypeid;
      jmethodID memoryObject_gettokenid;
      jmethodID memoryObject_getframenum;
      jmethodID memoryObject_getdetectionconfidence;
      jmethodID memoryObject_gettrackingconfidence;

    };

  } //namespace stm
} //namespace ade

#endif

// jclass memoryObjectClass;
//  jmethodID memoryObject_constructor;
//  //set methods
//  jmethodID memoryObject_settype;
//  jmethodID memoryObject_setid;
//  jmethodID memoryObject_setframenum;
//  jmethodID memoryObject_setconfidence;
//  jmethodID memoryObject_setpan;
//  jmethodID memoryObject_settilt;
//  jmethodID memoryObject_setxcg;
//  jmethodID memoryObject_setycg;
//  jmethodID memoryObject_setoxcg;
//  jmethodID memoryObject_setoycg;
//  jmethodID memoryObject_setul_x;
//  jmethodID memoryObject_setul_y;
//  jmethodID memoryObject_setwidth;
//  jmethodID memoryObject_setheight;
//  jmethodID memoryObject_setdist;
//  jmethodID memoryObject_setcolorr;
//  jmethodID memoryObject_setcolorg;
//  jmethodID memoryObject_setcolorb;
//  jmethodID memoryObject_setcolorid;
//  jmethodID memoryObject_setarea;
//  //get methods
//  jmethodID memoryObject_gettype;
//  jmethodID memoryObject_gettokenid;
//  jmethodID memoryObject_getframenum;
//  jmethodID memoryObject_getconfidence;
//  jmethodID memoryObject_getpan;
//  jmethodID memoryObject_gettilt;
//  jmethodID memoryObject_getxcg;
//  jmethodID memoryObject_getycg;
//  jmethodID memoryObject_getoxcg;
//  jmethodID memoryObject_getoycg;
//  jmethodID memoryObject_getul_x;
//  jmethodID memoryObject_getul_y;
//  jmethodID memoryObject_getwidth;
//  jmethodID memoryObject_getheight;
//  jmethodID memoryObject_getdist;
//  jmethodID memoryObject_getcolorr;
//  jmethodID memoryObject_getcolorg;
//  jmethodID memoryObject_getcolorb;
//  jmethodID memoryObject_getcolorid;
//  jmethodID memoryObject_getarea;

// static jclass memoryObjectClass;
//  static jmethodID memoryObject_constructor;
//  //set methods
//  static jmethodID memoryObject_settype;
//  static jmethodID memoryObject_setid;
//  static jmethodID memoryObject_setframenum;
//  static jmethodID memoryObject_setconfidence;
//  static jmethodID memoryObject_setpan;
//  static jmethodID memoryObject_settilt;
//  static jmethodID memoryObject_setxcg;
//  static jmethodID memoryObject_setycg;
//  static jmethodID memoryObject_setoxcg;
//  static jmethodID memoryObject_setoycg;
//  static jmethodID memoryObject_setul_x;
//  static jmethodID memoryObject_setul_y;
//  static jmethodID memoryObject_setwidth;
//  static jmethodID memoryObject_setheight;
//  static jmethodID memoryObject_setdist;
//  static jmethodID memoryObject_setcolorr;
//  static jmethodID memoryObject_setcolorg;
//  static jmethodID memoryObject_setcolorb;
//  static jmethodID memoryObject_setcolorid;
//  static jmethodID memoryObject_setarea;
//  //get methods
//  static jmethodID memoryObject_gettype;
//  static jmethodID memoryObject_gettokenid;
//  static jmethodID memoryObject_getframenum;
//  static jmethodID memoryObject_getconfidence;
//  static jmethodID memoryObject_getpan;
//  static jmethodID memoryObject_gettilt;
//  static jmethodID memoryObject_getxcg;
//  static jmethodID memoryObject_getycg;
//  static jmethodID memoryObject_getoxcg;
//  static jmethodID memoryObject_getoycg;
//  static jmethodID memoryObject_getul_x;
//  static jmethodID memoryObject_getul_y;
//  static jmethodID memoryObject_getwidth;
//  static jmethodID memoryObject_getheight;
//  static jmethodID memoryObject_getdist;
//  static jmethodID memoryObject_getcolorr;
//  static jmethodID memoryObject_getcolorg;
//  static jmethodID memoryObject_getcolorb;
//  static jmethodID memoryObject_getcolorid;
//  static jmethodID memoryObject_getarea;
