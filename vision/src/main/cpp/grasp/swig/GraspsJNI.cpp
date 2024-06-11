/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "GraspsJNI.hpp"


#include "stm/MemoryObjectInterface.hpp"           // Interface with Java data struct
#include "stm/ArrayListInterface.hpp"                 // Interface with Java data struct.  can use for ArrayList of any java object
#include "stm/MemoryObject.hpp"
#include "grasp/Grasp.hpp"
#include "grasp/GraspInterface.hpp"
#include "grasp/GraspDetector.hpp"

using namespace diarc::stm;

JNIEXPORT jobject JNICALL Java_edu_tufts_hrilab_vision_grasp_swig_GraspDetectorModuleJNI_calculateGraspPoses(JNIEnv* env, jclass cls, jdoubleArray transform, jdoubleArray pointcloud, jint width, jint height) {

  jsize len = env->GetArrayLength(pointcloud);
  jdouble* cloud_data = env->GetDoubleArrayElements(pointcloud, 0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>((int) width, (int) height));
  int numPoints = width * height;
  for (int n=0; n<numPoints; n++) {
    pcl::PointXYZ& point = cloud->at(n);
    int i = n*3;
    point.x = cloud_data[i];
    point.y = cloud_data[i+1];
    point.z = cloud_data[i+2];
  }
  std::cout << "point cloud values: " << std::endl;
  for (const auto& p: cloud->points) {
    std::cout << "point " << p << std::endl;
  }
  env->ReleaseDoubleArrayElements(pointcloud, cloud_data, 0);

  jdouble* data = env->GetDoubleArrayElements(transform, 0);
  cv::Mat transform_matrix = cv::Mat_<double>(4,4);
  for (int i=0; i < 16; ++i) {
    transform_matrix.at<double>(i/4,i%4) = data[i];
  }
  std::cout << "trasnform matrix " << transform_matrix << std::endl;
  env->ReleaseDoubleArrayElements(transform, data, 0);

  ArrayListInterface graspsJNI;
  graspsJNI.initialize(env);
  MemoryObject::Ptr memoryObject;
  std::vector<diarc::grasp::Grasp> grasps = diarc::grasp::GraspDetector::getInstance()->calculateGraspOptions(cloud, transform_matrix);
  for (auto grasp : grasps) {
    GraspInterface graspJNI;
    graspJNI.initialize(env);
    int i = 0;
    for (auto point : grasp.points->points) {
      graspJNI.setPoint(i++, point.x, point.y, point.z);
    }
    graspJNI.setOrientation(grasp.orientation.x(), grasp.orientation.y(), grasp.orientation.z(), grasp.orientation.w());
    graspsJNI.add(graspJNI.getJavaObject());
  }
  return graspsJNI.getJavaObject();
}
