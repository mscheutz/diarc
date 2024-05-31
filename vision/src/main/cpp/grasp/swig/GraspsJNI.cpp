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

JNIEXPORT jobject JNICALL Java_edu_tufts_hrilab_vision_grasp_swig_GraspsJNI_calculateGraspPoses(JNIEnv* env, jclass cls, jobject mo) {
  MemoryObjectInterface moJNI;
  moJNI.initialize(env, mo);
  MemoryObject memoryObject = moJNI.

  ArrayListInterface graspsJNI;
  graspsJNI.initialize(env);
  std::vector<diarc::grasp::Grasp> grasps = diarc::grasp::GraspDetector::getInstance()->calculateGraspOptions(memoryObject, env);
  for (auto grasp : grasps) {
    GraspInterface graspJNI;
    graspJNI.initialize(env);
    int i = 0;
    for (auto point : grasp.points) {
      graspJNI.setPoint(i++, point.x, point.y, point.z);
    }
    graspJNI.setOrientation(grasp.orientation.x(), grasp.orientation.y(), grasp.orientation.z(), grasp.orientation.w());
    graspsJNI.add(graspJNI.getJavaObject());
  }
  return graspsJNI.getJavaObject();
}
