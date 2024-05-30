/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "MemoryObjectInterface.hpp"
#include "visionproc/VisionProcess.hpp"
#include <stdio.h>
//NOTE: To get the signatures for the Java functions, use this on the Class:
// javap -p -s java/util/Vector
// and substitute the class path in place of java/util/Vector

// jclass MemoryObjectInterface::memoryObjectClass;
//  jmethodID MemoryObjectInterface::memoryObject_constructor;
//  //set methods
//  jmethodID MemoryObjectInterface::memoryObject_settype;
//  jmethodID MemoryObjectInterface::memoryObject_settokenid;
//  jmethodID MemoryObjectInterface::memoryObject_setframenum;
//  jmethodID MemoryObjectInterface::memoryObject_setconfidence;
//  //get methods
//  jmethodID MemoryObjectInterface::memoryObject_gettype;
//  jmethodID MemoryObjectInterface::memoryObject_gettokenid;
//  jmethodID MemoryObjectInterface::memoryObject_getframenum;
//  jmethodID MemoryObjectInterface::memoryObject_getconfidence;

using namespace diarc::stm;

log4cxx::LoggerPtr MemoryObjectInterface::logger = log4cxx::Logger::getLogger("diarc.stm.MemoryObjectInterface");

MemoryObjectInterface::MemoryObjectInterface() {
}

void MemoryObjectInterface::initialize(JNIEnv* newEnv, jobject obj) {
  initialize(newEnv);
  memoryObject = obj;
}

void MemoryObjectInterface::initialize(JNIEnv* newEnv) {

  env = newEnv;
  //if (memoryObjectClass == NULL) {
  memoryObjectClass = env->FindClass("edu/tufts/hrilab/vision/stm/MemoryObject");
  if (memoryObjectClass == NULL) {
    printf("[MemoryObjectInterface::initialize]: Error! FindClass\n");
    return;
  }
  //}
  //if (memoryObject_constructor == NULL) {
  memoryObject_constructor = env->GetMethodID(memoryObjectClass, "<init>", "()V");
  if (memoryObject_constructor == NULL) {
    printf("[MemoryObjectInterface::initialize]: Error! <init>\n");
    return;
  }
  //}

  memoryObject = env->NewObject(memoryObjectClass, memoryObject_constructor, NULL);

  //if (memoryObject_settype == NULL) {
  memoryObject_settypeid = env->GetMethodID(memoryObjectClass, "setTypeId", "(J)V");
  if (memoryObject_settypeid == NULL)
    printf("[MemoryObjectInterface::initialize]: Error! addTypeId\n");
  //}

  //if (memoryObject_settokenid == NULL) {
  memoryObject_settokenid = env->GetMethodID(memoryObjectClass, "setTokenId", "(J)V");
  if (memoryObject_settokenid == NULL)
    printf("[MemoryObjectInterface::initialize]: Error! setId\n");
  //}

  //if (memoryObject_setframenum == NULL) {
  memoryObject_setframenum = env->GetMethodID(memoryObjectClass, "setFrameNum", "(J)V");
  if (memoryObject_setframenum == NULL)
    printf("[MemoryObjectInterface::initialize]: Error! setFrameNum\n");
  //}

  memoryObject_setvariable = env->GetMethodID(memoryObjectClass, "setVariable", "(Ljava/lang/String;)V");
  if (memoryObject_setvariable == NULL)
    printf("[MemoryObjectInterface::initialize]: Error! memoryObject_setvariable\n");

  //if (memoryObject_setconfidence == NULL) {
  memoryObject_setdetectionconfidence = env->GetMethodID(memoryObjectClass, "setDetectionConfidence", "(D)V");
  if (memoryObject_setdetectionconfidence == NULL)
    printf("[MemoryObjectInterface::initialize]: Error! setDetectionConfidence\n");
  //}

  //if (memoryObject_setconfidence == NULL) {
  memoryObject_settrackingconfidence = env->GetMethodID(memoryObjectClass, "setTrackingConfidence", "(D)V");
  if (memoryObject_settrackingconfidence == NULL)
    printf("[MemoryObjectInterface::initialize]: Error! setTrackingConfidence\n");
  //}

  //if (memoryObject_setlocation == NULL) {
  memoryObject_setlocation = env->GetMethodID(memoryObjectClass, "setLocation", "(DDD)V");
  if (memoryObject_setlocation == NULL)
    printf("[MemoryObjectInterface::initialize]: Error! setLocation\n");
  //}

  //if (memoryObject_setdirection == NULL) {
  memoryObject_setdirection = env->GetMethodID(memoryObjectClass, "setDirection", "(DDD)V");
  if (memoryObject_setdirection == NULL)
    printf("[MemoryObjectInterface::initialize]: Error! setDirection\n");
  //}

  //if (memoryObject_setbb == NULL) {
  memoryObject_setbb = env->GetMethodID(memoryObjectClass, "setBoundingBox", "(IIII)V");
  if (memoryObject_setbb == NULL)
    printf("[MemoryObjectInterface::initialize]: Error! setBoundingBox\n");
  //}

  memoryObject_setdimensions = env->GetMethodID(memoryObjectClass, "setDimensions", "(DDD)V");
  if (memoryObject_setdimensions == NULL)
    printf("[MemoryObjectInterface::initialize]: Error! setDimensions\n");

  memoryObject_setNumPoints = env->GetMethodID(memoryObjectClass, "setNumPoints", "(I)V");
  if (memoryObject_setNumPoints == NULL)
    printf("[MemoryObjectInterface::initialize]: Error! setNumPoints\n");

  memoryObject_addPoint = env->GetMethodID(memoryObjectClass, "addPoint", "(IDDD)V");
  if (memoryObject_addPoint == NULL)
    printf("[MemoryObjectInterface::initialize]: Error! addPoint\n");

  memoryObject_setNumFaces = env->GetMethodID(memoryObjectClass, "setNumFaces", "(I)V");
  if (memoryObject_setNumFaces == NULL)
    printf("[MemoryObjectInterface::initialize]: Error! setNumFaces\n");

  memoryObject_addFace = env->GetMethodID(memoryObjectClass, "addFace", "(I[I)V");
  if (memoryObject_addFace == NULL)
    printf("[MemoryObjectInterface::initialize]: Error! addFace\n");

  memoryObject_setNumOrientations = env->GetMethodID(memoryObjectClass, "setNumOrientations", "(I)V");
  if (memoryObject_setNumOrientations == NULL)
    printf("[MemoryObjectInterface::initialize]: Error! setNumOrientations\n");

  memoryObject_setNumMaskIndices = env->GetMethodID(memoryObjectClass, "setNumMaskIndices", "(I)V");
  if (memoryObject_setNumMaskIndices == NULL)
    printf("[MemoryObjectInterface::initialize]: Error! setNumMaskIndices\n");

  memoryObject_addMaskIndex = env->GetMethodID(memoryObjectClass, "addMaskIndex", "(II)V");
  if (memoryObject_addMaskIndex == NULL)
    printf("[MemoryObjectInterface::initialize]: Error! addMaskIndex\n");

  memoryObject_addOrientation = env->GetMethodID(memoryObjectClass, "addOrientation", "(IDDDD)V");
  if (memoryObject_addOrientation == NULL)
    printf("[MemoryObjectInterface::initialize]: Error! addOrientation\n");

  memoryObject_setbasetransform = env->GetMethodID(memoryObjectClass, "setBaseTransform", "([D)V");
  if (memoryObject_setbasetransform == NULL)
    printf("[MemoryObjectInterface::initialize]: Error! setBaseTransform\n");

  memoryObject_addDescriptor = env->GetMethodID(memoryObjectClass, "addDescriptor", "(Ljava/lang/String;F)V");
  if (memoryObject_addDescriptor == NULL)
    printf("[MemoryObjectInterface::initialize]: Error! addDescriptor\n");

  memoryObject_addChild = env->GetMethodID(memoryObjectClass, "addChild", "(Ledu/tufts/hrilab/vision/stm/MemoryObject;)V");
  if (memoryObject_addChild == NULL)
    printf("[MemoryObjectInterface::initialize]: Error! addChild\n");

  memoryObject_addRelation = env->GetMethodID(memoryObjectClass, "addRelation", "(FLjava/lang/String;Ledu/tufts/hrilab/vision/stm/MemoryObject;Z)V");
  if (memoryObject_addRelation == NULL)
    printf("[MemoryObjectInterface::initialize]: Error! addRelation\n");

  //if (memoryObject_gettype == NULL) {
  memoryObject_gettypeid = env->GetMethodID(memoryObjectClass, "getTypeId", "()J");
  if (memoryObject_gettypeid == NULL)
    printf("[MemoryObjectInterface::initialize]: Error! getTypeId\n");
  //}

  //if (memoryObject_gettokenid == NULL) {
  memoryObject_gettokenid = env->GetMethodID(memoryObjectClass, "getTokenId", "()J");
  if (memoryObject_gettokenid == NULL)
    printf("[MemoryObjectInterface::initialize]: Error! getId\n");
  //}

  // if (memoryObject_getframenum == NULL) {
  memoryObject_getframenum = env->GetMethodID(memoryObjectClass, "getFrameNum", "()J");
  if (memoryObject_getframenum == NULL)
    printf("[MemoryObjectInterface::initialize]: Error! getFrameNum\n");
  //}

  //if (memoryObject_getconfidence == NULL) {
  memoryObject_getdetectionconfidence = env->GetMethodID(memoryObjectClass, "getDetectionConfidence", "()D");
  if (memoryObject_getdetectionconfidence == NULL)
    printf("[MemoryObjectInterface::initialize]: Error! getDetectionConfidence\n");
  //}

  //if (memoryObject_getconfidence == NULL) {
  memoryObject_gettrackingconfidence = env->GetMethodID(memoryObjectClass, "getTrackingConfidence", "()D");
  if (memoryObject_gettrackingconfidence == NULL)
    printf("[MemoryObjectInterface::initialize]: Error! getTrackingConfidence\n");
  //}
}

jobject MemoryObjectInterface::getJavaObject() const {
  return memoryObject;
}

void MemoryObjectInterface::fillJavaMemoryObject(const MemoryObject::ConstPtr& current) {
  std::tr1::unordered_map<long long, MemoryObjectInterface> sceneGraphNodes;
  fillJavaMemoryObject(current, sceneGraphNodes);
}

void MemoryObjectInterface::fillJavaMemoryObject(const MemoryObject::ConstPtr& current,
        std::tr1::unordered_map<long long, MemoryObjectInterface>& sceneGraphNodes) {
  LOG4CXX_TRACE(logger, "[fillJavaMemoryObject] method entered.");

  // PWS: REMEMBER--this data structure gets copied in MemoryObject.java, so if you add a field, you need to copy it there...

  //lock memory object so all data is from the same update iteration
  current->lock();

  setTypeId(current->getTypeId());
  setTokenId(current->getId());
  setVariable(current->getVariableName());
  setDetectionConfidence(current->getDetectionConfidence());
  setTrackingConfidence(current->getTrackingConfidence());
  setFrameNum(current->getTrackingMask()->getFrameNumber());
  const cv::Point3d& location = current->getTrackingMask()->getLocation();
  setLocation(location.x, location.y, location.z);
  const cv::Vec3d& direction = current->getTrackingMask()->getDirection();
  setDirection(direction[0], direction[1], direction[2]);
  const cv::Rect& bb = current->getTrackingMask()->getBoundingBox();
  setBoundingBox(bb.x, bb.y, bb.width, bb.height);
  const cv::Point3d& dimensions = current->getTrackingMask()->getDimensions();
  setDimensions(dimensions.x, dimensions.y, dimensions.z);
  setBaseTransform(current->getTrackingMask()->getTransform());

  // add mask
  setImageMask(current->getDetectionMask()->getIndicesMask());

  //if there's wireframe info, pass that too...
  //if memory object is PointCloudObject
  PointCloudObject::ConstPtr current_pcObject = boost::dynamic_pointer_cast<const PointCloudObject > (current);
  if (current_pcObject) {
    //and has a wireframe
    if (boost::shared_ptr<std::vector< pcl::Vertices > >() != current_pcObject->getWireframePolygons()) {
      LOG4CXX_DEBUG(logger, "[fillRegularMemoryObject] setting mesh.");
      setMesh(current_pcObject->getTrackingMask()->getObjectPointCloud(), *(current_pcObject->getWireframePolygons()));
    } else {
      LOG4CXX_DEBUG(logger, "[fillRegularMemoryObject] NOT setting mesh.");
    }
  } else if (current->getCaptureData()->hasCloudData()) {
    // just pass point cloud
    setPointCloud(current->getTrackingMask()->getUnorganizedObjectPointCloud());
  }


  //set descriptors
  //TODO: replace with updated approach
  ValidationResults validationResults = current->getValidationResults();
  const ValidationResult::VecByPredicate& validationResultsByPred = validationResults.getResults();
  ValidationResult::VecByPredicate::const_iterator iter;
  for (iter = validationResultsByPred.begin(); iter != validationResultsByPred.end(); ++iter) {
    addDescriptor(iter->first.toString(), validationResults.getConfidence(iter->first));
  }

  // add "this" to sceneGraphNodes
  sceneGraphNodes.insert(std::tr1::unordered_map<long long, MemoryObjectInterface>::value_type(current->getId(), *this));

  // add children
  LOG4CXX_TRACE(logger, boost::format("[fillJavaMemoryObject] adding children for %lld.") % current->getId());
  MemoryObject::Vec children = current->getChildren();
  MemoryObject::Vec::const_iterator children_itr;
  for (children_itr = children.begin(); children_itr != children.end(); ++children_itr) {
    addChild(*children_itr, sceneGraphNodes);
  }

  // add relations
  LOG4CXX_TRACE(logger, boost::format("[fillJavaMemoryObject] adding relations for %lld.") % current->getId());
  const RelationValidationResult::Vec relations = current->getRelations();
  RelationValidationResult::Vec::const_iterator relations_itr;
  for (relations_itr = relations.begin(); relations_itr != relations.end(); ++relations_itr) {
    MemoryObject::ConstPtr relatedObject = (*relations_itr)->getRelatedObject();
    if (relatedObject) {
      addRelation((*relations_itr)->getConfidence(), (*relations_itr)->getDescriptor().toString(), relatedObject,
                  sceneGraphNodes);
    }
  }

  //unlock memory object
  current->unlock();

  LOG4CXX_TRACE(logger, boost::format("[fillJavaMemoryObject] done filling java MO for %lld.") % current->getId());
}

void MemoryObjectInterface::setTypeId(const long long& typeId) {
  env->CallVoidMethod(memoryObject, memoryObject_settypeid, typeId);
}

void MemoryObjectInterface::setTokenId(const long long& tokenId) {
  env->CallVoidMethod(memoryObject, memoryObject_settokenid, tokenId);
}

void MemoryObjectInterface::setFrameNum(const long long& fNum) {
  env->CallVoidMethod(memoryObject, memoryObject_setframenum, fNum);
}

void MemoryObjectInterface::setVariable(const std::string& variableName) {
  jstring variableName_jstr = env->NewStringUTF(variableName.c_str());
  env->CallVoidMethod(memoryObject, memoryObject_setvariable, variableName_jstr);
}

void MemoryObjectInterface::setDetectionConfidence(const double& conf) {
  env->CallVoidMethod(memoryObject, memoryObject_setdetectionconfidence, conf);
}

void MemoryObjectInterface::setTrackingConfidence(const double& conf) {
  env->CallVoidMethod(memoryObject, memoryObject_settrackingconfidence, conf);
}

void MemoryObjectInterface::setLocation(const double& x, const double& y, const double& z) {
  env->CallVoidMethod(memoryObject, memoryObject_setlocation, (jdouble) x, (jdouble) y, (jdouble) z);
}

void MemoryObjectInterface::setDirection(const double& x, const double& y, const double& z) {
  env->CallVoidMethod(memoryObject, memoryObject_setdirection, (jdouble) x, (jdouble) y, (jdouble) z);
}

void MemoryObjectInterface::setBoundingBox(const int x, const int y, const int width, const int height) {
  env->CallVoidMethod(memoryObject, memoryObject_setbb, x, y, width, height);
}

void MemoryObjectInterface::setDimensions(const double& x, const double& y, const double& z) {
  env->CallVoidMethod(memoryObject, memoryObject_setdimensions, (jdouble) x, (jdouble) y, (jdouble) z);
}

void MemoryObjectInterface::setImageMask(const std::vector<int>& imageMask) {

  //add indices to java object
  env->CallVoidMethod(memoryObject, memoryObject_setNumMaskIndices, (jint) imageMask.size());
  for (size_t i = 0; i < imageMask.size(); ++i) {
    int maskIndex = imageMask[i];
    env->CallVoidMethod(memoryObject, memoryObject_addMaskIndex, (jint) i, (jint) maskIndex);
  }
}

void MemoryObjectInterface::setPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr points) {
  //printf("MemoryObjectInterface setPointCloud: points: %d \n", points.size());

  //add vertices to java object
  pcl::PointXYZ point;
  env->CallVoidMethod(memoryObject, memoryObject_setNumPoints, (jint) points->size());
  for (size_t i = 0; i < points->size(); ++i) {
    point = points->points[i];
    //TODO: check for NaNs
    env->CallVoidMethod(memoryObject, memoryObject_addPoint, (jint) i, (jdouble) point.x, (jdouble) point.y, (jdouble) point.z);
  }
}

void MemoryObjectInterface::setMesh(pcl::PointCloud<pcl::PointXYZ>::ConstPtr points, const std::vector<pcl::Vertices>& polygons) {
  //printf("MemoryObjectInterface setMesh: points: %d polygons: %d\n", points.size(), polygons.size());

  //add vertices to java object
  pcl::PointXYZ point;
  env->CallVoidMethod(memoryObject, memoryObject_setNumPoints, (jint) points->size());
  for (size_t i = 0; i < points->size(); ++i) {
    point = points->points[i];
    env->CallVoidMethod(memoryObject, memoryObject_addPoint, (jint) i, (jdouble) point.x, (jdouble) point.y, (jdouble) point.z);
  }

  //add polygons (indices into vertices array) to java object
  pcl::Vertices polygon;
  env->CallVoidMethod(memoryObject, memoryObject_setNumFaces, (jint) polygons.size());
  for (size_t i = 0; i < polygons.size(); ++i) {
    polygon = polygons[i];
    if (polygon.vertices.size() < 3) {
      printf("[MemoryObjectInterfac::setMesh] WARNING less than 3 points in polygon. Ignoring polygon!\n");
    } else {
      jsize num_vertices = (jsize) polygon.vertices.size();
      jintArray j_polygon = env->NewIntArray(num_vertices);
      jint vertex_buffer[num_vertices];
      for (jsize j = 0; j < num_vertices; ++j) {
        vertex_buffer[j] = (jint) polygon.vertices[j];
      }
      env->SetIntArrayRegion(j_polygon, 0, num_vertices, vertex_buffer);
      env->CallVoidMethod(memoryObject, memoryObject_addFace, (jint) i, j_polygon);
    }
  }
}

void MemoryObjectInterface::setNumOrientations(const int size) {
  env->CallVoidMethod(memoryObject, memoryObject_setNumOrientations, (jint) size);
}

void MemoryObjectInterface::setOrientation(const int index, const double& x, const double& y, const double& z, const double& w) {
  LOG4CXX_DEBUG(logger, boost::format("[setOrientation] index: %d. orientation: (%f,%f,%f,%f).") % index % x % y % z % w);
  env->CallVoidMethod(memoryObject, memoryObject_addOrientation, (jint) index, (jdouble) x, (jdouble) y, (jdouble) z, (jdouble) w);
}

void MemoryObjectInterface::setBaseTransform(const cv::Mat& transform) {
  jsize num_points = 16;
  jdouble transform_buffer[num_points];
  for (jsize i = 0; i < num_points; ++i) {
    transform_buffer[i] = (jdouble) transform.ptr<double>(0)[i];
  }
  jdoubleArray j_transform = env->NewDoubleArray(num_points);
  env->SetDoubleArrayRegion(j_transform, 0, num_points, transform_buffer);
  env->CallVoidMethod(memoryObject, memoryObject_setbasetransform, j_transform);
}

void MemoryObjectInterface::addDescriptor(const std::string& descriptor, const float& confidence) {
  jstring descriptor_jstr = env->NewStringUTF(descriptor.c_str());
  env->CallVoidMethod(memoryObject, memoryObject_addDescriptor, descriptor_jstr, (jfloat) confidence);
}

void MemoryObjectInterface::addChild(const MemoryObject::ConstPtr& mo, std::tr1::unordered_map<long long, MemoryObjectInterface>& sceneGraphNodes) {
  MemoryObjectInterface moInterface = MemoryObjectInterface();
  moInterface.initialize(env);
  moInterface.fillJavaMemoryObject(mo, sceneGraphNodes);
  env->CallVoidMethod(memoryObject, memoryObject_addChild, moInterface.getJavaObject());
}

void MemoryObjectInterface::addRelation(const float& confidence, const std::string& descriptor, const MemoryObject::ConstPtr& mo,
        std::tr1::unordered_map<long long, MemoryObjectInterface>& sceneGraphNodes) {
  jstring descriptor_jstr = env->NewStringUTF(descriptor.c_str());
  long long id = mo->getId();
  std::tr1::unordered_map<long long, MemoryObjectInterface>::iterator node_itr = sceneGraphNodes.find(id);
  if (node_itr == sceneGraphNodes.end()) {
    // relation node hasn't been instantiated yet -- will create Java relation
    // when the other node is encountered in the scene graph traverse
    LOG4CXX_DEBUG(logger, boost::format("[fillJavaMemoryObject] can't add relation %s to %lld yet.") % descriptor % id);
    return;
  }

  LOG4CXX_TRACE(logger, boost::format("[fillJavaMemoryObject] adding relation %s to %lld.") % descriptor % id);
  env->CallVoidMethod(memoryObject, memoryObject_addRelation, (jfloat) confidence, descriptor_jstr, node_itr->second.getJavaObject(), (jboolean) true);
}

long long MemoryObjectInterface::getTypeId() {
  return env->CallIntMethod(memoryObject, memoryObject_gettypeid);
}

long long MemoryObjectInterface::getTokenId() {
  return env->CallIntMethod(memoryObject, memoryObject_gettokenid);
}

long long MemoryObjectInterface::getFrameNum() {
  return env->CallLongMethod(memoryObject, memoryObject_getframenum);
}

double MemoryObjectInterface::getDetectionConfidence() {
  return env->CallDoubleMethod(memoryObject, memoryObject_getdetectionconfidence);
}

double MemoryObjectInterface::getTrackingConfidence() {
  return env->CallDoubleMethod(memoryObject, memoryObject_gettrackingconfidence);
}
