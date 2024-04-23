/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "TrackedObjects.hpp"

#include <boost/format.hpp>
#include <tr1/unordered_map>
#include <opencv2/core/core.hpp>

using namespace ade::stm;

TrackedObjects* TrackedObjects::instance = NULL;
boost::mutex TrackedObjects::instance_mutex;

// returns the instance of the array with tracked objects 

TrackedObjects* TrackedObjects::getInstance() {
  boost::lock_guard<boost::mutex> lock(instance_mutex);
  if (instance == NULL) {
    instance = new TrackedObjects();
  }
  return instance;
}

void TrackedObjects::deleteInstance() {
  boost::lock_guard<boost::mutex> lock(instance_mutex);
  if (instance != NULL) {
    printf("[deleteInstance]. Deleting instance.");
    delete instance;
    instance = NULL;
  }
}

TrackedObjects::TrackedObjects()
: mo_tokenId(),
mo_typeId(),
removedMOBuffer() {
  logger = log4cxx::Logger::getLogger("ade.stm.TrackedObjects");
}

TrackedObjects::~TrackedObjects() {
}

void TrackedObjects::add(MemoryObject::Ptr newMemObj) {
  if (logger->isDebugEnabled()) {
    LOG4CXX_DEBUG(logger, boost::format("Adding tracked object with id: %lld. types: %lld.")
            % newMemObj->getId() % newMemObj->getTypeId());
  }

  //TODO: can we do a more fine-grained lock?
  boost::lock_guard<boost::mutex> lock(trackedObjectsMutex);

  //add to mo_tokenId
  //Each element is inserted only if its key is not equivalent to the key of 
  //any other element already in the container (keys in an unordered_map are unique).
  mo_tokenId.insert(std::pair<long long, MemoryObject::Ptr> (newMemObj->getId(), newMemObj));

  //add to mo_typeId
  long long typeId = newMemObj->getTypeId();
  MemoryObjectHashSet_HashedByType_Map::iterator mo_typeId_itr;
  //find hash set with given objects type id
  mo_typeId_itr = mo_typeId.find(typeId);
  if (mo_typeId_itr != mo_typeId.end()) {
    (*mo_typeId_itr).second.insert(newMemObj);
  } else {
    //trackedobject type doesn't exist yet
    //create new hash_set for new type
    MemoryObjects_HashedById_Set newTypeHashSet;
    newTypeHashSet.insert(newMemObj);
    mo_typeId[typeId] = newTypeHashSet;
  }
}

void TrackedObjects::add(const MemoryObject::Ptr& existingMemObj, const long long& typeId) {
  if (logger->isDebugEnabled()) {
    LOG4CXX_DEBUG(logger, boost::format("Adding tracked object with id: %lld as type: %lld.")
            % existingMemObj->getId() % typeId);
  }

  //TODO: can we do a more fine-grained lock?
  boost::lock_guard<boost::mutex> lock(trackedObjectsMutex);

  //add to mo_type
  MemoryObjectHashSet_HashedByType_Map::iterator mo_typeId_itr = mo_typeId.find(typeId);
  if (mo_typeId_itr != mo_typeId.end()) {
    mo_typeId_itr->second.insert(existingMemObj);
  } else {
    //trackedobject type doesn't exist yet
    //create new hash_set for new type
    MemoryObjects_HashedById_Set newTypeHashSet;
    newTypeHashSet.insert(existingMemObj);
    mo_typeId[typeId] = newTypeHashSet;
  }
}

void TrackedObjects::remove(const MemoryObject::Ptr& existingMemObj) {
  if (logger->isDebugEnabled()) {
    LOG4CXX_DEBUG(logger, boost::format("Removing tracked object with id: %lld. types: %lld.")
            % existingMemObj->getId() % existingMemObj->getTypeId());
  }

  //TODO: can we do a more fine-grained lock?
  boost::lock_guard<boost::mutex> lock(trackedObjectsMutex);

  //remove from mo_tokenId
  mo_tokenId.erase(existingMemObj->getId());

  //remove from mo_typeId
  MemoryObjectHashSet_HashedByType_Map::iterator mo_typeId_itr;
  mo_typeId_itr = mo_typeId.find(existingMemObj->getTypeId());
  if (mo_typeId_itr != mo_typeId.end()) {
    mo_typeId_itr->second.erase(existingMemObj);
  }

//  removedMOBuffer.push_back(existingMemObj);
}

void TrackedObjects::remove(const MemoryObject::Ptr& existingMemObj, const long long& typeId) {
  if (logger->isDebugEnabled()) {
    LOG4CXX_DEBUG(logger, boost::format("Removing tracked object with id: %lld as type: %lld.")
            % existingMemObj->getId() % typeId);
  }

  //TODO: can we do a more fine-grained lock?
  boost::lock_guard<boost::mutex> lock(trackedObjectsMutex);

  //remove from mo_type
  MemoryObjectHashSet_HashedByType_Map::iterator mo_typeId_itr = mo_typeId.find(typeId);
  mo_typeId_itr->second.erase(existingMemObj);
}

//void TrackedObjects::remove(const long long& typeId) {
//  LOG4CXX_DEBUG(logger, boost::format("Removing all tracked objects of type: %lld.") % typeId);
//  //TODO: can we do a more fine-grained lock?
//  boost::lock_guard<boost::mutex> lock(trackedObjectsMutex);
//  MemoryObjectHashSet_HashedByType_Map::iterator typeItr = mo_typeId.find(typeId);
//  if (typeItr == mo_typeId.end()) {
//    LOG4CXX_WARN(logger, boost::format("Trying to remove type that does not exist: %lld.") % typeId);
//    return;
//  }
//
//  MemoryObjects_HashedById_Set::iterator MOiter;
//  for (MOiter = (*typeItr).second.begin(); MOiter != (*typeItr).second.end(); ++MOiter) {
//    //remove from mo_tokenId
//    long long id = (*MOiter)->getId();
//    mo_tokenId.erase(id);
//  }
//
//  //finally clear types container. doing this within loop will mess up iterator.
//  (*typeItr).second.clear();
//  //EP@: remove type from mo_typeId?
//  mo_typeId.erase(typeItr);
//}

MemoryObject::Ptr TrackedObjects::getById(const long long& id) const {
  boost::lock_guard<boost::mutex> lock(trackedObjectsMutex);

  MemoryObjects_HashedById_Map::const_iterator itr = mo_tokenId.find(id);
  if (itr != mo_tokenId.end()) {
    return itr->second;
  }
  return MemoryObject::Ptr();
}

MemoryObjects_HashedById_Set TrackedObjects::getByTypeId(const long long& typeId) const {
  boost::lock_guard<boost::mutex> lock(trackedObjectsMutex);

  MemoryObjectHashSet_HashedByType_Map::const_iterator itr = mo_typeId.find(typeId);
  if (itr != mo_typeId.end()) {
    return ((*itr).second);
  }
  return MemoryObjects_HashedById_Set();
}

int TrackedObjects::getSizeTypeId(const long long& typeId) const {
  boost::lock_guard<boost::mutex> lock(trackedObjectsMutex);

  MemoryObjectHashSet_HashedByType_Map::const_iterator itr = mo_typeId.find(typeId);
  if (itr != mo_typeId.end()) {
    return ((*itr).second.size());
  }
  return 0;
}

//get all memoryObjects above given confidence threshold

void TrackedObjects::getMemoryObjects(ArrayListInterface& toSend, const double& conf,
        JNIEnv* env) const {
  LOG4CXX_TRACE(logger, boost::format("[getMemoryObjects] method entered. conf thresh: %f") % conf);
  //TODO: can we do a more fine-grained lock?
  boost::lock_guard<boost::mutex> lock(trackedObjectsMutex);

  MemoryObjects_HashedById_Map::const_iterator mo_id_iter;
  for (mo_id_iter = mo_tokenId.begin(); mo_id_iter != mo_tokenId.end(); ++mo_id_iter) {
    const MemoryObject::Ptr& currMO = (*mo_id_iter).second;
    LOG4CXX_TRACE(logger, boost::format("memory object id: %lld. track conf: %f. detect conf: %f.")
            % currMO->getId() % currMO->getTrackingConfidence()
            % currMO->getDetectionConfidence());

    if ((currMO->getTrackingConfidence() >= conf) && (currMO->getDetectionConfidence() >= conf)) {
      MemoryObjectInterface moToAdd;
      moToAdd.initialize(env);
      moToAdd.fillJavaMemoryObject(currMO);
      toSend.add(moToAdd.getJavaObject());
    }
  }
}

//get all memoryObjects above given confidence threshold

void TrackedObjects::getMemoryObjectsByTypeId(ArrayListInterface& toSend, const long long& typeId,
        const double& conf, JNIEnv* env) const {
  //TODO: can we do a more fine-grained lock?
  boost::lock_guard<boost::mutex> lock(trackedObjectsMutex);

  MemoryObjects_HashedById_Set::const_iterator moIter;
  MemoryObjectHashSet_HashedByType_Map::const_iterator typeItr = mo_typeId.find(typeId);

  //if type doesn't exist in tracked objects (yet)
  if (mo_typeId.end() == typeItr) {
    return;
  } else {
    for (moIter = (*typeItr).second.begin(); moIter != (*typeItr).second.end(); ++moIter) {
      if (((*moIter)->getTrackingConfidence() >= conf) &&
              ((*moIter)->getDetectionConfidence() >= conf)) {
        MemoryObjectInterface moToAdd;
        moToAdd.initialize(env);
        moToAdd.fillJavaMemoryObject((*moIter));
        toSend.add(moToAdd.getJavaObject());
      }
    }
  }
}

//get all memoryObjects with specified tokenid (more than one if a single 
//MemoryObject meets constraints of more than one visual search)

void TrackedObjects::getMemoryObjectsByTokenId(ArrayListInterface& toSend, const long long& tokenId,
        const double& conf, JNIEnv* env) const {
  //TODO: can we do a more fine-grained lock?  
  boost::lock_guard<boost::mutex> lock(trackedObjectsMutex);

  MemoryObjects_HashedById_Map::const_iterator mo_id_iter = mo_tokenId.find(tokenId);
  if (mo_id_iter != mo_tokenId.end()) {
    const MemoryObject::Ptr& currMO = mo_id_iter->second;
    if (currMO->getTrackingConfidence() >= conf && currMO->getDetectionConfidence() >= conf) {
      MemoryObjectInterface moToAdd;
      moToAdd.initialize(env);
      moToAdd.fillJavaMemoryObject(currMO);
      toSend.add(moToAdd.getJavaObject());
    }
  }
}

//get all types with at least one tracked object above confidence threshold

void TrackedObjects::getMemoryObjectTypeIds(jlongArray& toSend, const double& conf,
        JNIEnv* env) const {
  //TODO: can we do a more fine-grained lock?
  boost::lock_guard<boost::mutex> lock(trackedObjectsMutex);

  MemoryObjectHashSet_HashedByType_Map::const_iterator typeItr;
  jlong idBuffer[mo_typeId.size()]; //init to max possible size
  int count = 0;
  for (typeItr = mo_typeId.begin(); typeItr != mo_typeId.end(); ++typeItr) {
    MemoryObjects_HashedById_Set::const_iterator moIter;
    for (moIter = typeItr->second.begin(); moIter != typeItr->second.end(); ++moIter) {
      if (((*moIter)->getTrackingConfidence() >= conf) && ((*moIter)->getDetectionConfidence() >= conf)) {
        idBuffer[count++] = (jlong) typeItr->first;
        //only need one of this type above conf thresh
        break;
      }
    }
  }

  toSend = env->NewLongArray((jsize) count);
  env->SetLongArrayRegion(toSend, 0, count, idBuffer);
}

//get all memoryObject ids with confidence above threshold

void TrackedObjects::getMemoryObjectIds(jlongArray& toSend, const double& conf,
        JNIEnv* env) const {
  //TODO: can we do a more fine-grained lock?
  boost::lock_guard<boost::mutex> lock(trackedObjectsMutex);

  MemoryObjects_HashedById_Map::const_iterator mo_id_iter;
  jlong idBuffer[mo_tokenId.size()]; //init to max possible size
  int count = 0;
  for (mo_id_iter = mo_tokenId.begin(); mo_id_iter != mo_tokenId.end(); ++mo_id_iter) {
    const MemoryObject::Ptr& currMO = (*mo_id_iter).second;
    if (currMO->getTrackingConfidence() >= conf && currMO->getDetectionConfidence() >= conf) {
      idBuffer[count++] = (jlong) mo_id_iter->first;
    }
  }

  toSend = env->NewLongArray((jsize) count);
  env->SetLongArrayRegion(toSend, 0, count, idBuffer);
}

//get all memoryObject ids of specified type with confidence above threshold

void TrackedObjects::getMemoryObjectIds(jlongArray& toSend, const long long& typeId,
        const double& conf, JNIEnv* env) const {
  //TODO: can we do a more fine-grained lock?
  boost::lock_guard<boost::mutex> lock(trackedObjectsMutex);

  MemoryObjects_HashedById_Set::const_iterator moIter;
  MemoryObjectHashSet_HashedByType_Map::const_iterator typeItr = mo_typeId.find(typeId);

  //if type doesn't exist in tracked objects (yet)
  if (mo_typeId.end() == typeItr) {
    toSend = env->NewLongArray((jsize) 0);
  } else {
    //init to max possible size
    jlong idBuffer[typeItr->second.size()];
    int count = 0;

    for (moIter = typeItr->second.begin(); moIter != typeItr->second.end(); ++moIter) {
      if (((*moIter)->getTrackingConfidence() >= conf) && ((*moIter)->getDetectionConfidence() >= conf)) {
        idBuffer[count++] = (jlong) (*moIter)->getId();
      }
    }

    //init java array
    toSend = env->NewLongArray((jsize) count);
    //fill java array
    env->SetLongArrayRegion(toSend, 0, count, idBuffer);
  }
}

//get memoryObject with specified tokenid if confidence above threshold
void TrackedObjects::getMemoryObject(MemoryObjectInterface& toSend,
        const long long& tokenId, const double& conf, JNIEnv* env) const {
  //TODO: can we do a more fine-grained lock?  
  boost::lock_guard<boost::mutex> lock(trackedObjectsMutex);

  MemoryObjects_HashedById_Map::const_iterator mo_id_iter = mo_tokenId.find(tokenId);
  if (mo_id_iter != mo_tokenId.end()) {
    const MemoryObject::Ptr& currMO = mo_id_iter->second;
    if (currMO->getTrackingConfidence() >= conf && currMO->getDetectionConfidence() >= conf) {
      toSend.fillJavaMemoryObject(currMO);
    }
  }
}

void TrackedObjects::getRemovedMemoryObjects(ArrayListInterface& toSend, JNIEnv* env) {
  boost::lock_guard<boost::mutex> lock(trackedObjectsMutex);

  for (auto MO : removedMOBuffer) {
      MemoryObjectInterface moToAdd;
      moToAdd.initialize(env);
      moToAdd.fillJavaMemoryObject(MO);
      toSend.add(moToAdd.getJavaObject());
  }

  removedMOBuffer.clear();
}

bool TrackedObjects::confirmMemoryObject(const long long& tokenId) const {
  boost::lock_guard<boost::mutex> lock(trackedObjectsMutex);

  if (mo_tokenId.find(tokenId) == mo_tokenId.end()) {
    return false;
  }

  return true;
}
