/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef TRACKEDOBJECTS_HPP
#define TRACKEDOBJECTS_HPP

#include <boost/shared_ptr.hpp>       //shared_ptr
#include <boost/thread/mutex.hpp>
#include <jni.h>
#include <log4cxx/logger.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <tr1/unordered_map> //<hash_map>
#include <tr1/unordered_set> //<hash_set>

#include "stm/MemoryObject.hpp"
#include "stm/MemoryObjectInterface.hpp"             /* Interface with Java data struct */
#include "stm/ArrayListInterface.hpp"                /* Data struct to send */


//define hash fcn and equal_to predicate for class MemoryObject::Ptr
namespace std {
  namespace tr1 {

    template<>
    struct hash<ade::stm::MemoryObject::Ptr> : public unary_function<ade::stm::MemoryObject::Ptr, size_t> {

      size_t operator()(const ade::stm::MemoryObject::Ptr& mo) const {
        return static_cast<size_t> (mo->getId());
      }
    };
  }
}

// forward declare friend class from another namespce
class RelationTracker;

namespace ade {
  namespace stm {

    struct memobj_equal_to : std::binary_function<MemoryObject::Ptr, MemoryObject::Ptr, bool> {
    public:

      bool operator()(const MemoryObject::Ptr& mo1, const MemoryObject::Ptr& mo2) const {
        return (mo1->getId() == mo2->getId());
      }
    };


    //! unordered set of memory objects hashed by their tokenIds
    typedef std::tr1::unordered_set<MemoryObject::Ptr, std::tr1::hash<MemoryObject::Ptr>, memobj_equal_to> MemoryObjects_HashedById_Set;
    //! unordered map of memory objects hashed by their tokenIds
    typedef std::tr1::unordered_map<long long, MemoryObject::Ptr> MemoryObjects_HashedById_Map;
    //! unordered map of lists of memory objects hashed by their typeIds
    typedef std::tr1::unordered_map<long long, MemoryObjects_HashedById_Set> MemoryObjectHashSet_HashedByType_Map;

    class TrackedObjects {
    public:
      static TrackedObjects* getInstance();
      static void deleteInstance();

      //can modify the underlying MemoryObjects with these methods, but must call above add/remove methods to modify hash containers
      //TODO: verify modification rules are enforced by const qualifiers
      MemoryObject::Ptr getById(const long long& id) const;
      MemoryObjects_HashedById_Set getByTypeId(const long long& typeId) const;
      int getSizeTypeId(const long long& typeId) const;

      //native -> java methods
      void getMemoryObjects(ArrayListInterface& toSend, const double& conf, JNIEnv* env) const;
      void getMemoryObjectsByTypeId(ArrayListInterface& toSend, const long long& typeId, const double& conf, JNIEnv* env) const;
      void getMemoryObjectsByTokenId(ArrayListInterface& toSend, const long long& tokenId, const double& conf, JNIEnv* env) const;
      void getMemoryObjectTypeIds(jlongArray& toSend, const double& conf, JNIEnv* env) const;
      void getMemoryObjectIds(jlongArray& toSend, const double& conf, JNIEnv* env) const;
      void getMemoryObjectIds(jlongArray& toSend, const long long& typeId, const double& conf, JNIEnv* env) const;
      void getMemoryObject(MemoryObjectInterface& toSend, const long long& tokenId, const double& conf, JNIEnv* env) const;
      bool confirmMemoryObject(const long long& tokenId) const;
      void getRemovedMemoryObjects(ArrayListInterface& toSend, JNIEnv* env);
      //end native -> java methods

    private:
      TrackedObjects();
      ~TrackedObjects();

      //only locks on hash containers. does not protect underlying memoryObjects.
      mutable boost::mutex trackedObjectsMutex;

      /**
       * @brief Add new MO to ALL storage containers.
       * @param newMemObj object to add
       */
      void add(MemoryObject::Ptr newMemObj);
      /**
       * Add existing MO to typeId container.
       * @param existingMemObj
       * @param typeId
       */
      void add(const MemoryObject::Ptr& existingMemObj, const long long& typeId);
      /**
       * @brief Remove MO from ALL storage containers.
       * @param existingMemObj object to remove
       */
      void remove(const MemoryObject::Ptr& existingMemObj);
      /**
       * @brief Remove object from tracker only as particular typeId. MO can be 
       * multiple types at once. Only removes from tracker completely if it is not
       * being tracked as part of any other type.
       * @param existingMemObj object to remove from type collection
       * @param typeId id of type that object is being removed from
       */
      void remove(const MemoryObject::Ptr& existingMemObj, const long long& typeId);
      /**
       * @brief Remove ALL MO of specified type from ALL containers.
       * @param ypeId
       */
      //void remove(const long long& typeId);

      //stored MemoryObjects

      //memoryObjects hashed by memory object token id
      MemoryObjects_HashedById_Map mo_tokenId;
      //memoryObjects hash_sets hashed by object type id
      MemoryObjectHashSet_HashedByType_Map mo_typeId;

      static boost::mutex instance_mutex;
      static TrackedObjects* instance;
      
      //! MemoryObjects add/remove themselves to/from the tracking queue
      friend class MemoryObject;

      //! RelationTracker needs to add/remove objects from tracking containers by typeId only
      friend class ::RelationTracker;

      log4cxx::LoggerPtr logger;

      std::vector<MemoryObject::Ptr> removedMOBuffer;
    };

  } //namespace stm
} //namespace ade

#endif //TRACKEDOBJECTS_HPP
