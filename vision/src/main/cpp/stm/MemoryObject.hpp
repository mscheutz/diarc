/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef MEMORYOBJECT_HPP
#define MEMORYOBJECT_HPP

//to enable unorderd_set and unordered_map
#ifndef __IBMCPP_TR1__
#define __IBMCPP_TR1__
#endif

#include "common/fol/PredicateHelper.hpp"
#include "common/VisionConstants.hpp"
#include "common/CaptureData.hpp"
#include "ConfidenceLevel.hpp"
#include "imgproc/siftFeatures/SiftFeatures.hpp"
#include "MemoryObjectMask.hpp"
#include "RelationValidationResult.hpp"
#include "ValidationResults.hpp"

#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_array.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/unordered_set.hpp>
#include <limits>
#include <log4cxx/logger.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <deque>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <tr1/unordered_map>
#include <tr1/unordered_set> //<hash_set>
#include <utility>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/Vertices.h>
#include <pcl/ModelCoefficients.h>

#ifdef USE_V4R_V0
#include <v4r/SurfaceUtils/SurfaceModel.hpp>
#endif //USE_V4R_V0

namespace diarc {
  namespace stm {

    //TODO: should there be separate 2D and 3D MemoryObject base classes?
    //TODO: find a best way to generalize things

    //IMPLEMENTATION NOTES: ALL inheriting classes MUST ensure that no data is 
    //changed in between lock and unlock calls 

    /*
     * struct for assigning unique tracking IDs
     */
    struct ObjectId {

      ObjectId()
              : next_id(0) {
      }

      long getNext() {
        boost::lock_guard<boost::mutex> lock(id_mutex);
        return next_id++;
      };

    private:
      long next_id;
      boost::mutex id_mutex;

    };

    //forward declare to avoid cyclic #includes
    class TrackedObjects;

    class MemoryObject : public boost::enable_shared_from_this<MemoryObject> {
    public:
      typedef boost::shared_ptr<MemoryObject> Ptr;
      typedef boost::shared_ptr<const MemoryObject> ConstPtr;
      typedef std::vector<Ptr> Vec;
      typedef boost::shared_ptr<Vec> VecPtr;

      MemoryObject(const long long &typeID, const std::string &variable, CaptureData::ConstPtr capture,
                   MemoryObjectMask::ConstPtr mask);

      //      MemoryObject(const long long& typeID, const std::string& variable, CaptureData::ConstPtr capture, const cv::Mat& imageMask);
      MemoryObject(const long long &typeID, const std::string &variable, CaptureData::ConstPtr capture,
                   const cv::Rect &boundingBoxMask);

      MemoryObject(const long long &typeID, const std::string &variable, CaptureData::ConstPtr capture,
                   const std::vector<int> &imageIndicesMask);

      //! virtual for inheritance
      virtual ~MemoryObject();

      /**
       * Compare this MemoryObject with passed in object to calculate how likely
       * it is that they are the same object.
       * TODO: have this return double (ie. 0.0 - 1.0), corresponding to strength
       * of the match.
       * @param other MemoryObject to compare to
       * @return 
       */
      virtual double compare(const MemoryObject::Ptr &other) const;

      /**
       * Update this MemoryObject's data with passed in "match".
       * @param match
       */
      virtual void update(const MemoryObject::Ptr &match);

      /**
       * Start tracking object and add it to TrackedObjects containers.
       * @param if MemoryObject is a top-level node in the scene graph being
       * tracked by a tracker
       */
      void startTracking(bool root = true);

      /**
       * Stop tracking object and remove it from TrackedObjects containers.
       * @param if MemoryObject is a top-level node in the scene graph being
       * tracked by a tracker
       */
      void stopTracking(bool root = true);

      /**
       * Is this MemoryObject being tracked.
       * @return 
       */
      bool isTracked() const;

      /**
       * Decay the tracking confidence based on the time that has passed since
       * the last successful update.
       */
      void decayTrackingConfidence();

      /**
       * Set the decay rate for this tracked object. Rate is pre millisecond.
       */
      void setTrackingConfidenceDecayRate(const float& decayRatePerMilsec);

      /**
       * Add new tracking result to the tracking history queue and update
       * the tracking confidence.
       * @param mask - MemoryObjectMask for current tracking frame
       * @param confidence - tracking confidence between [0 1].
       */
      void addNewTrackingResult(MemoryObjectMask::ConstPtr mask, const float &confidence);

      /**
       * Add child MemoryObject as part of scene graph.
       * @param child - child to add to scene graph
       */
      void addChild(MemoryObject::Ptr child);

      /**
       * Remove child MemoryObject as part of scene graph.
       * @param child - child to remove to scene graph
       */
      void removeChild(MemoryObject::Ptr child);

      /**
       * Adds edge between this MemoryObject and another related one as part of scene graph.
       * Also makes symmetric relation from relatedObject (assuming reciprocate is true).
       * @param confidence - probability between [0 1]
       * @param descriptor - predicate description of relation (e.g., on(X,Y), near(Y,Z))
       * @param relatedObject - other MemoryObject with which relation holds
       * @param reciprocate (optional) - if should add relation to relatedObject (all
       * non-MemoryObject callers should use default true value)
       */
      void addRelation(const float &confidence, const PredicateHelper &descriptor, MemoryObject::Ptr relatedObject,
                       bool reciprocate = true);

      /**
      * Decay the relation confidence values for all relations of relationName. If relatedObject is NOT specified, all
      * relations matching relationName will be decayed and their relatedObject will also be notified
      * to decay the symmetric relation (aka recurse == true). If relatedObject is specified, only the relation
      * matching the relationName and relatedObject will be decayed and the symmetric
      * relation will not be decayed (recurse == false).
      * @param relationName
      * @param relatedObject
      */
      void decayRelationConfidence(const std::string &relationName,
                                   const diarc::stm::MemoryObject::Ptr &relatedObject = diarc::stm::MemoryObject::Ptr());

      /**
      * Remove all relations. If relatedObject is NOT specified, all
      * local relations will be removed and their relatedObject will also be notified
      * to remove the symmetric relation (aka recurse == true). If relatedObject IS specified, only the local
      * one-way relation will be removed and the symmetric relation will not be removed (recurse == false).
      * @param relatedObject
      */
      void removeRelations(const diarc::stm::MemoryObject::Ptr &relatedObject = diarc::stm::MemoryObject::Ptr());

      /**
      * Remove all relations of relationName below given threshold. If relatedObject is NOT specified, all
      * relations matching relationName and threshold will be removed and their relatedObject will also be notified
      * to remove the symmetric relation (aka recurse == true). If relatedObject is specified, only the relation
      * matching the relationName, threshold, and relatedObject will be removed and the symmetric
      * relation will not be removed (recurse == false).
      * @param relationName
      * @param confidenceThresh
      * @param relatedObject
      */
      void removeRelations(const std::string &relationName, const float &confidenceThresh,
                           const diarc::stm::MemoryObject::Ptr &relatedObject = diarc::stm::MemoryObject::Ptr());

      /**
      * Remove all relations of relationName. If relatedObject is not specified, all
      * relations matching relationName will be removed and their relatedObject will also be notified
      * to remove the symmetric relation (aka recurse == true). If relatedObject is specified, only the relation
      * matching the relationName and relatedObject will be removed and the symmetric
      * relation will not be removed (recurse == false).
      * @param relationName
      * @param relatedObject
      */
      void removeRelations(const std::string &relationName,
                           const diarc::stm::MemoryObject::Ptr &relatedObject = diarc::stm::MemoryObject::Ptr());

      /**
       * Set the detection/validation confidence from a particular detector/validator.
       * Use this method when the mask is unimportant and the descriptor applies
       * to the entire MemoryObject and not just a portion of it.
       * @param confidence value between [0 1]
       * @param label predicate representation of validation property
       */
      void addValidationResult(const float &confidence, const PredicateHelper &descriptor);

      //! Add a new validation result to this MemoryObject
      void addValidationResult(ValidationResult::ConstPtr validationResult);

      //! Convenience method
      void addValidationResult(const float &confidence, const PredicateHelper &descriptor,
                               const cv::Mat_<float> &imageMask);

      //! Convenience method
      void addValidationResult(const float &confidence, const PredicateHelper &descriptor,
                               const std::vector<int> &indicesMask);

      /**
       * Get unique id (token id).
       * @return token id
       */
      long long getId() const;

      /**
       * Get type id - non-unique search id.
       * @return type id
       */
      long long getTypeId() const;

      /**
       * Get variable name of MemoryObject (e.g., X, Y, etc).
       * @return variable name of MemoryObject
       */
      std::string getVariableName() const;

      /**
      * Get MemoryObjectMask from object detection.
      * @return
      */
      MemoryObjectMask::ConstPtr getDetectionMask() const;

      /**
      * Get MemoryObjectMask from latest object tracking step.
      * @return
      */
      MemoryObjectMask::ConstPtr getTrackingMask() const;

      //! get tracking confidence between [0 1]
      float getTrackingConfidence() const;

      //! get detection confidence between [0 1]
      float getDetectionConfidence() const;

      //! get detection confidence between [0 1] 
      float getDetectionConfidence(const PredicateHelper::Set &descriptors) const;

      /**
       * Get all MemoryObjects 'below' this node in this scene graph that have
       * variableName. Same as getChildren but also checks "this".
       */
      MemoryObject::Vec getMemoryObjects(const std::string &variableName);

      //! get all children (and children of children, etc) of this MemoryObject
      //! matching variable name
      //! TODO: rename this getMemoryObjects
      const MemoryObject::Vec getChildren(const std::string &variableName) const;

      //! get all children of this MemoryObject
      const MemoryObject::Vec getChildren() const;

      //! get all relations (of relation name (e.g., on, near)) of this MemoryObject
      const RelationValidationResult::Vec getRelations(const std::string &relationName) const;

      //! get all relations of this MemoryObject
      const RelationValidationResult::Vec getRelations() const;

      //TODO: make thread safe
      ValidationResults getValidationResults() const;

      //TODO: make thread safe
      ValidationResult::Vec getValidationResults(const PredicateHelper &descriptor) const;

      std::string getValidationResultsString() const; //for debug output

      //! get capture data that was used to detect this object
      CaptureData::ConstPtr getCaptureData() const;

      //! use when calling multiple sets/gets in succession to prevent clients from getting "mismatched" data
      void lock() const;

      void unlock() const;

    protected:

      /**
       * Unique id (token id). -1 if has never been tracked by object tracker,
       * otherwise unique MemoryObject id given by object tracker and
       * cannot be changed.
       */
      long long id;

      long long typeId;

      //! non-unique variable name from logical description
      std::string variableName;

      //! capture data that was used to detect the object
      CaptureData::ConstPtr captureData;

      //! Info about segmented object
      MemoryObjectMask::ConstPtr mask;

      //! all ValidationResults for this memory object
      ValidationResults validationResults;

      //! children memory objects in the scene graph (hashed by variable name)
      std::tr1::unordered_map<std::string, Vec> children;

      //! related memory objects (edges) in the scene graph (hashed by relationship name (e.g., on, near, etc.))
      std::tr1::unordered_map<std::string, RelationValidationResult::Vec> relations;

      //! if object is currently being tracked
      bool isBeingTracked;

      //! the tracking confidence level of the object
      ConfidenceLevel trackingConfidence;

      //! how many past tracking iterations to keep on the trackingHistory Q
      int trackingHistoryLength;

      //! tracking history for last n tracking iterations
      std::deque<MemoryObjectMask::ConstPtr> trackingHistory;

      //TODO: make this a recursive read/write lock, if that's even a thing
      mutable boost::recursive_mutex data_mutex;

      static log4cxx::LoggerPtr logger;

    private:
      //! currently tracked objects
      static TrackedObjects *trackedObjects;

      //! tracking id generator, guarantees all tracked objects in memory have a unique id
      static ObjectId trackingIdGenerator;

    };

    /**
     * Class for representing a detected/tracked object generated using
     * point cloud data (particularly, the PCL libraries)
     */
    class PointCloudObject : public MemoryObject {
    public:
      typedef boost::shared_ptr<PointCloudObject> Ptr;
      typedef boost::shared_ptr<const PointCloudObject> ConstPtr;

      virtual ~PointCloudObject();

      virtual double compare(const MemoryObject::Ptr &other) const;

      virtual void update(const MemoryObject::Ptr &match);

      void setWireframe(std::vector<pcl::Vertices> &polygons);

      void setWireframe(std::vector<pcl::Vertices> &polygons, pcl::ModelCoefficients::Ptr coeffs);

      boost::shared_ptr<std::vector<pcl::Vertices> const> getWireframePolygons() const;

      pcl::ModelCoefficients::ConstPtr getWireframeCoefficients() const;

    protected:

    private:
      //! Indices into object cloud (in base class) that form polygons of wireframe
      boost::shared_ptr<const std::vector<pcl::Vertices> > wireframe_polygons;

      //! Coefficients of model (plane, cylinder, sphere, etc..)
      pcl::ModelCoefficients::ConstPtr object_coeffs;
    };

#ifdef USE_V4R_V0

    class SurfaceObject : public MemoryObject {
    public:
      typedef boost::shared_ptr<SurfaceObject> Ptr;
      typedef boost::shared_ptr<const SurfaceObject> ConstPtr;

      SurfaceObject(const long long& typeID, const std::string& variable,
              CaptureData::ConstPtr capture, const std::vector<int>& imageIndicesMask)
      : MemoryObject(typeID, variable, capture, imageIndicesMask) {

      };

      virtual ~SurfaceObject() {
      };

      virtual double compare(const MemoryObject::Ptr& other) const;

      void setSurfaceModels(std::vector<surface::SurfaceModel::Ptr>& surfaceModels);

      int getNumSurfaces() const;
      std::vector<surface::SurfaceModel::Ptr> getSurfaceModels();
      const std::vector<cv::Mat>& getSurfaceImages() const;

    protected:

    private:
      std::vector<surface::SurfaceModel::Ptr> surfaceModels;
      std::vector<cv::Mat> surfaceImages;

      cv::Mat getProjectedSurfaceImage(surface::SurfaceModel::Ptr surf);
      cv::Mat getOrthogonallyProjectedSurfaceImage(surface::SurfaceModel::Ptr surf);
      cv::Rect calculateSurfaceBoundingRectangle(surface::SurfaceModel::Ptr surf);
    };
#endif //USE_V4R_V0

  } //namespace stm
} //namespace diarc

#endif  //MEMORYOBJECT_HPP
