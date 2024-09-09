/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include <cfloat>
//#include <eigen3/Eigen/src/StlSupport/StdVector.h>
#include <boost/lexical_cast.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "capture/util/CaptureUtilities.hpp"
#include "ClusterDetector.hpp"
#include "display/Display.hpp"
#include "imgproc/saliency/SaliencyProcessor.hpp"
#include "point_clouds/sorted_extract_clusters.hpp"

using namespace diarc::stm;

bool IndexedValueTypeSortFunction(IndexedValueType i, IndexedValueType j) {
  return (i.value > j.value);
}

ClusterDetector::ClusterDetector(const long long& processorId, const int imgWidth, const int imgHeight)
: ObjectDetector(processorId, imgWidth, imgHeight),
object_cluster_tolerance(0.04),
object_cluster_min_size(20 * (imgWidth / 320) * (imgHeight / 240)) {
  salmap = cv::Mat_<float>::ones(imgHeight, imgWidth);
  visionProcessName = "ClusterDetector";
  plane = ExtractedPlane::Ptr(new ExtractedPlane());
  logger = log4cxx::Logger::getLogger("diarc.detector.ClusterDetector");
}

ClusterDetector::~ClusterDetector() {
}

void ClusterDetector::registerForCaptureNotification() {
  // intentionally empty
}

void ClusterDetector::unregisterForCaptureNotification() {
  // intentionally empty
}

void ClusterDetector::handleSaliencyNotification(SaliencyNotification::ConstPtr notification) {
  notification->saliencyMap.copyTo(salmap);
}

void ClusterDetector::handlePlaneNotification(PlaneNotification::ConstPtr notification) {
  plane = notification->plane;
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud = plane->getCloudRGB();

  //EAK: why not make these things class members, instead of instantiating and
  //setting parameters *every* iteration ??
  //@ep the was a memory bug with members, I couldn't find where it comes from and made things like this

  // Clustering parameters
  SortedEuclideanClusterExtraction<pcl::PointXYZRGB> cluster;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr clusters_tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
  clusters_tree->setEpsilon(1);
  cluster.setClusterTolerance(object_cluster_tolerance);
  cluster.setMinClusterSize(object_cluster_min_size);
  cluster.setSearchMethod(clusters_tree);

  std::vector<pcl::PointIndices> clusters;

  cluster.setInputCloud(cloud);
  pcl::PointIndices::ConstPtr indices = plane->getFilteredObjectsIndices();
  cluster.setIndices(indices);

  //find mappings between 3D points and image plane
  // TODO potentially should be transferred to the capture device
  cv::Point3f point;
  objectPoints.clear();
  imagePoints.clear();
  objectPoints.resize(cloud->points.size());
  for (int i = 0; i < cloud->points.size(); ++i) {
    point.x = cloud->points.at(i).x;
    point.y = cloud->points.at(i).y;
    point.z = cloud->points.at(i).z;

    if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
      point.x = 0;
      point.y = 0;
      point.z = 0;
    }

    objectPoints.at(i) = point;
  }
  diarc::capture::util::projectPoints(objectPoints, imagePoints, 0);
  assert(imagePoints.size() == cloud->points.size());

  sortPointcloud();

  cluster.setSortedIndices(sorted_indices);

  // perform clustering
  try {
    clusters.clear();
    cluster.extract(clusters, getIncrementalProcessing());
  } catch (...) {
    LOG4CXX_ERROR(logger, "Unable to extract clusters!");
  }

  //send notifications
  MemoryObject::VecPtr newClusterObjects = createMemoryObjects(clusters, notification->captureData);
  sendDetectionNotifications(newClusterObjects);

  if (getDisplayFlag()) {
    display(newClusterObjects);
  }
}

void ClusterDetector::sortPointcloud() {
  LOG4CXX_TRACE(logger, "[sortPointcloud] method entered.");
  pcl::PointIndices::ConstPtr indices = plane->getFilteredObjectsIndices();
  long number_of_indices = indices->indices.size();
  if (number_of_indices > 0) {
    sorted_indices.clear();
    sorted_indices.resize(number_of_indices);
    for (int i = 0; i < number_of_indices; ++i) {
      int index = indices->indices.at(i);
      int xx = imagePoints.at(index).x;
      int yy = imagePoints.at(index).y;
      // check for valid image position
      if (xx > 0 && xx < img_width && yy > 0 && yy < img_height) {
        sorted_indices.at(i) = IndexedValueType(salmap.at<float>(yy, xx), index);
      } else {
        sorted_indices.at(i) = IndexedValueType(0, index);
      }
    }

    if (getIncrementalProcessing()) {
      std::vector<IndexedValueType>::iterator it = sorted_indices.begin();
      std::vector<IndexedValueType>::iterator it_end = sorted_indices.end();
      std::sort(it, it_end, IndexedValueTypeSortFunction);
    }
  }
}

MemoryObject::VecPtr ClusterDetector::createMemoryObjects(const std::vector<pcl::PointIndices>& clusters,
        CaptureData::ConstPtr captureData) {

  LOG4CXX_DEBUG(logger, "[createMemoryObjects] method entered.");
  MemoryObject::VecPtr newClusterObjects = MemoryObject::VecPtr(new MemoryObject::Vec());

  //pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud = plane->getCloudRGB();

  std::vector<pcl::PointIndices>::const_iterator cluster_iter;
  for (cluster_iter = clusters.begin(); cluster_iter != clusters.end(); ++cluster_iter) {

    // set detection confidence, descriptors, and typeids
    float confidence = 1.0f; //TODO
    TypesByDescriptorConstPtr descriptors = getDescriptors();
    TypesByDescriptor::const_iterator descriptors_itr;
    for (descriptors_itr = descriptors->begin(); descriptors_itr != descriptors->end(); ++descriptors_itr) {
      LOG4CXX_DEBUG(logger, boost::format("Descriptor: %s.") % descriptors_itr->first.toString());

      std::tr1::unordered_set<long long>::const_iterator typeId_itr;
      for (typeId_itr = descriptors_itr->second.begin(); typeId_itr != descriptors_itr->second.end(); ++typeId_itr) {
        LOG4CXX_DEBUG(logger, boost::format("TypeId: %ld.") % *(typeId_itr));

        MemoryObject::Ptr newCluster(new MemoryObject(*typeId_itr, descriptors_itr->first.getArg(0), captureData, cluster_iter->indices));
        newCluster->addValidationResult(confidence, descriptors_itr->first, cluster_iter->indices);

        //EAK: this works, but is probably unnecessary right now
        //compute and set wireframe (mesh) for cluster
        //            pcl::PolygonMesh::Ptr polygonMesh(new pcl::PolygonMesh());
        //            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMesh(new pcl::PointCloud<pcl::PointXYZ>());
        //            ComputePolygonMesh<pcl::PointXYZ > (currCluster, polygonMesh);
        //            //SplitPlanesAndMesh<pcl::PointXYZ > (currCluster, polygonMesh);
        //            pcl::fromROSMsg(polygonMesh->cloud, *cloudMesh);
        //            newCluster->setWireframe(cloudMesh, polygonMesh->polygons);

        //add to list of newly detected clusters
        newClusterObjects->push_back(newCluster);
      }
    }
  }

  return newClusterObjects;
}

void ClusterDetector::display(MemoryObject::VecPtr newClusterObjects) {

  // copy the full cloud into a green point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud = plane->getCloudRGB();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr displayCloud(new pcl::PointCloud<pcl::PointXYZRGB>(cloud->width, cloud->height));
  pcl::copyPointCloud(*cloud, *displayCloud);
  for (size_t i = 0; i < displayCloud->size(); ++i) {
    displayCloud->at(i).r = 0;
    displayCloud->at(i).g = 255;
    displayCloud->at(i).b = 0;
  }

  //display
  MemoryObject::Vec::iterator newClusterIter;
  bool firstDisplayIteration = true;
  for (newClusterIter = newClusterObjects->begin(); newClusterIter != newClusterObjects->end(); ++newClusterIter) {
    if (firstDisplayIteration) {
      (*newClusterIter)->getCaptureData()->frame.copyTo(displayFrame);
      firstDisplayIteration = false;
    }
    const cv::Rect& rect = (*newClusterIter)->getDetectionMask()->getBoundingBox();

    cv::rectangle(displayFrame, cv::Point(rect.x, rect.y),
            cv::Point(rect.x + rect.width, rect.y + rect.height),
            cv::Scalar(255, 255, 255), 2, 8, 0);
  }

  //draw on cluster boxes
  diarc::Display::displayFrame(displayFrame, getDisplayName());

  // set objects point cloud to yellow
  for (unsigned long i = 0; i < plane->getFilteredObjectsIndices()->indices.size(); ++i) {
    int index = plane->getFilteredObjectsIndices()->indices.at(i);
    displayCloud->at(index).r = 255;
    displayCloud->at(index).g = 255;
    displayCloud->at(index).b = 0;
  }

  // set plane points to blue
  for (unsigned long i = 0; i < plane->getPlaneIndices()->indices.size(); ++i) {
    int index = plane->getPlaneIndices()->indices.at(i);
    displayCloud->at(index).r = 255;
    displayCloud->at(index).g = 0;
    displayCloud->at(index).b = 0;
  }

  //printf("[ClusterDetector] num clusters: %d\n", newClusters->size());
  for (newClusterIter = newClusterObjects->begin(); newClusterIter != newClusterObjects->end(); ++newClusterIter) {

    //cast from base to derived type to see if there's a wireframe to display
    PointCloudObject::Ptr currPCObj = boost::dynamic_pointer_cast<PointCloudObject > (*newClusterIter);

    //display polygon mesh if set, otherwise display point cloud
    if (currPCObj && boost::shared_ptr<std::vector< pcl::Vertices > >() != currPCObj->getWireframePolygons()) {
      //displayCloud->operator+=(*(currPCObj->getDetectionMask()->getObjectPointCloudRGB()));

      //get mesh to display
      //pcl::PolygonMesh::Ptr polygonMesh(new pcl::PolygonMesh());
      //pcl::toROSMsg(*clusterCloud, polygonMesh->cloud);
      //polygonMesh->polygons = *(currPCObj->getWireframePolygons());
      //diarc::Display::displayPolygonMesh(polygonMesh, "cluster_mesh", getDisplayName());
    } else {
      //displayCloud->operator+=(*((*newClusterIter)->getDetectionMask()->getObjectPointCloudRGB()));
    }
  }

  diarc::Display::displayPointCloud(displayCloud, "cluster", getDisplayName());
}
