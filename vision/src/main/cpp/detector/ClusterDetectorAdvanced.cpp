/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * Advanced detector for 3D point clusters.
 * Can handle cluttered scenes and does not depend on supporting plane.
 *
 * @author Michael Zillich
 * @date Feb 2013
 */

#include <time.h>
#include <sstream>
#include <fstream>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <log4cxx/fileappender.h>
#include <log4cxx/simplelayout.h>
#include <opencv/cv.hpp>

#include "capture/util/CaptureUtilities.hpp"
#include "capture/Capture.hpp"
#include "ClusterDetectorAdvanced.hpp"
#include "display/Display.hpp"
#include "imgproc/saliency/SaliencyProcessor.hpp"

using namespace diarc::stm;
using namespace std;

static double timespec_diff(struct timespec *x, struct timespec *y) /// TODO Remove later
{
  if (x->tv_nsec < y->tv_nsec) {
    int nsec = (y->tv_nsec - x->tv_nsec) / 1000000000 + 1;
    y->tv_nsec -= 1000000000 * nsec;
    y->tv_sec += nsec;
  }
  if (x->tv_nsec - y->tv_nsec > 1000000000) {
    int nsec = (y->tv_nsec - x->tv_nsec) / 1000000000;
    y->tv_nsec += 1000000000 * nsec;
    y->tv_sec -= nsec;
  }
  return (double) (x->tv_sec - y->tv_sec) +
          (double) (x->tv_nsec - y->tv_nsec) / 1000000000.;
}

ClusterDetectorAdvanced::ClusterDetectorAdvanced(const long long& processorId, const int imgWidth, const int imgHeight)
: ObjectDetector(processorId, imgWidth, imgHeight),
segmenter(NULL),
saliencyReceived(false) {
  salmap = cv::Mat_<float>::ones(imgHeight, imgWidth);
  visionProcessName = "ClusterDetectorAdvanced";
  logger = log4cxx::Logger::getLogger("diarc.detector.ClusterDetectorAdvanced");
}

ClusterDetectorAdvanced::~ClusterDetectorAdvanced() {
}

void ClusterDetectorAdvanced::init() {
  if (segmenter == NULL) {
    segmenter = new segmentation::Segmenter();
    segmenter->setModelFilename(objectModelFilename);
    segmenter->setScaling(scalingParamsFilename);
  }
  saliencyReceived = false;
}

void ClusterDetectorAdvanced::cleanup() {
  delete segmenter;
  segmenter = NULL;
}

void ClusterDetectorAdvanced::loadConfig(const std::string& config) {
  LOG4CXX_TRACE(logger, "[loadConfig] method entered.");

  //get directory
  unsigned found = config.find_last_of("/\\");
  std::string dir = config.substr(0, found + 1);

  // populate tree structure pt
  using boost::property_tree::ptree;
  ptree pt;
  read_xml(config, pt);

  // parse xml file

  BOOST_FOREACH(ptree::value_type const& predicateNode, pt.get_child("processor")) {
    if (predicateNode.first.compare("predicate") == 0) {
      std::string descriptorName = predicateNode.second.get<std::string> ("<xmlattr>.name", "unknown");
      LOG4CXX_DEBUG(logger, boost::format("[loadConfig] descriptorName: %s.") % descriptorName);

      objectModelFilename = dir + predicateNode.second.get_child("model").get<std::string> ("<xmlattr>.filename", "unknown");
      scalingParamsFilename = dir + predicateNode.second.get_child("params").get<std::string> ("<xmlattr>.filename", "unknown");
      LOG4CXX_DEBUG(logger, boost::format("[loadConfig] modelFilename: %s. paramFilename: %s.") % objectModelFilename % scalingParamsFilename);
    }
  }
}

void ClusterDetectorAdvanced::handleSaliencyNotification(SaliencyNotification::ConstPtr notification) {
  notification->saliencyMap.copyTo(salmap);
  std::vector<cv::Mat> salmaps;
  salmaps.push_back(salmap);
  segmenter->setSaliencyMaps(salmaps);
  saliencyReceived = true;
}

void ClusterDetectorAdvanced::handleCaptureNotification(CaptureNotification::ConstPtr notification) {

  // perform the detection, this can take a while (from hundreds of ms to a few s)
  MemoryObject::VecPtr newDetectedObjects = detectClustersNonincremental(notification);

  // send notifications
  sendDetectionNotifications(newDetectedObjects);

  //display results
  if (getDisplayFlag()) {
    display(newDetectedObjects);
    //localIterativeDisplay(newDetectedObjects);
  }
}

MemoryObject::VecPtr ClusterDetectorAdvanced::detectClustersNonincremental(CaptureNotification::ConstPtr capture) {

  //EAK: copy here because segmenter can't take ConstPtr
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = capture->captureData->cloudRGB->makeShared();
  // check the input cloud
  if (cloud.get() == 0) {
    LOG4CXX_ERROR(logger, "[detectClustersNonincremental] no point cloud available in captureData.");
    MemoryObject::VecPtr newObjects(new MemoryObject::Vec());
    return newObjects;
  } else if (!cloud->isOrganized()) {
    LOG4CXX_ERROR(logger, "[detectClustersNonincremental] we need an organized point cloud, and the point cloud we got is not.");
    MemoryObject::VecPtr newObjects(new MemoryObject::Vec());
    return newObjects;
  }

  LOG4CXX_DEBUG(logger, "[detectClustersNonincremental] starting ...");
  struct timespec start, end;
  struct timespec start2, end2;
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
  clock_gettime(CLOCK_REALTIME, &start2);
  segmenter->setPointCloud(cloud);
  if (saliencyReceived) {
    segmenter->attentionSegment();
  } else {
    segmenter->segment();
  }
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end);
  clock_gettime(CLOCK_REALTIME, &end2);
  LOG4CXX_DEBUG(logger, boost::format("[detectClustersNonincremental] runtime: %4.3f (real time %4.3f)") %
          timespec_diff(&end, &start) % timespec_diff(&end2, &start2));


  return createMemoryObjects(segmenter->getSurfaces(), capture);
}

MemoryObject::VecPtr ClusterDetectorAdvanced::createMemoryObjects(const vector<surface::SurfaceModel::Ptr>& surfs, CaptureNotification::ConstPtr capture) {
  LOG4CXX_TRACE(logger, "[createMemoryObjects] method entered.");

  //group surfaces with same label into objects
  map<int, vector<surface::SurfaceModel::Ptr> > objects;
  for (size_t i = 0; i < surfs.size(); ++i) {
    int label = surfs[i]->label;
    std::vector<surface::SurfaceModel::Ptr>& temp = objects[label]; //will create vec if doesn't already exist
    temp.push_back(surfs[i]);
  }
  LOG4CXX_DEBUG(logger, boost::format("[createMemoryObjects] have %d objects.") % (int) objects.size());

  //make new MemoryObjects from grouped surfaces
  MemoryObject::VecPtr newObjects(new MemoryObject::Vec());
  map<int, vector<surface::SurfaceModel::Ptr> >::iterator object_iter;
  for (object_iter = objects.begin(); object_iter != objects.end(); ++object_iter) {
    vector<surface::SurfaceModel::Ptr>& surfaceModels = object_iter->second;

    // get point indices
    pcl::PointIndices::Ptr pointIndices(new pcl::PointIndices());
    for (int i = 0; i < surfaceModels.size(); ++i) {
      for (size_t j = 0; j < surfaceModels[i]->indices.size(); ++j) {
        pointIndices->indices.push_back(surfaceModels[i]->indices[j]);
      }
    }

    // set detection confidence, descriptors, and typeids
    TypesByDescriptorConstPtr descriptors = getDescriptors();
    TypesByDescriptor::const_iterator descriptors_itr = descriptors->begin();
    for (descriptors_itr = descriptors->begin(); descriptors_itr != descriptors->end(); ++descriptors_itr) {
      LOG4CXX_DEBUG(logger, boost::format("Descriptor: %s.") % descriptors_itr->first.toString());
      std::tr1::unordered_set<long long>::const_iterator typeId_itr;
      for (typeId_itr = descriptors_itr->second.begin(); typeId_itr != descriptors_itr->second.end(); ++typeId_itr) {
        LOG4CXX_DEBUG(logger, boost::format("TypeId: %ld.") % *(typeId_itr));

        SurfaceObject::Ptr newObject(new SurfaceObject(*typeId_itr, descriptors_itr->first.getArg(0), capture->captureData, pointIndices->indices));
        newObject->setSurfaceModels(surfaceModels);

        //EAK: this works, but is probably unnecessary right now
        //compute and set wireframe (mesh) for cluster
        //            pcl::PolygonMesh::Ptr polygonMesh(new pcl::PolygonMesh());
        //            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMesh(new pcl::PointCloud<pcl::PointXYZ>());
        //            ComputePolygonMesh<pcl::PointXYZ > (currCluster, polygonMesh);
        //            //SplitPlanesAndMesh<pcl::PointXYZ > (currCluster, polygonMesh);
        //            pcl::fromROSMsg(polygonMesh->cloud, *cloudMesh);
        //            newObject->setWireframe(cloudMesh, polygonMesh->polygons);

        newObjects->push_back(newObject);
      }
    }
  }

  return newObjects;
}

void ClusterDetectorAdvanced::display(MemoryObject::VecPtr newObjects) {

  if (newObjects->size() < 1) {
    return;
  }

  //get image to display
  newObjects->at(0)->getCaptureData()->frame.copyTo(displayFrame);

  //display  
  MemoryObject::Vec::iterator newObjectIter;
  for (newObjectIter = newObjects->begin(); newObjectIter != newObjects->end(); ++newObjectIter) {
    const CvRect& rect = (*newObjectIter)->getBoundingBox();

    cv::rectangle(displayFrame, cv::Point(rect.x, rect.y),
            cv::Point(rect.x + rect.width, rect.y + rect.height),
            cv::Scalar(0, 0, 255), 2, 8, 0);
  }

  //draw on cluster boxes
  diarc::Display::displayFrame(displayFrame, getDisplayName());

  //3D visualization
  // create a point cloud with each object different random color
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr displayCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  displayCloud->resize(img_width * img_height);
  pcl::PointXYZRGB tmpPoint;
  double r, g, b;
  for (MemoryObject::Vec::iterator o = newObjects->begin(); o != newObjects->end(); ++o) {
    //cast from base to derived type
    SurfaceObject::Ptr currSurfObj = boost::dynamic_pointer_cast<SurfaceObject > (*o);
    if (currSurfObj == SurfaceObject::Ptr()) {
      LOG4CXX_WARN(logger, "Bad cast from MemoryObjectPtr to SurfaceObject::Ptr.");
      continue;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud = currSurfObj->getCaptureData()->cloudRGB;
    std::vector<int> objectIndices = currSurfObj->getIndicesMask();
    r = std::rand() % 255;
    g = std::rand() % 255;
    b = std::rand() % 255;
    for (size_t i = 0; i < objectIndices.size(); i++) {
      tmpPoint = cloud->points[objectIndices[i]];
      tmpPoint.r = r;
      tmpPoint.g = g;
      tmpPoint.b = b;
      displayCloud->points.push_back(tmpPoint);
    }
  }
  diarc::Display::displayPointCloud(displayCloud, "clusters", getDisplayName());
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* cloud_void) {
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud = *static_cast<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > *> (cloud_void);
  if (event.getKeySym() == "s" && event.keyDown()) {
    //save pcd to file
    static int counter = 0;
    std::string pcd_filename = "/dev/shm/shapeimg_" + boost::lexical_cast<std::string>(counter++) + ".pcd";
    //std::string pcd_filename = "/dev/shm/shapeimg.pcd";// + boost::lexical_cast<std::string>(counter++) + ".pcd";
    pcl::io::savePCDFile(pcd_filename, *cloud);
  }
}

void ClusterDetectorAdvanced::localIterativeDisplay(MemoryObject::VecPtr newObjects) {

  //3D visualization
  if (!viewer) {
    viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer);
    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*) &displayCloud);
    viewer->initCameraParameters();
    viewer->addCoordinateSystem(0.1, 0);
    viewer->setCameraPosition(0, 0, -1, 0, 0, 0, 0, -1, 0);
  }

  // create a point cloud with each object different random color
  if (!displayCloud) {
    displayCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
  }

  viewer->addPointCloud<pcl::PointXYZRGB>(displayCloud, "display cloud");

  pcl::PointXYZRGB tmpPoint;
  double r, g, b;
  for (MemoryObject::Vec::iterator o = newObjects->begin(); o != newObjects->end(); ++o) {
    //cast from base to derived type
    SurfaceObject::Ptr currSurfObj = boost::dynamic_pointer_cast<SurfaceObject > (*o);
    if (currSurfObj == SurfaceObject::Ptr()) {
      LOG4CXX_WARN(logger, "Bad cast from MemoryObjectPtr to SurfaceObject::Ptr.");
      continue;
    }
    displayCloud = currSurfObj->getObjectPointCloudRGB();
    viewer->updatePointCloud<pcl::PointXYZRGB>(displayCloud, "display cloud");

    while (!viewer->wasStopped()) {
      viewer->spinOnce();
    }
    viewer->resetStoppedFlag();
    if (!getDisplayFlag()) {
      viewer->removePointCloud("display cloud");
      viewer->close();
      //viewer.reset();
      break;
    }
  }
}
