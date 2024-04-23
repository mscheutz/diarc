/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "ShapeValidator.hpp"
#include "capture/calibration/Cameras.hpp"
#include "display/Display.hpp"
#include "common/notification/MemoryObjectsNotification.hpp"

#include <boost/foreach.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_base.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

using namespace ade::stm;

ShapeValidator::ShapeValidator(const long long& processorId, const unsigned int imgWidth,
        const unsigned int imgHeight, const bool isStereo)
: ObjectValidator(processorId, imgWidth, imgHeight, isStereo)
/*,classifier(1980)*/ {
  visionProcessName = "ShapeValidator";
  logger = log4cxx::Logger::getLogger("ade.imgproc.validator.ShapeValidator");
}

ShapeValidator::~ShapeValidator() {
}

void ShapeValidator::loadConfig(const std::string& config) {
  cylinderSeg.setOptimizeCoefficients(true);
  cylinderSeg.setModelType(pcl::SACMODEL_CYLINDER);
  cylinderSeg.setMethodType(pcl::SAC_RANSAC);
  cylinderSeg.setNormalDistanceWeight(0.1);
  cylinderSeg.setMaxIterations(1000);
  cylinderSeg.setDistanceThreshold(0.05);
  cylinderSeg.setRadiusLimits(0, 0.1);
}

void ShapeValidator::handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification) {
  LOG4CXX_TRACE(logger, "[handleMemoryObjectNotification] method entered.");
  MemoryObject::Ptr object = notification->object;
  // HACK
  // write out all projected surface images to collect training data
  //  std::string filename;
  //  static int cnt = 0;
  //  SurfaceObject::Ptr surfaceObject = boost::dynamic_pointer_cast<SurfaceObject>(object);
  //  if (surfaceObject) {
  //    cv::Mat objectImage = surfaceObject->getObjectImage();
  //    filename = "surf" + boost::lexical_cast<std::string>(cnt) + ".png";
  //    imwrite(filename, objectImage);
  //    
  //    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr objectCloud = surfaceObject->getObjectPointCloud();
  //    filename = "surf" + boost::lexical_cast<std::string>(cnt) + ".pcd";
  //    pcl::io::savePCDFile(filename, *objectCloud);
  //    
  //    cnt++;
  //  }
  // HACK END

  //if (checkShape(object)) {
  if (checkForShapes(object)) {
    sendValidationNotifications(object);
  }

  if (getDisplayFlag()) {
    MemoryObject::VecPtr validatedObjects(new MemoryObject::Vec());
    validatedObjects->push_back(object);
    display(validatedObjects);
    sleep(1);
  }

  LOG4CXX_TRACE(logger, "[handleMemoryObjectNotification] method returning.");
}

void ShapeValidator::handleMemoryObjectsNotification(MemoryObjectsNotification::ConstPtr notification) {
  MemoryObject::VecPtr objects = notification->objects;
  LOG4CXX_TRACE(logger, boost::format("[handleMemoryObjectsNotification] method entered with %d objects.") % objects->size());

  MemoryObject::VecPtr validatedObjects(new MemoryObject::Vec());
  MemoryObject::Vec::const_iterator object_iter;
  for (object_iter = objects->begin(); object_iter != objects->end(); ++object_iter) {
    if (checkForShapes(*object_iter)) {
      validatedObjects->push_back(*object_iter);
    }
  }

  if (getDisplayFlag()) {
    display(validatedObjects);
  }

  LOG4CXX_DEBUG(logger, boost::format("[handleMemoryObjectsNotification] found %d valid objects.") % validatedObjects->size());
  sendValidationNotifications(validatedObjects);
}

bool ShapeValidator::checkForShapes(MemoryObject::Ptr object) {

  // get current descriptors to process
  TypesByDescriptorConstPtr descriptors = getDescriptors();
  int numDescriptors = descriptors->size();

  if (numDescriptors == 0) {
    LOG4CXX_ERROR(logger, "No descriptors set, returning.");
    return false;
  }

  object->lock();

  std::string label = "";
  float confidence = 0.0;
  TypesByDescriptor::const_iterator descriptors_itr = descriptors->begin();
  for (descriptors_itr = descriptors->begin(); descriptors_itr != descriptors->end(); ++descriptors_itr) {
    label = descriptors_itr->first.getName();
    if (label.compare("box") == 0) {
      confidence = checkForBox(object);
      object->addValidationResult(confidence, descriptors_itr->first);
    } else if (label.compare("cylinder") == 0) {
      confidence = checkForCylinder(object);
      object->addValidationResult(confidence, descriptors_itr->first);
    } else {
      LOG4CXX_WARN(logger, boost::format("[checkForShapes] don't know how to process descriptor: %s.") % descriptors_itr->first.toString());
    }
  }

  object->unlock();

  //check if meets shape requirements
  if (logger->isDebugEnabled()) {
    std::string message = object->getValidationResultsString();
    LOG4CXX_DEBUG(logger, boost::format("[checkForShapes] validation results: %s.") % message);
  }
  return (confidence > 0.0) ? true : false;
}

//assumes that all necessary surfaces are contained in a single MemoryObject

float ShapeValidator::checkForBox(MemoryObject::Ptr object) {
  LOG4CXX_TRACE(logger, "[checkForBox] method entered.");

  //SurfaceObject
  SurfaceObject::Ptr surfaceObject = boost::dynamic_pointer_cast<SurfaceObject>(object);
  if (surfaceObject) {
    //need at least two sides to make a box
    if (surfaceObject->getNumSurfaces() < 2) {
      LOG4CXX_TRACE(logger, "[checkForBox] not enough surfaces!");
      return 0.0;
    } else {
      LOG4CXX_TRACE(logger, "[checkForBox] at least two surfaces!");
    }

    int objectCloudSize = surfaceObject->getIndicesMask().size();
    int surfaceSizeThresh = objectCloudSize / 10;
    float confidence = 1.0;
    int numPairs = 0;
    int numSkippedPairs = 0;
    const static float tolerance = M_PI / 6.0;
    float angle = 0.0;
    std::vector<surface::SurfaceModel::Ptr> surfs = surfaceObject->getSurfaceModels();
    std::vector<surface::SurfaceModel::Ptr>::const_iterator surf_iter1;
    std::vector<surface::SurfaceModel::Ptr>::const_iterator surf_iter2;

    //iterate through all surface pairs
    //check that all pairs are at right angles (can't see opposing sides in 2.5-D)
    for (surf_iter1 = surfs.begin(); surf_iter1 != surfs.end(); ++surf_iter1) {
      for (surf_iter2 = surf_iter1 + 1; surf_iter2 != surfs.end(); ++surf_iter2, ++numPairs) {
        //check that each surface is a significant portion of total object
        //to deal with imperfect segmentation
        if ((*surf_iter1)->indices.size() < surfaceSizeThresh || (*surf_iter2)->indices.size() < surfaceSizeThresh) {
          LOG4CXX_DEBUG(logger, "[checkForBox] SurfaceModel too small, continuing to next surface pair.");
          LOG4CXX_DEBUG(logger, boost::format("[checkForBox] SurfaceModel too small. object: %d. surface 1: %d. surface 2: %d.")
                  % objectCloudSize % (*surf_iter1)->indices.size() % (*surf_iter2)->indices.size());
          ++numSkippedPairs;
          continue;
        }

        //check that both surfaces are planes
        if ((*surf_iter1)->type != 0 || (*surf_iter2)->type != 0) {
          LOG4CXX_DEBUG(logger, "[checkForBox] SurfaceObject contains non-plane surface. Can't be box.");
          continue;
          //return 0.0;
        }

        //get surface normal
        Eigen::Vector3f coeffs1((*surf_iter1)->coeffs[0], (*surf_iter1)->coeffs[1], (*surf_iter1)->coeffs[2]);
        Eigen::Vector3f coeffs2((*surf_iter2)->coeffs[0], (*surf_iter2)->coeffs[1], (*surf_iter2)->coeffs[2]);

        //normalize
        coeffs1.normalize();
        coeffs2.normalize();

        //check angle between normals
        angle = std::acos(coeffs1.dot(coeffs2));

        //TODO: come up with better criteria/confidence measure
        float angle_diff = std::abs(angle - (M_PI / 2.0));
        LOG4CXX_DEBUG(logger, boost::format("[checkForBox] pair number %d has %f angle. Dist from PI/2: %f.") % numPairs % angle % angle_diff);
        if (angle_diff < tolerance) {
          confidence *= 1.0 - (angle_diff / (M_PI / 2.0));
        } else {
          return 0.0;
        }
      }
    }

    //done processing pairs, make sure we didn't skip them all and return a 1.0 conf
    if (numPairs == numSkippedPairs) {
      LOG4CXX_DEBUG(logger, boost::format("[checkForBox] skipped all %d pairs, returning 0.0 confidence.") % numPairs);
      return 0.0;
    }
    LOG4CXX_DEBUG(logger, boost::format("[checkForBox] box found with %f confidence and %d surfaces.") % confidence % surfaceObject->getNumSurfaces());
    return confidence;

  } else {
    LOG4CXX_ERROR(logger, "[checkForBox] could not cast MemoryObject to appropriate type.");
    return 0.0;
  }
}

float ShapeValidator::checkForCylinder(MemoryObject::Ptr object) {
  //SurfaceObject
  SurfaceObject::Ptr surfaceObject = boost::dynamic_pointer_cast<SurfaceObject>(object);
  if (surfaceObject) {

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud = surfaceObject->getObjectPointCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloud(new pcl::PointCloud<pcl::PointXYZ>());

    // Build a passthrough filter to remove NaNs
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 10);
    pass.filter(*objectCloud);

    if (objectCloud->size() == 0) {
      LOG4CXX_WARN(logger, "[checkForCylinder] 0 points in object point cloud.");
      return 0.0;
    }

    //normal estimation
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::Normal>::Ptr objectNormals(new pcl::PointCloud<pcl::Normal> ());
    ne.setSearchMethod(tree);
    ne.setInputCloud(objectCloud);
    ne.setKSearch(25);
    ne.compute(*objectNormals);

    //set input data for segmentation object
    cylinderSeg.setInputCloud(objectCloud);
    cylinderSeg.setInputNormals(objectNormals);

    //segment cylinder
    LOG4CXX_DEBUG(logger, "[checkForCylinder] segmenting cylinder...");
    pcl::PointIndices::Ptr cylinderIndices(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients());
    cylinderSeg.segment(*cylinderIndices, *coeffs);
    LOG4CXX_DEBUG(logger, "[checkForCylinder]...done segmenting cylinder.");

    int numInliers = cylinderIndices->indices.size();
    int totalPoints = objectCloud->size();
    float inlierRatio = static_cast<float> (numInliers) / totalPoints;
    if (inlierRatio < 0.25) {
      LOG4CXX_TRACE(logger, "[checkForCylinder] not enough inliers for cylinder.");
      return 0.0;
    }

    //return (num inliers) / (total points) as confidence value
    LOG4CXX_DEBUG(logger, boost::format("[checkForCylinder] found model with %d inliers out of %d points.")
            % cylinderIndices->indices.size() % objectCloud->size());
    return static_cast<float> (cylinderIndices->indices.size()) / objectCloud->size();

  } else {
    LOG4CXX_ERROR(logger, "[checkForCylinder] could not cast MemoryObject to appropriate type.");
    return 0.0;
  }
}

void ShapeValidator::display(MemoryObject::VecPtr objects) {
  LOG4CXX_TRACE(logger, "[display] method entered.");

  if (objects->size() < 1) {
    return;
  }

  //get image to display
  objects->at(0)->getCaptureData()->frame.copyTo(displayFrame);

  // go through all memory objects
  MemoryObject::Vec::const_iterator object_iter;
  for (object_iter = objects->begin(); object_iter != objects->end(); ++object_iter) {

    //get object info
    cv::Rect pbox = (*object_iter)->getBoundingBox();

    //actually draw
    int color = 255; // * confidence;
    cv::rectangle(displayFrame, cvPoint(pbox.x, pbox.y), cvPoint(pbox.x + pbox.width, pbox.y + pbox.height), cv::Scalar(color, color, color), 3);
  }

  ade::Display::displayFrame(displayFrame, getDisplayName());

  //3D visualization
  // create a point cloud with each object surface a different random color
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr displayCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  displayCloud->resize(img_width * img_height);
  pcl::PointXYZRGB tmpPoint;
  double r, g, b;
  for (MemoryObject::Vec::iterator o = objects->begin(); o != objects->end(); ++o) {
    //cast from base to derived type
    SurfaceObject::Ptr currSurfObj = boost::dynamic_pointer_cast<SurfaceObject > (*o);
    if (currSurfObj == SurfaceObject::Ptr()) {
      LOG4CXX_WARN(logger, "Bad cast from MemoryObjectPtr to SurfaceObject::Ptr.");
      continue;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud = currSurfObj->getCaptureData()->cloudRGB;
    std::vector<surface::SurfaceModel::Ptr> surfaces = currSurfObj->getSurfaceModels();
    std::vector<surface::SurfaceModel::Ptr>::iterator surfaceIter;
    for (surfaceIter = surfaces.begin(); surfaceIter != surfaces.end(); ++surfaceIter) {
      r = std::rand() % 255;
      g = std::rand() % 255;
      b = std::rand() % 255;
      for (size_t i = 0; i < (*surfaceIter)->indices.size(); i++) {
        tmpPoint = cloud->points[(*surfaceIter)->indices[i]];
        tmpPoint.r = r;
        tmpPoint.g = g;
        tmpPoint.b = b;
        displayCloud->points.push_back(tmpPoint);
      }
    }
  }
  ade::Display::displayPointCloud(displayCloud, "clusters", getDisplayName());
}

/////////////////// EXPERIMENTAL //////////////////////////////
//
//void ShapeValidator::loadConfig(const std::string& config) {
//  LOG4CXX_TRACE(logger, "[loadConfig] method entered.");
//
//  //get directory
//  unsigned found = config.find_last_of("/\\");
//  std::string dir = config.substr(0, found + 1);
//
//  // populate tree structure pt
//  using boost::property_tree::ptree;
//  ptree pt;
//  read_xml(config, pt);
//
//  // traverse pt
//  std::string cloudFilename, imageFilename, descriptorName, functorName;
//  int arity;
//
//  //to read in cloud
//  pcl::PCDReader reader;
//
//  // Create an empty kdtree representation, and pass it to the normal estimation object. 
//  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given). 
//  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
//
//  //USC 
//  usc.setSearchMethod(tree);
//  usc.setLocalRadius(0.025); //0.025
//  usc.setMinimalRadius(0.001);
//  usc.setPointDensityRadius(0.02);
//  usc.setRadiusSearch(0.025); //0.025
//  //usc.setAzimuthBins(100);
//  //usc.setElevationBins(100);
//
//  // Output datasets
//  pcl::UniqueShapeContext< pcl::PointXYZRGB>::PointCloudOut::Ptr usc_output(new pcl::UniqueShapeContext< pcl::PointXYZRGB>::PointCloudOut());
//
//  BOOST_FOREACH(ptree::value_type const& predicateNode, pt.get_child("processor")) {
//    if (predicateNode.first.compare("predicate") == 0) {
//      functorName = predicateNode.second.get<std::string> ("<xmlattr>.functorname", "unknown");
//      arity = predicateNode.second.get<int> ("<xmlattr>.arity", -1);
//      LOG4CXX_DEBUG(logger, boost::format("[loadConfig] name: %s. functor name: %s. arity: %d.") % predicateNode.first % functorName % arity);
//
//      std::vector<std::string> imageFiles;
//
//      BOOST_FOREACH(ptree::value_type const& descriptorNode, predicateNode.second) {
//        imageFiles.clear();
//        if (descriptorNode.first.compare("descriptor") == 0) {
//          descriptorName = descriptorNode.second.get<std::string> ("<xmlattr>.name", "unknown");
//          LOG4CXX_DEBUG(logger, boost::format("[loadConfig] descriptorName: %s.") % descriptorName);
//
//          //get all filenames for descriptor
//          std::string imgName = descriptorNode.second.get("image", "");
//          std::string cloudName = descriptorNode.second.get("cloud", "");
//          LOG4CXX_DEBUG(logger, boost::format("[loadConfig] dir: %s. cloud: %s. image: %s") % dir % cloudName % imgName);
//
//          imageFilename = dir + imgName;
//          cloudFilename = dir + cloudName;
//
//          //load image from file
//          cv::Mat image = cv::imread(imageFilename);
//
//          //load cloud from file
//          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//          reader.read(cloudFilename, *cloud);
//
//          //filter cloud
//          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZRGB>);
//          filterPointCloud(image, cloud, cloudFiltered);
//
//          if (cloudFiltered->size() == 0) {
//            LOG4CXX_WARN(logger, "[loadConfig] 0 inlier points in filtered point cloud.");
//            continue;
//          }
//
//          // Compute the features 
//          LOG4CXX_INFO(logger, "[loadConfig] feature extraction starting...");
//          usc.setInputCloud(cloudFiltered);
//          usc.setSearchSurface(cloud);
//          //usc.setIndices(cloudIndices);
//          usc_output->clear();
//          usc.compute(*usc_output);
//          LOG4CXX_INFO(logger, "[loadConfig] done.");
//
//
//          //write features to file
//          //          LOG4CXX_INFO(logger, "[loadConfig] to file:");
//          //          std::string filename = descriptorName + ".txt";
//          //          FILE* f = fopen(filename.c_str(), "wt");
//          //          if (!f) {
//          //            std::cout << "Could not open file: " << filename << std::endl;
//          //            continue;
//          //          }
//          //          for (size_t i = 0; i < usc_output->size(); ++i) {
//          //            for (size_t j = 0; j < 1980; ++j) {
//          //              fprintf(f, "%f, ", usc_output->at(i).descriptor[j]);
//          //
//          //              //if (usc_output->at(i).descriptor[j] != 0.0f)
//          //              //  LOG4CXX_INFO(logger, boost::format("%d, %d: %f. ") % i % j % usc_output->at(i).descriptor[j]);
//          //            }
//          //            fprintf(f, "\n");
//          //          }
//          //          fclose(f);
//
//          LOG4CXX_INFO(logger, boost::format("%s. num features calculated: %d") % descriptorName % usc_output->size());
//          classifier.addFeatures(usc_output->points, descriptorName);
//        }
//      }
//    }
//  }
//
//  //and finally, build the classifier
//  classifier.buildClassifier();
//
//  //  LOG4CXX_DEBUG(logger, "[checkShape] calculating nearest neighbors...");
//  //  PCLFeatureKNNClassifier<pcl::ShapeContext1980>::Results results = classifier.nearestKSearch(usc_output->points, 1);
//  //  results.print();
//  //  LOG4CXX_DEBUG(logger, "[checkShape] done.");
//}

//
//bool ShapeValidator::checkShape(MemoryObject::Ptr object) {
//  LOG4CXX_DEBUG(logger, "[checkShape] method entered.");
//  SurfaceObject::Ptr surfaceObject = boost::dynamic_pointer_cast<SurfaceObject>(object);
//  if (surfaceObject) {
//
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZRGB>);
//    filterPointCloud(surfaceObject->getObjectImage(), surfaceObject->getObjectPointCloud(), cloudFiltered);
//
//    //    // Build a passthrough filter to remove NaNs
//    //    pcl::PassThrough<pcl::PointXYZRGB> pass;
//    //    pass.setFilterFieldName("z");
//    //    pass.setFilterLimits(0, 10);
//    //    pass.setInputCloud(surfaceObject->getObjectPointCloud());
//    //    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//    //    pass.filter(*filteredCloud);
//
//    //compute usc features
//    LOG4CXX_DEBUG(logger, "[checkShape] computing features...");
//    usc.setInputCloud(cloudFiltered);
//    usc.setSearchSurface(surfaceObject->getObjectPointCloud());
//    pcl::UniqueShapeContext< pcl::PointXYZRGB>::PointCloudOut::Ptr usc_output(new pcl::UniqueShapeContext< pcl::PointXYZRGB>::PointCloudOut());
//    usc.compute(*usc_output);
//    LOG4CXX_DEBUG(logger, "[checkShape] done.");
//
//    //usc_output->points.resize(100);
//    LOG4CXX_DEBUG(logger, "[checkShape] calculating nearest neighbors...");
//    PCLFeatureKNNClassifier<pcl::ShapeContext1980>::Results results = classifier.nearestKSearch(usc_output->points, 1);
//    results.print();
//    LOG4CXX_DEBUG(logger, "[checkShape] done.");
//  }
//
//  return false;
//}
//
////TODO: use voxel filter to reduce number of features calculated
////TODO: ignore "edge" points around segmented object
//
//void ShapeValidator::filterPointCloud(const cv::Mat& image,
//        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr inputCloud,
//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ouputCloud) {
//
//  //don't consider edge points
//  LOG4CXX_DEBUG(logger, "[filterPointCloud] removing border edges.");
//  pcl::PointIndicesPtr cloudIndices = pcl::PointIndicesPtr(new pcl::PointIndices());
//  removeBorderEdges(image, cloudIndices);
//
//  //filter out nans
//  LOG4CXX_DEBUG(logger, "[filterPointCloud] filtering out nans.");
//  pcl::PassThrough<pcl::PointXYZRGB> pass;
//  pass.setFilterFieldName("z");
//  pass.setFilterLimits(0, 10);
//  pass.setInputCloud(inputCloud);
//  pass.setIndices(cloudIndices);
//  pass.filter(cloudIndices->indices);
//  //pass.filter(*cloud);
//
//  //voxel grid filter to reduce num of features calculated
//  LOG4CXX_DEBUG(logger, "[filterPointCloud] downsampling.");
//  pcl::VoxelGrid<pcl::PointXYZRGB> vox;
//  vox.setInputCloud(inputCloud);
//  vox.setIndices(cloudIndices);
//  vox.setLeafSize(0.01f, 0.01f, 0.01f);
//  vox.filter(*ouputCloud);
//
//  //debugging
//  if (logger->isDebugEnabled()) {
//    LOG4CXX_DEBUG(logger, boost::format("[filterPointCloud] size of orig cloud: %d. filtered cloud: %d.") % inputCloud->size() % ouputCloud->size());
//    ade::Display::createWindowIfDoesNotExist("input");
//    ade::Display::displayPointCloud(inputCloud, "input", "input");
//    ade::Display::createWindowIfDoesNotExist("ouput");
//    ade::Display::displayPointCloud(ouputCloud, "ouput", "ouput");
//    ade::Display::createWindowIfDoesNotExist("image");
//    ade::Display::displayFrame(image, "image");
//  }
//}
//
//void ShapeValidator::removeBorderEdges(const cv::Mat &image, pcl::PointIndicesPtr& cloudIndices) {
//  cv::Vec3b backgroundColor(0, 0, 192), black(0, 0, 0);
//
//  //build mask
//  cv::Mat mask(image.rows, image.cols, CV_8UC1);
//  mask.setTo(255);
//  for (int i = 0; i < image.rows; i++)
//    for (int j = 0; j < image.cols; j++)
//      if (image.at<cv::Vec3b>(i, j) == backgroundColor || image.at<cv::Vec3b>(i, j) == black)
//        mask.at<unsigned char>(i, j) = 0;
//  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4, 4), cv::Point(-1, -1));
//  cv::erode(mask, mask, element, cv::Point(-1, -1), 3);
//
//  //add indices based on mask
//  for (int i = 0; i < mask.rows; i++)
//    for (int j = 0; j < mask.cols; j++)
//      if (mask.at<unsigned char>(i, j) == 255)
//        cloudIndices->indices.push_back(i * mask.cols + j);
//}