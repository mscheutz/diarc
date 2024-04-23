/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "GlobalFeatureValidator.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types_conversion.h>
#include <pcl/point_types.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/vfh.h>
#include <pcl/features/crh.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/our_cvfh.h>
#include <pcl/features/board.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/recognition/crh_alignment.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/surface/mls.h>

#include "capture/calibration/Cameras.hpp"
#include "display/Display.hpp"
#include "point_clouds/PCLFunctions.hpp"
#include "common/notification/LearningNotification.hpp"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/locks.hpp>
#include <boost/filesystem.hpp>

#include <Eigen/Geometry>

using namespace ade::stm;

GlobalFeatureValidator::GlobalFeatureValidator(const long long &processorId, const unsigned int imgWidth,
                                               const unsigned int imgHeight)
        : ObjectValidator(processorId, imgWidth, imgHeight, false),
          show_matches_(false),
          feature_type_(0),
          models_(),
          model_descriptors_(new pcl::PointCloud<DescriptorType>()),
          descriptor_to_model_(),
          type_to_model_(),
          match_search_(),
          ghv_(),
          configFile(),
          has_learned_mutex_(),
          has_learned_flag_(false) {
  visionProcessName = "GlobalFeatureValidator";
  logger = log4cxx::Logger::getLogger("ade.imgproc.validator.GlobalFeatureValidator");

  // Set parameters for global hypothesis verification
  ghv_.setInlierThreshold(0.005f); //(0.01f); //original (0.005f);
  //ghv_.setOcclusionThreshold(0.01f);
  //ghv_.setRegularizer(3.0f);
  //ghv_.setRadiusClutter(0.03f);
  //ghv_.setClutterRegularizer(5.0f);
  ghv_.setDetectClutter(false);
  //ghv_.setRadiusNormals(0.05);


  if (!logger->isDebugEnabled()) {
    // disable pcl console output
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    // disable all ScopeTime output (actually cerr which is through cerr)
    std::cerr.rdbuf(0);
  }
}

GlobalFeatureValidator::~GlobalFeatureValidator() {
  //saveConfig(configFile + ".auto_save");
}

void GlobalFeatureValidator::loadConfig(const std::string &config) {
  LOG4CXX_INFO(logger, boost::format("[loadConfig] method entered: %s") % config);
  configFile = config;

  // get directory
  unsigned found = config.find_last_of("/\\");
  std::string dir = config.substr(0, found + 1);

  // populate tree structure pt
  using boost::property_tree::ptree;
  ptree pt;
  read_xml(config, pt);

  std::string descriptorName;

  // parse tree

  BOOST_FOREACH(ptree::value_type const& predicateNode, pt.get_child("processor")) {
          if (predicateNode.first.compare("predicate") == 0) {
            descriptorName = predicateNode.second.get<std::string>("<xmlattr>.name", "unknown");
            LOG4CXX_DEBUG(logger, boost::format("[loadConfig] descriptorName: %s.") % descriptorName);

            BOOST_FOREACH(ptree::value_type const& objectNode, predicateNode.second) {
                    if (objectNode.first.compare("object") == 0) {

                      // get all info for model
                      ModelType model;
                      model.type_ = descriptorName;
                      ade::stm::Grasp grasp;
                      Eigen::Vector3f point;
                      Eigen::Quaternionf orient;
                      std::string cloud_filename;
                      // std::string image_filename;
                      bool use_color = false;

                      BOOST_FOREACH(ptree::value_type const& infoNode, objectNode.second) {
                              if (infoNode.first.compare("cloud") == 0) {
                                cloud_filename = static_cast<std::string> (infoNode.second.data());
                                LOG4CXX_DEBUG(logger,
                                              boost::format("[loadConfig] dir: %s. cloud: %s") % dir % cloud_filename);
                                cloud_filename = dir + cloud_filename;
                                // LOG4CXX_DEBUG(logger,
                                //               boost::format("[loadConfig] creating model with image filename: %s") % image_filename);
                                LOG4CXX_DEBUG(logger,
                                              boost::format("[loadConfig] creating model with use_color set to: %d") % use_color);
                                if (!createModel(cloud_filename, use_color, model)) {
                                // if (!createModel(cloud_filename, image_filename, use_color, model)) {
                                  //reset model and move to next model
                                  model = ModelType();
                                  continue;
                                }
                                // Requires global_features.xml <object> tag to have <image> followed by <cloud> children
                                //   so image_filename is set first then cloud is loaded and model is created
                                // Otherwise image won't be used and color filtering won't occur
                              //} //else if (infoNode.first.compare("image") == 0) {
                              //   image_filename = static_cast<std::string> (infoNode.second.data());
                              //   LOG4CXX_DEBUG(logger,
                              //                 boost::format("[loadConfig] dir: %s. image: %s") % dir % image_filename);
                              //   image_filename = dir + image_filename;
                              }
                              //<use_color> tag must be above <cloud> to be used properly
                              else if (infoNode.first.compare("use_color") == 0) {
                                use_color = static_cast<std::string> (infoNode.second.data()).compare("true");
                              }
                              else if (infoNode.first.compare("grasp") == 0) {

                                BOOST_FOREACH(ptree::value_type const& graspNode, infoNode.second) {
                                        if (graspNode.first.compare("type") == 0) {
                                          std::string grasp_type = static_cast<std::string> (graspNode.second.data());
                                          if (grasp_type.compare("PINCH_APART") == 0) {
                                            grasp.type_ = Grasp::PINCH_APART;
                                          } else if (grasp_type.compare("PINCH_TOGETHER") == 0) {
                                            grasp.type_ = Grasp::PINCH_TOGETHER;
                                          } else if (grasp_type.compare("PUSH") == 0) {
                                            grasp.type_ = Grasp::PUSH;
                                          } else if (grasp_type.compare("TWO_ARM") == 0) {
                                            grasp.type_ = Grasp::TWO_ARM;
                                          } else {
                                            LOG4CXX_ERROR(logger,
                                                          boost::format("Invalid grasp type: %s.") % grasp_type);
                                          }
                                        } else if (graspNode.first.compare("point") == 0) {
                                          point.x() = graspNode.second.get<double>("x", 0);
                                          point.y() = graspNode.second.get<double>("y", 0);
                                          point.z() = graspNode.second.get<double>("z", 0);
                                          grasp.points_.push_back(point);
                                          LOG4CXX_DEBUG(logger, boost::format("[loadConfig] point: (%f, %f, %f)")
                                                                % point.x() % point.y() % point.z());
                                        } else if (graspNode.first.compare("orientation") == 0) {
                                          orient.x() = graspNode.second.get<double>("x", 0);
                                          orient.y() = graspNode.second.get<double>("y", 0);
                                          orient.z() = graspNode.second.get<double>("z", 0);
                                          orient.w() = graspNode.second.get<double>("w", 1);
                                          grasp.orientations_.push_back(orient);
                                          LOG4CXX_DEBUG(logger,
                                                        boost::format("[loadConfig] orientation: (%f, %f, %f, %f)")
                                                        % orient.x() % orient.y() % orient.z() % orient.w());
                                        }
                                      }
                                // add new grasp option to model and reset grasp object for next iteration
                                model.grasp_options_.push_back(grasp);
                                grasp = Grasp();
                              }
                            }

                      // and finally add new model to list of models and reset model object for next iteration
                      if (!model.cloud_->empty()) {
                        models_.push_back(model);
                      }
                      model = ModelType();
                    }
                  }
          }
        }

  // lock around models data structures
  boost::lock_guard<boost::mutex> models_lock(models_mutex_);

  // build mapping from descriptor index to model index (bc single model can have multiple descriptors)
  std::vector<ModelType>::const_iterator model_itr;
  pcl::PointCloud<DescriptorType>::const_iterator descriptor_iter;
  int model_idx = 0;
  int descriptor_idx = 0;
  for (model_itr = models_.begin(); model_itr != models_.end(); ++model_itr) {
    for (descriptor_iter = model_itr->descriptors_->begin();
         descriptor_iter != model_itr->descriptors_->end(); ++descriptor_iter) {
      model_descriptors_->push_back(*descriptor_iter);
      descriptor_to_model_[descriptor_idx] = model_idx;
      ++descriptor_idx;
    }
    type_to_model_[model_itr->type_].push_back(model_idx);
    ++model_idx;
  }

  // build nn search tree
  match_search_.setInputCloud(model_descriptors_);
}


void GlobalFeatureValidator::saveConfig(const std::string &config) {
  LOG4CXX_TRACE(logger, "[saveConfig] method entered.");

  // get directory
  unsigned found = config.find_last_of("/\\");
  std::string dir = config.substr(0, found + 1);

  // populate tree structure
  // create top level processor xml node
  boost::property_tree::ptree processorNode;
  processorNode.put("processor.<xmlattr>.type", "type");
  processorNode.put("processor.<xmlattr>.arity", "1");

  // iterate through all the model tpyes (e.g., mug, cup, knife, etc.)
  std::map<std::string, std::vector<int> >::const_iterator model_type_itr;
  std::vector<int>::const_iterator model_idx_itr;
  for (model_type_itr = type_to_model_.begin(); model_type_itr != type_to_model_.end(); ++model_type_itr) {
    std::string model_type = model_type_itr->first;
    int viewNum = 0;

    // create new predicate xml node
    boost::property_tree::ptree predicateNode;
    predicateNode.put("<xmlattr>.name", model_type);

    // create new pcd directory for clouds of model type
    std::string pcd_relative_type_dir = "pcd/" + model_type + "/";
    boost::filesystem::path pcd_type_dir(dir + pcd_relative_type_dir);
    boost::filesystem::create_directory(pcd_type_dir);

    for (model_idx_itr = model_type_itr->second.begin();
         model_idx_itr != model_type_itr->second.end(); ++model_idx_itr) {
      const ModelType &model = models_[*model_idx_itr];

      // write point cloud to file
      std::string pcd_filname = model_type + "_view" + boost::lexical_cast<std::string>(viewNum++) + ".pcd";
      std::string pcd_filename_path = dir + pcd_relative_type_dir + pcd_filname;
      LOG4CXX_INFO(logger, boost::format("pcd filename: %s") % pcd_filename_path);
      if (model.hasRGB)
        pcl::io::savePCDFile(pcd_filename_path, *(model.cloudRGB_));
      else
        pcl::io::savePCDFile(pcd_filename_path, *(model.cloud_));

      // create new object xml node
      boost::property_tree::ptree objectNode;
      objectNode.put("cloud", pcd_relative_type_dir + pcd_filname);

      // add object node to predicate node
      predicateNode.add_child("object", objectNode);
    }

    // add predicate node to processor node
    processorNode.add_child("processor.predicate", predicateNode);
  }

#if BOOST_VERSION < 105600 // Support legacy syntax
  boost::property_tree::xml_parser::write_xml(config, processorNode, std::locale(),
                                              boost::property_tree::xml_writer_settings<char>('\t', 1));
#else
  boost::property_tree::xml_parser::write_xml(config, processorNode, std::locale(), boost::property_tree::xml_writer_settings<std::string>('\t', 1));
#endif
}

void GlobalFeatureValidator::handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification) {
  LOG4CXX_TRACE(logger, "[handleMemoryObjectNotification] method entered.");
  MemoryObject::Ptr object = notification->object;
  pcl::PointCloud<PointType>::ConstPtr scene_cloud = object->getDetectionMask()->getCaptureData()->cloud;

  // check if this typeId should currently be processed
  DescriptorsByTypeConstPtr types = getTypes();
  DescriptorsByType::const_iterator types_itr = types->find(object->getTypeId());
  if (types_itr != types->end()) {

    // get MemoryObject cloud and remove NaNs
    object->lock();
    pcl::PointCloud<PointType>::Ptr cluster_cloud(new pcl::PointCloud<PointType>);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*(object->getDetectionMask()->getObjectPointCloud()), *(cluster_cloud), indices);
    object->unlock();

    // perform classification on point cloud -- and lock while we do it
    models_mutex_.lock();
    std::vector<ModelType> matching_models;

    object->lock();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<int> indicesRGB;
    pcl::removeNaNFromPointCloud(*(object->getDetectionMask()->getObjectPointCloudRGB()), *(cluster_cloudRGB), indicesRGB);
    object->unlock();

    // First find matching models through pointcloud shape comparison, then filter out (hopefully) false matches through color comparison
    //   Decide whether we want to combine the notion of shape and color at once or is one and then the other is good enough
    matching_models = classifyPointCloud(scene_cloud, cluster_cloud);
    LOG4CXX_DEBUG(logger, boost::format("matching models size pre color matching: %d") % matching_models.size());
    
    // Compare model HSV histogram to one created from object MO
    // Model histogram is automatically computed and populated from cloud when object learning

    //REMOVED: otherwise requires <image> tag in global_features.xml - if not present then will perform cloud-based color matching if stored pcd has RGB data
    //           else doesn't do any color matching
    // if (!object->getDetectionMask()->getObjectImage().empty())
    //   // Compare HSV histograms computed from object JPG
    //   matching_models = filterByColor(matching_models, object->getDetectionMask()->getObjectImage().clone());

    // TODO - Make sure this works, but cloud RGB data should always be available
    if (cluster_cloudRGB->size() > 0)
      // Compare HSV histograms computed from object RGB clouds
      matching_models = filterByColor(matching_models, cluster_cloudRGB);
    else
      LOG4CXX_DEBUG(logger, "[GlobalFeatureValidator] No RGBD data for incoming MO, not doing any color matching");
    LOG4CXX_DEBUG(logger, boost::format("filtered models size: %d") % matching_models.size());
    models_mutex_.unlock();


    // update MemoryObject with classification info and send notifications
    if (matching_models.size() > 0) {
      bool validationFound = false;

      // check if validator is actually looking for the detected object type
      // set confidence and descriptors for search typeids
      float confidence = 1.0f; //TODO: set this properly

      PredicateHelper::Set::const_iterator descriptors_itr;
      for (descriptors_itr = types_itr->second.begin(); descriptors_itr != types_itr->second.end(); ++descriptors_itr) {
        if (matching_models[0].type_.compare(descriptors_itr->getName()) == 0 ||
            descriptors_itr->getName().compare("any") == 0) {
          LOG4CXX_DEBUG(logger,
                        boost::format("TypeId: %ld. Descriptor: %s.") % types_itr->first % descriptors_itr->toString());
          if (descriptors_itr->getName().compare("any") == 0) {
            // add "any" and classified type, so that the validation contains all the required descriptors (i.e., "any")
            object->addValidationResult(confidence, PredicateHelper(*descriptors_itr));
            object->addValidationResult(confidence, PredicateHelper(
                    matching_models[0].type_ + "(" + descriptors_itr->getArg(0) + ")"));
          } else {
            object->addValidationResult(confidence, *descriptors_itr);
          }
          validationFound = true;
        }
      }

      if (validationFound) {
        LOG4CXX_DEBUG(logger,
                      boost::format("%s detected. Sending notification.") % matching_models[0].type_);
        sendValidationNotifications(object);
      } else {
        LOG4CXX_DEBUG(logger,
                      boost::format("%s detected but not looking for that category.") % matching_models[0].type_);
      }
    } else {
      LOG4CXX_DEBUG(logger, "No objects detected.");
    }
  }

}

void GlobalFeatureValidator::handleLearningNotification(LearningNotification::ConstPtr notification) {
  if (has_learned_flag_) {
    LOG4CXX_DEBUG(logger, boost::format("Ignoring learn notification until has_learned_flag_ has been reset: %s")
                          % notification->descriptors.begin()->toString());
    return;
  } else {
    LOG4CXX_DEBUG(logger, boost::format("Trying to learn: %s") % notification->descriptors.begin()->toString());
  }

  // get MemoryObject cloud and remove NaNs
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<int> indices;
  MemoryObject::Ptr object = notification->object;
  pcl::removeNaNFromPointCloud(*(object->getDetectionMask()->getObjectPointCloud()), *(cloud), indices);
  if (object->getDetectionMask()->getCaptureData()->hasCloudRGBData())
    pcl::removeNaNFromPointCloud(*(object->getDetectionMask()->getObjectPointCloudRGB()), *(cloudRGB), indices);

  // lock around models data structures
  boost::lock_guard<boost::mutex> models_lock(models_mutex_);

  // build new model of object
  ModelType model;
  if (notification->descriptors.size() != 1) {
    // can only deal with a single descriptor as of now, so check that's the case
    LOG4CXX_WARN(logger,
                 boost::format("Can only handle single learning descriptor (found %d). Using only first one: %s.")
                 % notification->descriptors.size()
                 % notification->descriptors.begin()->toString());
  }
  model.type_ = notification->descriptors.begin()->getArg(1);

  //Model created from object learning will use color matching by default if RGB data is present
  //  this should be fine in most cases because lighting, background, etc should all be same
  //  within single demo where object is taught
  //  Once taught, model properties can be modified in config file if color is not desired
  populateModel(cloud, cloudRGB, cloudRGB->size() > 0, model);
  // populateModel(cloud, cloudRGB, object->getDetectionMask()->getObjectImage(), model);
  // populateModel(cloud, model);

  // add new model to local list of models
  models_.push_back(model);
  int model_idx = models_.size() - 1;
  type_to_model_[model.type_].push_back(model_idx);

  // add model's descriptors to local data structures for descriptor to model lookup
  pcl::PointCloud<DescriptorType>::const_iterator descriptor_iter;
  for (descriptor_iter = model.descriptors_->begin(); descriptor_iter != model.descriptors_->end(); ++descriptor_iter) {
    model_descriptors_->push_back(*descriptor_iter);
    descriptor_to_model_[model_descriptors_->size() - 1] = model_idx;
  }

  // re-build nn search tree
  match_search_.setInputCloud(model_descriptors_);

  // set has learned flag
  boost::lock_guard<boost::recursive_mutex> has_learned_lock(has_learned_mutex_);
  has_learned_flag_ = true;

  LOG4CXX_DEBUG(logger, boost::format("Successfully learned: %s") % model.type_);

  // save learned thing to file
  //saveConfig(configFile + ".auto_save");
}

void GlobalFeatureValidator::displayResults(const pcl::PointCloud<PointType>::ConstPtr &scene_cloud,
                                            const std::vector<pcl::PointCloud<PointType>::ConstPtr> &model_clouds_aligned,
                                            const std::vector<ModelType> &matching_models,
                                            const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &transforms) {

  pcl::visualization::PointCloudColorHandlerCustom<PointType>::Ptr scene_color_handler(
          new pcl::visualization::PointCloudColorHandlerCustom<PointType>(scene_cloud, 0, 255, 0));
  ade::Display::displayPointCloud(scene_cloud, scene_color_handler, "scene_cloud", getDisplayName());

  // display aligned model candidates
  pcl::PointCloud<PointType>::Ptr models_aligned_cloud(new pcl::PointCloud<PointType>);
  for (size_t i = 0; i < model_clouds_aligned.size(); i++) {
    models_aligned_cloud->operator+=(*model_clouds_aligned[i]);
  }
  pcl::visualization::PointCloudColorHandlerCustom<PointType>::Ptr models_aligned_color_handler(
          new pcl::visualization::PointCloudColorHandlerCustom<PointType>(models_aligned_cloud, 255, 255, 0));
  ade::Display::displayPointCloud(models_aligned_cloud, models_aligned_color_handler, "aligned_candidates_cloud",
                                  getDisplayName());

  // display matching models and pose aligned matching models
  pcl::PointCloud<PointType>::Ptr matching_models_cloud(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr pose_aligned_matching_models(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr tmp_cloud(new pcl::PointCloud<PointType>());
  for (size_t i = 0; i < matching_models.size(); i++) {
    matching_models_cloud->operator+=(*(matching_models[i].cloud_));

    tmp_cloud->clear();
    pcl::transformPointCloud(*matching_models[i].cloud_, *tmp_cloud, transforms[i]);
    pose_aligned_matching_models->operator+=(*tmp_cloud);

  }
  pcl::visualization::PointCloudColorHandlerCustom<PointType>::Ptr matching_models_color_handler(
          new pcl::visualization::PointCloudColorHandlerCustom<PointType>(matching_models_cloud, 0, 0, 255));
  ade::Display::displayPointCloud(matching_models_cloud, matching_models_color_handler, "matching_models_cloud",
                                  getDisplayName());

  pcl::visualization::PointCloudColorHandlerCustom<PointType>::Ptr pose_aligned_scene_color_handler(
          new pcl::visualization::PointCloudColorHandlerCustom<PointType>(pose_aligned_matching_models, 255, 0, 0));
  ade::Display::displayPointCloud(pose_aligned_matching_models, pose_aligned_scene_color_handler,
                                  "aligned_matching_models_cloud",
                                  getDisplayName());

  //so we can actually see what's happening before it gets updated with the next object
  //sleep(3.0);
}

bool GlobalFeatureValidator::hasLearned() const {
  boost::lock_guard<boost::recursive_mutex> has_learned_lock(has_learned_mutex_);
  return has_learned_flag_;
}

void GlobalFeatureValidator::resetHasLearned() {
  boost::lock_guard<boost::recursive_mutex> has_learned_lock(has_learned_mutex_);
  has_learned_flag_ = false;
}

////////////////////////////////////////////////////////////////////////////////
/////////////// TODO: put everything below this in separate util file //////////
////////////////////////////////////////////////////////////////////////////////

// Create model from xml file in loadConfig
// bool GlobalFeatureValidator::createModel(const std::string &model_file, const std::string &model_image_file, bool use_color, ModelType &model) {
bool GlobalFeatureValidator::createModel(const std::string &model_file, bool use_color, ModelType &model) {
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
  cv::Mat modelImage;

  // Always try to load model cloud
  if (pcl::io::loadPCDFile(model_file, *(cloud)) < 0) {
    LOG4CXX_ERROR(logger, boost::format("Error loading model cloud: %s") % model_file);
    return false;
  }

  // is use_color is set to true, attempt loading rgb image or rgbd cloud of object
  if (use_color) {
    //*Removing image color capabilities
    // load image to be used for color similarity comparison if available
    // if (!model_image_file.empty()) {
    //   LOG4CXX_DEBUG(logger, boost::format("[createModel] loading model image with file: %s") % model_image_file);
    //   modelImage = cv::imread(model_image_file, cv::IMREAD_COLOR);
    //   if (modelImage.empty()) LOG4CXX_WARN(logger, "[createModel] loaded model image, but image is empty - check provided image path is correct");
    // }
    // else
    //   LOG4CXX_WARN(logger, "[createModel] model_image_file empty, not loading model image");

    // load rgbd cloud data for color comparison if available
    if (pcl::io::loadPCDFile(model_file, *(cloudRGB)) < 0) {
      LOG4CXX_WARN(logger, boost::format("Model set to use_color, but error loading modelRGB cloud: %s") % model_file);
    }
  }

  // return populateModel(cloud, cloudRGB, modelImage, model);
  return populateModel(cloud, cloudRGB, use_color, model);
}

// populate model without any RGB cloud or HSV hist data
//  used for temp model creation when classifying point clouds and comparing to stored models
bool GlobalFeatureValidator::populateModel(pcl::PointCloud<PointType>::ConstPtr cloud, ModelType &model) {
  //model.cloud_ = cloud;

  //
  // make uniform resolution
  //
  //makeUniformResolution(cloud, model.cloud_);
  model.cloud_ = cloud->makeShared();

  //
  //  Compute Normals
  //
  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
  norm_est.setKSearch(10);
  norm_est.setInputCloud(model.cloud_);
  norm_est.compute(*(model.normals_));

  //
  //  Compute Descriptor
  //
  if (feature_type_ == 0) {
    computeVFHDescriptors(model.cloud_, model.normals_, model.descriptors_);
  } else if (feature_type_ == 1) {
    //  computeOURCVFHDescriptors(model, model_normals, model_descriptors);
  } else if (feature_type_ == 2) {

  } else {
    LOG4CXX_ERROR(logger, "Trying to use unknown feature type.");
    return false;
  }

  //
  // Compute Camera Roll Histogram
  //
  computeCRH(model.cloud_, model.normals_, model.centroid_, model.crh_);

  return true;
}

void GlobalFeatureValidator::printMatrix4f(const Eigen::Matrix4f &matrix, const std::string &matrix_name) {
  // Print the rotation matrix and translation vector
  //  Eigen::Matrix3f rotation = matrix.block<3, 3>(0, 0);
  //  Eigen::Vector3f translation = matrix.block<3, 1>(0, 3);

  LOG4CXX_DEBUG(logger, boost::format("%s: \n") % matrix_name.c_str());
  LOG4CXX_DEBUG(logger,
                boost::format("            | %6.3f %6.3f %6.3f | \n") % matrix(0, 0) % matrix(0, 1) % matrix(0, 2));
  LOG4CXX_DEBUG(logger,
                boost::format("        R = | %6.3f %6.3f %6.3f | \n") % matrix(1, 0) % matrix(1, 1) % matrix(1, 2));
  LOG4CXX_DEBUG(logger,
                boost::format("            | %6.3f %6.3f %6.3f | \n") % matrix(2, 0) % matrix(2, 1) % matrix(2, 2));
  LOG4CXX_DEBUG(logger, boost::format("\n"));
  LOG4CXX_DEBUG(logger,
                boost::format("        t = < %0.3f, %0.3f, %0.3f >\n") % matrix(0, 3) % matrix(1, 3) % matrix(2, 3));
}

void GlobalFeatureValidator::transRotToPose(const Eigen::Vector4f &translation, const Eigen::Quaternionf &orientation,
                                            Eigen::Matrix4f &pose) {
  Eigen::Affine3f r(orientation);
  Eigen::Affine3f t(Eigen::Translation3f(translation(0), translation(1), translation(2)));
  //  printf("q = < %0.3f, %0.3f, %0.3f, %0.3f >\n", orientation.x(), orientation.y(), orientation.z(), orientation.w());
  //  printf("t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));

  pose = (t * r).matrix();
  //printMatrix4f(pose, "pose");
}

float GlobalFeatureValidator::computeMeshResolution(pcl::PointCloud<PointType>::ConstPtr input) {
  typedef typename pcl::KdTree<PointType>::Ptr KdTreeInPtr;

#if PCL_VERSION_COMPARE(>=, 1, 12, 0)
  KdTreeInPtr tree = std::make_shared<pcl::KdTreeFLANN<PointType> >(false);
#else
  KdTreeInPtr tree = boost::make_shared<pcl::KdTreeFLANN<PointType> >(false);
#endif

  tree->setInputCloud(input);

  std::vector<int> nn_indices(9);
  std::vector<float> nn_distances(9);
  std::vector<int> src_indices;

  float sum_distances = 0.0;
  std::vector<float> avg_distances(input->points.size());
  // Iterate through the source data set
  for (size_t i = 0; i < input->points.size(); ++i) {
    tree->nearestKSearch(input->points[i], 9, nn_indices, nn_distances);

    float avg_dist_neighbours = 0.0;
    for (size_t j = 1; j < nn_indices.size(); j++)
      avg_dist_neighbours += sqrtf(nn_distances[j]);

    avg_dist_neighbours /= static_cast<float> (nn_indices.size());

    avg_distances[i] = avg_dist_neighbours;
    sum_distances += avg_dist_neighbours;
  }

  std::sort(avg_distances.begin(), avg_distances.end());
  float avg = avg_distances[static_cast<int> (avg_distances.size()) / 2 + 1];
  return avg;
}

void GlobalFeatureValidator::makeUniformResolution(pcl::PointCloud<PointType>::ConstPtr cloud,
                                                   pcl::PointCloud<PointType>::Ptr &cloud_resampled) {

  LOG4CXX_DEBUG(logger, boost::format("[makeUniformResolution] input cloud points: %lu, isOrganized: %s.") %
                        cloud->points.size() %
                        (cloud->isOrganized() ? "true" : "false"));
  //  pcl::visualization::PCLVisualizer viewer("MLS");
  //  viewer.addPointCloud(cloud, "cloud");
  //  viewer.spin();

  //
  // Moving Least Squares (MLS) Upsampling
  //

  pcl::PointCloud<PointType>::Ptr out(new pcl::PointCloud<PointType>());

  //  pcl::MovingLeastSquares<PointType, PointType> mls;
  //  typename pcl::search::KdTree<PointType>::Ptr tree;
  //  Eigen::Vector4f centroid_cluster;
  //  pcl::compute3DCentroid(*cloud, centroid_cluster);
  //  float dist_to_sensor = centroid_cluster.norm();
  //  float sigma = dist_to_sensor * 0.01f;
  //  mls.setInputCloud(cloud);
  //  mls.setSearchMethod(tree);
  //  mls.setSearchRadius(sigma);
  //  mls.setUpsamplingMethod(mls.SAMPLE_LOCAL_PLANE);
  //  mls.setUpsamplingRadius(0.005);//(0.002);
  //  mls.setUpsamplingStepSize(0.001);
  //
  //  mls.process(*out);
  //  
  //  printf("mls cloud points: %lu, isOrganized: %s.\n", out->points.size(), (out->isOrganized() ? "true" : "false"));
  //  if (out->points.size() == 0) {
  //    PCL_WARN("NORMAL estimator: Cloud has no points after mls, wont be able to compute normals!\n");
  //    return;
  //  }
  //  viewer.removeAllPointClouds();
  //  viewer.addPointCloud(out, "cloud");
  //  viewer.spin();

  //
  // Downsample to set resolution
  //

  bool compute_mesh_resolution = true;
  float voxel_grid_size = 0.009f; //0.003f;
  if (compute_mesh_resolution) {
    float factor_voxel_grid = 3.0f; //3.0f;
    float cloud_resolution = computeMeshResolution(cloud);
    voxel_grid_size = cloud_resolution * factor_voxel_grid;
    LOG4CXX_DEBUG(logger, boost::format("cloud resolution: %0.03f voxel grid resolution: %0.03f") % cloud_resolution %
                          voxel_grid_size);
  }

  pcl::PointCloud<PointType>::Ptr out2(new pcl::PointCloud<PointType>());
  pcl::VoxelGrid<PointType> grid;
  grid.setInputCloud(cloud);
  //  grid.setInputCloud(out);
  grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
  grid.setDownsampleAllData(true);
  grid.filter(*out2);
  out = out2;

  LOG4CXX_DEBUG(logger, boost::format("output cloud points: %lu, isOrganized: %s.")
                        % out->points.size() % (out->isOrganized() ? "true" : "false"));
  if (out->points.size() == 0) {
    PCL_WARN("NORMAL estimator: Cloud has no points after voxel grid, wont be able to compute normals!\n");
    return;
  }
  //  viewer.removeAllPointClouds();
  //  viewer.addPointCloud(out2, "cloud");
  //  viewer.spin();

  cloud_resampled = out;

}

void GlobalFeatureValidator::computeVFHDescriptors(pcl::PointCloud<PointType>::ConstPtr cloud,
                                                   pcl::PointCloud<NormalType>::ConstPtr normals,
                                                   pcl::PointCloud<DescriptorType>::Ptr &descriptors) {

  // Create the VFH estimation class, and pass the input dataset+normals to it

  pcl::VFHEstimation<PointType, NormalType, DescriptorType> vfh;
  vfh.setInputCloud(cloud);
  vfh.setInputNormals(normals);

  // Create an empty kdtree representation, and pass it to the FPFH estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
  vfh.setSearchMethod(tree);

  // Output datasets
  //pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

  // Compute the features
  vfh.compute(*descriptors);

  // vfhs->points.size () should be of size 1*
}

//void GlobalFeatureValidator::computeOURCVFHDescriptors(pcl::PointCloud<PointType>::ConstPtr cloud,
//        pcl::PointCloud<NormalType>::ConstPtr normals,
//        pcl::PointCloud<DescriptorType>::Ptr descriptors) {
//
//  pcl::OURCVFHEstimation<PointType, NormalType, DescriptorType> ourcvfh;
//  pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>);
//  ourcvfh.setSearchMethod(kdtree);
//  ourcvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI); // 5 degrees.
//  ourcvfh.setCurvatureThreshold(1.0);
//  ourcvfh.setNormalizeBins(false);
//  // Set the minimum axis ratio between the SGURF axes. At the disambiguation phase,
//  // this will decide if additional Reference Frames need to be created, if ambiguous.
//  ourcvfh.setAxisRatio(0.8);
//
//  ourcvfh.setInputCloud(cloud);
//  ourcvfh.setInputNormals(normals);
//  ourcvfh.compute(*descriptors);
//}

void GlobalFeatureValidator::computeCRH(pcl::PointCloud<PointType>::ConstPtr cloud,
                                        pcl::PointCloud<NormalType>::ConstPtr normals,
                                        Eigen::Vector4f &centroid,
                                        pcl::PointCloud<pcl::Histogram<90> >::Ptr &histogram) {

  // CRH estimation object.

  pcl::CRHEstimation<PointType, NormalType, pcl::Histogram<90> > crh;
  crh.setInputCloud(cloud);
  crh.setInputNormals(normals);
  pcl::compute3DCentroid(*cloud, centroid);
  crh.setCentroid(centroid);

  // Compute the CRH.
  crh.compute(*histogram);
}

bool GlobalFeatureValidator::estimatePose(pcl::PointCloud<PointType>::Ptr model,
                                          pcl::PointCloud<PointType>::Ptr scene,
                                          const Eigen::Vector4f &model_centroid,
                                          const Eigen::Vector4f &scene_centroid,
                                          pcl::PointCloud<pcl::Histogram<90> >::Ptr model_crh,
                                          pcl::PointCloud<pcl::Histogram<90> >::Ptr scene_crh,
                                          Eigen::Matrix4f &pose) {

  //LOG4CXX_DEBUG(logger, boost::format("model centroid: (%0.3f, %0.3f, %0.3f)") % model_centroid(0) % model_centroid(1) %
  //                      model_centroid(2));
  //LOG4CXX_DEBUG(logger,
  //              boost::format("cluster centroid: (%0.3f, %0.3f, %0.3f)") % scene_centroid(0) % scene_centroid(1) %
  //              scene_centroid(2));

  // CRHAlignment works with Vector3f, not Vector4f.
  Eigen::Vector3f model_centroid3f(model_centroid(0), model_centroid(1), model_centroid(2));
  Eigen::Vector3f scene_centroid3f(scene_centroid(0), scene_centroid(1), scene_centroid(2));

  pcl::CRHAlignment<PointType, 90> alignment;
  //alignment.setInputAndTargetView(model, scene);
  alignment.setInputAndTargetCentroids(model_centroid3f, scene_centroid3f);
  alignment.align(*model_crh, *scene_crh);

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > roll_transforms;
  alignment.getTransforms(roll_transforms);

  if (roll_transforms.size() < 1) {
    LOG4CXX_WARN(logger, "No roll could be generated. Returning without a pose estimate.");
    return false;
  }

  //if (logger->isDebugEnabled()) {
  //  LOG4CXX_DEBUG(logger, "got pose aligned...");
  //  printMatrix4f(roll_transforms[0], "roll_transforms[0]");
  //}

  Eigen::Matrix4f model_pose;
  transRotToPose(model->sensor_origin_, model->sensor_orientation_, model_pose);

  //if (logger->isDebugEnabled()) {
  //  LOG4CXX_DEBUG(logger, "computing final pose...");
  //  printMatrix4f(roll_transforms[0], "roll_transforms[0]");
  //  printMatrix4f(model_pose, "model_pose");
  //}
  pose = Eigen::Matrix4f(roll_transforms[0] * model_pose);

  //LOG4CXX_DEBUG(logger, "computed final pose.");

  // show CRHs
  //  pcl::visualization::PCLHistogramVisualizer crh_visualizer;
  //  crh_visualizer.addFeatureHistogram<pcl::Histogram<90> >(*model_crh, 90, "model feature");
  //  crh_visualizer.addFeatureHistogram<pcl::Histogram<90> >(*scene_crh, 90, "scene feature");
  //  crh_visualizer.spin();

  return true;
}

std::vector<GlobalFeatureValidator::ModelType>
GlobalFeatureValidator::classifyPointCloud(const pcl::PointCloud<PointType>::ConstPtr &scene_cloud,
                                           const pcl::PointCloud<PointType>::ConstPtr &object_cloud) {

  pcl::ScopeTime t("Classification");


  // create model of incoming cluster
  LOG4CXX_DEBUG(logger,
                boost::format("Creating new incoming cluster object_cloud size: %lu") % object_cloud->points.size());
  ModelType object_cluster;
  populateModel(object_cloud, object_cluster);

  //
  //  Matching: Find Model-Scene Correspondences with KdTree
  //

  pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());
  {
    pcl::ScopeTime t("NN Matching");

    //  For each scene descriptor, find nearest neighbor into the model descriptor cloud and add it to the correspondences vector.
    for (size_t i = 0; i < object_cluster.descriptors_->size(); ++i) {
      int num_neighs = 1; //TODO: update code below to handle +1 neighbors
      std::vector<int> neigh_indices(num_neighs);
      std::vector<float> neigh_sqr_dists(num_neighs);
      //skipping NaNs
      if (!std::isfinite(object_cluster.descriptors_->at(i).histogram[0])) {
        continue;
      }
      int found_neighs = match_search_.nearestKSearch(object_cluster.descriptors_->at(i), num_neighs, neigh_indices,
                                                      neigh_sqr_dists);

      LOG4CXX_DEBUG(logger, boost::format("neigh_sqr_dists[0]: %f") % neigh_sqr_dists[0]);
      if (found_neighs == num_neighs) {
        //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
        //      if (found_neighs == num_neighs && neigh_sqr_dists[0] < 0.35f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)

        pcl::Correspondence corr(static_cast<int> (i), neigh_indices[0], neigh_sqr_dists[0]);
        model_scene_corrs->push_back(corr);
      }
    }
    LOG4CXX_DEBUG(logger, boost::format("Correspondences found: %lu") % model_scene_corrs->size());
  }

  //
  // Pose estimation
  //
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transforms;
  std::vector<ModelType> matching_models;
  {
    pcl::ScopeTime t("Pose estimation");

    //get matching models
    ModelType matching_model;
    Eigen::Matrix4f transform;
    for (int i = 0; i < model_scene_corrs->size(); ++i) {
      int model_idx = descriptor_to_model_[model_scene_corrs->at(0).index_match];
      matching_model = models_[model_idx];
      LOG4CXX_DEBUG(logger, boost::format("matching model: %s.") % matching_model.type_);

      if (estimatePose(matching_model.cloud_, object_cluster.cloud_, matching_model.centroid_, object_cluster.centroid_,
                       matching_model.crh_, object_cluster.crh_, transform)) {
        matching_models.push_back(matching_model);
        transforms.push_back(transform);   
      }
    }
  }

  if (matching_models.size() < 1) {
    LOG4CXX_DEBUG(logger, "No matching models found.");
    return matching_models;
  }

  //
  // ICP pose refinement
  //
  std::vector<pcl::PointCloud<PointType>::ConstPtr> model_clouds_aligned;
  if (true) {
    pcl::ScopeTime t("Pose refinement");

    //Prepare scene and model clouds for the pose refinement step
    float VOXEL_SIZE_ICP_ = 0.005f;
    int ICP_iterations_ = 5;
    pcl::PointCloud<PointType>::Ptr model_voxelized_icp(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr object_cluster_voxelized_icp(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr model_cloud_aligned(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr model_cloud_icp_aligned(new pcl::PointCloud<PointType>());

    pcl::VoxelGrid<PointType> voxel_grid_icp;
    voxel_grid_icp.setLeafSize(VOXEL_SIZE_ICP_, VOXEL_SIZE_ICP_, VOXEL_SIZE_ICP_);
    voxel_grid_icp.setInputCloud(object_cluster.cloud_);
    voxel_grid_icp.filter(*object_cluster_voxelized_icp);

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaximumIterations(ICP_iterations_);
    icp.setMaxCorrespondenceDistance(VOXEL_SIZE_ICP_ * 3.f);
    icp.setTransformationEpsilon(1e-5);

    for (int i = 0; i < matching_models.size(); ++i) {
      voxel_grid_icp.setInputCloud(matching_models[i].cloud_);
      voxel_grid_icp.filter(*model_voxelized_icp);

      pcl::transformPointCloud(*model_voxelized_icp, *model_cloud_aligned, transforms[i]);

      icp.setInputSource(model_cloud_aligned); //model
      icp.setInputTarget(object_cluster_voxelized_icp); //scene
      icp.align(*model_cloud_icp_aligned);

      Eigen::Matrix4f icp_trans = icp.getFinalTransformation();
      transforms[i] = icp_trans * transforms[i];

      model_clouds_aligned.push_back(model_cloud_icp_aligned);
      //pcl::transformPointCloud(*matching_models[i].cloud_, *model_cloud_aligned, transforms[i]);
      //model_clouds_aligned.push_back(model_cloud_aligned);

      if (icp.hasConverged()) {
        LOG4CXX_DEBUG(logger,
                      boost::format("ICP pose refinement converged. Fitness score: %f.") % icp.getFitnessScore());
      } else {
        LOG4CXX_DEBUG(logger, boost::format("ICP pose refinement did not converge. Fitness score: %f.") %
                              icp.getFitnessScore());
      }
    }
  }

  if (logger->isDebugEnabled()) {
    LOG4CXX_DEBUG(logger, boost::format("Matching models: %lu. Transforms: %lu. Model clouds aligned: %lu.")
                          % matching_models.size() % transforms.size() % model_clouds_aligned.size());
    LOG4CXX_DEBUG(logger, boost::format("object_cluster cloud size: %lu.")
                          % object_cluster.cloud_->size());
    for (int i = 0; i < model_clouds_aligned.size(); ++i) {
      LOG4CXX_DEBUG(logger, boost::format("model_clouds_aligned[%d] cloud size: %lu.")
                            % i % model_clouds_aligned[i]->size());

      pcl::PointCloud<PointType> downsampled_model_cloud_aligned;
      float size_model = 0.005;
      pcl::VoxelGrid<PointType> voxel_grid2;
      voxel_grid2.setInputCloud(model_clouds_aligned[i]);
      voxel_grid2.setLeafSize(size_model, size_model, size_model);
      voxel_grid2.filter(downsampled_model_cloud_aligned);
      LOG4CXX_DEBUG(logger, boost::format("downsampled model_clouds_aligned[%d] cloud size: %lu.")
                            % i % downsampled_model_cloud_aligned.size());
    }

  }

  //
  // Hypothesis verification
  //
  std::vector<bool> hypotheses_mask; // Mask Vector to identify positive hypotheses
  hypotheses_mask.resize(matching_models.size(), false);
  if (true) {
    pcl::ScopeTime t("Hypothesis verification");

    // copy scene_cloud bc the PCL setSceneCloud method doesn't accept a ConstPtr as it should
    pcl::PointCloud<PointType>::Ptr scene_cloud_nonconst = pcl::PointCloud<PointType>::Ptr(
            new pcl::PointCloud<PointType>(*scene_cloud));
    ghv_.setSceneCloud(scene_cloud_nonconst); // Scene Cloud
    bool occlusion_reasoning = false;
    if (occlusion_reasoning) {
      ghv_.setOcclusionCloud(scene_cloud_nonconst); // occlusion cloud
      //ghv_.setSceneCloud(object_cluster.cloud_); // Object Cloud
      ghv_.addModels(model_clouds_aligned, true); //Models to verify
    } else {
      ghv_.addModels(model_clouds_aligned, false); //Models to verify
      ghv_.addCompleteModels(model_clouds_aligned); //Models to verify
    }

    try {
      ghv_.verify();
      ghv_.getMask(hypotheses_mask); // i-element TRUE if hvModels[i] verifies hypotheses
    } catch (...) {
      LOG4CXX_ERROR(logger, "Hypothesis verification catch all.");
      hypotheses_mask.clear();
    }

    std::vector<ModelType> matching_models_temp;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transforms_temp;
    LOG4CXX_DEBUG(logger, boost::format("Hypotheses to test: %lu.") % matching_models.size());
    for (size_t i = 0; i < matching_models.size(); i++) {
      if (hypotheses_mask[i]) {
        LOG4CXX_DEBUG(logger, boost::format("Hypothesis %d (%s) passed!") % i % matching_models[i].type_);
        matching_models_temp.push_back(matching_models[i]);
        transforms_temp.push_back(transforms[i]);
      } else {
        LOG4CXX_DEBUG(logger, boost::format("Hypothesis %d (%s) failed!") % i % matching_models[i].type_);
      }
    }

    // reset matching models and transforms
    matching_models = matching_models_temp;
    transforms = transforms_temp;
  }

  //
  //  Output results
  //
  LOG4CXX_DEBUG(logger, boost::format("Model instances found: %lu.") % matching_models.size());
  //if (logger->isTraceEnabled()) {
  //  for (size_t i = 0; i < transforms.size(); ++i) {
  //    //      std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
  //    //      std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size() << std::endl;
  //
  //    printMatrix4f(transforms[i], "transforms[i]");
  //  }
  //}

  //
  // display results
  //
  if (getDisplayFlag()) {
    displayResults(scene_cloud, model_clouds_aligned, matching_models, transforms);
  }

  return matching_models;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//// Leaving image color matching methods commented because seems unnecessary given cloud 
////   rgbd data and clutters config, can add back in if ever needed

// Color comparison through model and object RGB images
// std::vector<GlobalFeatureValidator::ModelType> GlobalFeatureValidator::filterByColor(std::vector<ModelType> matching_models, cv::Mat object_image) {
//   std::vector<ModelType> filtered_models;
//   cv::Mat obj_hist;
//   bool computed_hist = false;

//   for (int i = 0; i < matching_models.size(); i++) {
//     cv::Mat model_hist = matching_models[i].imageHist_;
//     if (model_hist.empty()) {
//       LOG4CXX_DEBUG(logger, "No (image)histogram found for model, keeping as match without comparing color");
//       filtered_models.push_back(matching_models[i]);
//       continue;
//     }
//     if (!computed_hist) {
//       obj_hist = computeImageHist(object_image);
//       computed_hist = true;
//     }

//     LOG4CXX_DEBUG(logger, "Computed cloudHist for model");
//     double dist = cv::compareHist(obj_hist, model_hist, 0);
//     LOG4CXX_DEBUG(logger, boost::format("color similarity: %d, thresh: %d.") % dist % 0.45);

//     if (dist > 0.45) {
//       LOG4CXX_DEBUG(logger, "Hist Match!");
//       filtered_models.push_back(matching_models[i]);
//     }
//   }
//   return filtered_models;
// }

// cv::Mat GlobalFeatureValidator::computeImageHist(cv::Mat object_image) {//,
//                                                  // const int nbins) {//,
//                                                  // const int ndims,
//                                                  // const float maxRange) {
//   LOG4CXX_DEBUG(logger, "in compute image hist");

//   // histogram parameters
//   int h_bins = 10, s_bins = 16;
//   int hist_size[] = { h_bins, s_bins };
//   // hue varies from 0 to 179, saturation from 0 to 255
//   // TODO: clipped values to filter out black points - do more elegantly
//   float h_ranges[] = { 1, 179 };
//   float s_ranges[] = { 1, 255 };
//   const float* ranges[] = { h_ranges, s_ranges };
//   // Use the 0-th and 1-st channels
//   int channels[] = { 0, 1 };

//   // compute histogram for detected object
//   cv::cvtColor(object_image, object_image, cv::COLOR_RGB2HSV);
//   cv::Mat obj_hist;
//   cv::calcHist(&object_image, 1, channels, cv::Mat(), obj_hist, 2, hist_size, ranges, true, false);
//   cv::normalize(obj_hist, obj_hist, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

//   return obj_hist;
// }
///////////////////////////////////////////////////////////////////////////////////////////////////


// Computes color histogram of a pcl pointXYZRGB cloud for use in color comparison
cv::Mat GlobalFeatureValidator::computeCloudHist(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloudRGB,
                                                 const int h_bins = 10,
                                                 const int s_bins = 16) {
  LOG4CXX_DEBUG(logger, "in compute cloud hist");
  // Prevent 360 or 1 from computing bin value out of range, fix
  const float max_h = 360.1;
  const float max_s = 1.001;
  std::vector< std::vector<float> > hist(h_bins, std::vector<float>(s_bins));
  float divisor_h = max_h/(float)h_bins;
  float divisor_s = max_s/(float)s_bins;

  for (auto &point : cloudRGB->points) {
    // float rf = (long)point.r;
    // float gf = (long)point.g;
    // float bf = (long)point.b;
    // LOG4CXX_DEBUG(logger, boost::format("float rgb: %d,%d,%d") % rf % gf % bf);

    pcl::PointXYZHSV point_HSV;
    pcl::PointXYZRGBtoXYZHSV(point, point_HSV);

    // LOG4CXX_DEBUG(logger, boost::format("float HSV: %d,%d,%d") % point_HSV.h % point_HSV.s % point_HSV.v);

    int h_bin = floor(point_HSV.h/divisor_h);
    int s_bin = floor(point_HSV.s/divisor_s);
    hist[h_bin][s_bin]++;
  }

  // TODO: find better way to copy manually computed hist to CV mat
  cv::Mat hist_mat = cv::Mat(h_bins, s_bins, CV_32F);
  for (int i = 0; i < h_bins; i++) {
    for (int j = 0; j < s_bins; j++) {
      hist_mat.at<float>(i,j) = hist[i][j];
    }
  }
  cv::normalize(hist_mat, hist_mat, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

  return hist_mat;
}

// Iterates through list of candidate models matching given MO from notification and
//   filters out mistaken candidates based on color similarity between MO RGB cloud
//   and candidate model RGB clouds
// Args:
//   std:vector<ModelType> matching models: candidate models chosen for processed MO
//   const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &object_cloudRGB: RGB cloud of processed MO
// Returns: Filtered list of matching models based on color similarity
std::vector<GlobalFeatureValidator::ModelType> GlobalFeatureValidator::filterByColor(std::vector<ModelType> matching_models, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &object_cloudRGB) {
  std::vector<ModelType> filtered_models;
  // compute histogram for detected object
  cv::Mat obj_hist;
  bool computed_hist = false;

  for (int i = 0; i < matching_models.size(); i++) {
    if (!matching_models[i].hasRGB) {
      LOG4CXX_DEBUG(logger, "use_color set to true but no cloudRGB data found for model, keeping as match without comparing color");
      filtered_models.push_back(matching_models[i]);
      continue;
    }

    // Don't unnecessarily compute object hist if no models have RGB data
    // TODO: probaby want to store in model when created similar to how is done given image
    if (!computed_hist) {
      obj_hist = computeCloudHist(object_cloudRGB);
      computed_hist = true;
    }

    cv::Mat model_hist = matching_models[i].cloudHist_;

    // cout << "model_hist_cloud = " << endl << " "  << modelHistMat << endl << endl;
    // cout << "object_hist_cloud = " << endl << " "  << objHistMat << endl << endl;

    LOG4CXX_DEBUG(logger, "Computed cloudHist for model");
    double dist = cv::compareHist(obj_hist, model_hist, 0);
    LOG4CXX_DEBUG(logger, boost::format("color similarity: %d, thresh: %d.") % dist % 0.45);

    if (dist > 0.45) {
      LOG4CXX_DEBUG(logger, "CloudHist Match!");
      filtered_models.push_back(matching_models[i]);
    }
  }
  return filtered_models;
}

//Adapted to handle any case - whether one, both, or neither of RGBCloud and Image are empty
// bool GlobalFeatureValidator::populateModel(pcl::PointCloud<PointType>::ConstPtr cloud,pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudRGB, cv::Mat objectImage, bool use_color,ModelType &model) {
bool GlobalFeatureValidator::populateModel(pcl::PointCloud<PointType>::ConstPtr cloud,pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudRGB, bool use_color, ModelType &model) {
  if (use_color) {
    // Check if cloud is empty
    if (cloudRGB->size() > 0) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredcloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
      std::vector<int> indicesRGB;
      pcl::removeNaNFromPointCloud(*cloudRGB, *(filteredcloudRGB), indicesRGB);
      LOG4CXX_DEBUG(logger, "Populating model with rgbcloud");
      model.cloudRGB_ = filteredcloudRGB->makeShared();
      model.hasRGB = true;
      cv::Mat model_hist = computeCloudHist(filteredcloudRGB);
      model.cloudHist_ = model_hist;
    }
    else
      LOG4CXX_WARN(logger, "use_color set to true in config but no RGBD data available for model");
}
// Deprecating imageHist methods //
  //   // If object Image isn't empty, compute HSV hist for model
  //   if (!objectImage.empty()) {
  //     LOG4CXX_DEBUG(logger, "Have objectImage for model, computing hist");
  //     // TODO: save image in correct file location and update config for object learning
  //     // cv::imwrite("/home/dev/test_image.jpg", objectImage);

  //     // histogram parameters
  //     int h_bins = 10, s_bins = 16;
  //     int histSize[] = { h_bins, s_bins };
  //     // hue varies from 0 to 179, saturation from 0 to 255
  //     float h_ranges[] = { 1, 179 };
  //     float s_ranges[] = { 1, 255 };
  //     const float* ranges[] = { h_ranges, s_ranges };
  //     // Use the 0-th and 1-st channels
  //     int channels[] = { 0, 1 };

  //     // compute histogram for detected object
  //     // LOG4CXX_DEBUG(logger, "Computing cloudHist for model");
  //     cv::cvtColor(objectImage, objectImage, cv::COLOR_RGB2HSV);
  //     cv::Mat objHist;
  //     cv::calcHist(&objectImage, 1, channels, cv::Mat(), objHist, 2, histSize, ranges, true, false);
  //     cv::normalize(objHist, objHist, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
  //     model.imageHist_ = objHist;
  //     LOG4CXX_DEBUG(logger, "Populating model hist");
  //   }
  // }
// Deprecating imageHist methods //

  //
  // make uniform resolution
  //
  //makeUniformResolution(cloud, model.cloud_);
  model.cloud_ = cloud->makeShared();

  //
  //  Compute Normals
  //
  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
  norm_est.setKSearch(10);
  norm_est.setInputCloud(model.cloud_);
  norm_est.compute(*(model.normals_));

  //
  //  Compute Descriptor
  //
  if (feature_type_ == 0) {
    computeVFHDescriptors(model.cloud_, model.normals_, model.descriptors_);
  } else if (feature_type_ == 1) {
    //  computeOURCVFHDescriptors(model, model_normals, model_descriptors);
  } else if (feature_type_ == 2) {

  } else {
    LOG4CXX_ERROR(logger, "Trying to use unknown feature type.");
    return false;
  }

  //
  // Compute Camera Roll Histogram
  //
  computeCRH(model.cloud_, model.normals_, model.centroid_, model.crh_);

  return true;
}
