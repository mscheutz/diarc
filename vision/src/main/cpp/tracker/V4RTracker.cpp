/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "V4RTracker.hpp"
#include "capture/calibration/Cameras.hpp"
#include <boost/algorithm/string.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/register_point_struct.h>
#include <v4r/common/convertCloud.h>
#include <v4r/common/convertImage.h>
#include <v4r/features/FeatureDetector_KD_FAST_IMGD.h>
#include <v4r/keypoints/CodebookMatcher.h>
#include <v4r/keypoints/io.h>
#include <v4r/keypoints/impl/PoseIO.hpp>
#include <v4r/io/filesystem.h>

#ifdef HAVE_OCV_2
#include <v4r/KeypointBase/FeatureDetector_KD_ORB-ocv2.4.hh>
#else

#include <v4r/features/FeatureDetector_KD_ORB.h>
#include <bits/unordered_map.h>

#endif

using namespace ade::stm;
using namespace std;


struct IndexPoint {
  int idx;
};

POINT_CLOUD_REGISTER_POINT_STRUCT (IndexPoint,
                                   (int, idx, idx)
)


V4RTracker::V4RTracker(const long long &processorId, const int imgWidth, const int imgHeight)
        : ObjectTracker(processorId, imgWidth, imgHeight),
          trackers() {

  trackingConfidenceThreshold = 0.05f;
  visionProcessName = "V4RTracker";
  logger = log4cxx::Logger::getLogger("ade.tracker.V4RTracker");
}

V4RTracker::~V4RTracker() {
}


void V4RTracker::loadConfig(const std::string &config) {
  LOG4CXX_INFO(logger, boost::format("[loadConfig] method entered: %s") % config);

  // TODO use CameraParams to set intrinsic and distortion params
  //CameraParameters::ConstPtr camParams = Cameras::getInstance()->getCameraParameters(0);
  //cv::Mat intrinsic = cv::cvarrToMat(camParams->M); //cv::Mat_<double>::eye(3, 3);
  //cv::Mat D = (camParams->D);
  //cv::Mat dist_coeffs = D.reshape(1, 4); //cv::Mat::zeros(4, 1, CV_64F);
  //cv::Mat_<double> dist_coeffs;// = cv::Mat::zeros(4, 1, CV_64F);
  //dist_coeffs = cv::Mat_<double>();
  dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);
  intrinsic = cv::Mat_<double>::eye(3, 3);
  intrinsic(0, 0) = intrinsic(1, 1) = 525;
  intrinsic(0, 2) = 320, intrinsic(1, 2) = 240;


  // TODO: read parameters file
  param_.kt_param.plk_param.use_ncc = true;
  param_.kt_param.plk_param.ncc_residual = .5;

  // get directory
  unsigned found = config.find_last_of("/\\");
  std::string dir = config.substr(0, found + 1);

  // populate tree structure pt
  using boost::property_tree::ptree;
  ptree pt;
  read_xml(config, pt);

  std::string descriptorName;

  // parse tree
  modelDir_ = pt.get<std::string>("processor.modelsDir");

  BOOST_FOREACH(ptree::value_type const& predicateNode, pt.get_child("processor")) {
          if (predicateNode.first.compare("predicate") == 0) {

            std::string predicateName = predicateNode.second.get<std::string>("<xmlattr>.name");
            std::string v4rModelName = predicateNode.second.get<std::string>("v4rModelName");
            LOG4CXX_DEBUG(logger, boost::format("[loadConfig] predicateName: %s.") % predicateName);

            nameMap_.insert(std::unordered_map<std::string, std::string>::value_type(predicateName, v4rModelName));
          }
        }
}

void V4RTracker::haveNewImage(CaptureNotification::ConstPtr notification) {

  LOG4CXX_DEBUG(logger, "[haveNewImage] method entered.");
  const cv::Mat currFrame = notification->captureData->frame;

  //perform tracking iteration on all trackers
  Eigen::Matrix4f pose;
  double conf;
  bool status;

  boost::lock_guard<boost::mutex> lock(tracker_mutex);
  ObjectTrackerMonoPtrMap::iterator tracker_itr;
  LOG4CXX_DEBUG(logger, boost::format("%d trackers exist.") % trackers.size());
  for (tracker_itr = trackers.begin(); tracker_itr != trackers.end(); ++tracker_itr) {
    v4r::ObjectTrackerMono::Ptr &tracker = tracker_itr->second;
    status = tracker->track(currFrame, pose, conf);

    //update MemoryObject info
    LOG4CXX_DEBUG(logger, boost::format("tracked object #%lld.") % tracker_itr->first);
    MemoryObject::Ptr trackedMO = trackedObjects->getById(tracker_itr->first);
    LOG4CXX_DEBUG(logger, boost::format("tracking status: %s. confidence: %f.") % (status ? "true" : "false") % conf);

    // update tracked object with new tracking info
    if (status || conf > trackingConfidenceThreshold) {

      // create updated point cloud model
      pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::PointCloud<pcl::PointXYZ>::Ptr pose_aligned_model(new pcl::PointCloud<pcl::PointXYZ>());
      tracker->getModel().points.size();
      for (unsigned long i = 0; i < tracker->getModel().points.size(); ++i) {
        pcl::PointXYZ pt;
        v4r::GlobalPoint v4r_pt = tracker->getModel().points[i];
        pt.x = v4r_pt.pt.x();
        pt.y = v4r_pt.pt.y();
        pt.z = v4r_pt.pt.z();
        model->push_back(pt);
      }
      pcl::transformPointCloud(*model, *pose_aligned_model,
                               pose);

      MemoryObjectMask::Ptr newObjectMask(new MemoryObjectMask(notification->captureData, pose_aligned_model));
      trackedMO->addNewTrackingResult(newObjectMask, conf);
    } else {
      LOG4CXX_DEBUG(logger, "tracking unsuccessful, decreasing confidence.");
      //decreases confidence
      trackedMO->decayTrackingConfidence();
    }

    //send notifications that trackedMO was updated
    sendTrackingNotifications(trackedMO);
  }
}

void V4RTracker::startTracking(const MemoryObject::Ptr &newMemObj) {
  LOG4CXX_DEBUG(logger, "[startTracking] method entered.");
  boost::lock_guard<boost::mutex> lock(tracker_mutex);

  //explicitly call base class method
  ObjectTracker::startTracking(newMemObj);

  //instantiate new tracker
  LOG4CXX_DEBUG(logger, boost::format("Adding tracker for tokenID: %d.") % newMemObj->getId());
  v4r::ObjectTrackerMono::Ptr tracker(new v4r::ObjectTrackerMono(param_));

  //init new tracker
  // TODO: not sure why things crash if these values are only set in loadConfig
  dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);
  intrinsic = cv::Mat_<double>::eye(3, 3);
  intrinsic(0, 0) = intrinsic(1, 1) = 525;
  intrinsic(0, 2) = 320, intrinsic(1, 2) = 240;
  tracker->setObjectCameraParameter(intrinsic, dist_coeffs);
  tracker->setCameraParameter(intrinsic, dist_coeffs);

  // get model name from detected memory object
  std::string v4rModelName;
  PredicateHelper::Set::const_iterator descriptors_itr;
  std::unordered_map<std::string, std::string>::const_iterator nameMap_itr;
  PredicateHelper::Set descriptors = newMemObj->getValidationResults().getDescriptors();
  bool foundModelName = false;
  for (descriptors_itr = descriptors.begin(); descriptors_itr != descriptors.end(); ++descriptors_itr) {
    nameMap_itr = nameMap_.find(descriptors_itr->getName());
    if (nameMap_itr != nameMap_.end()) {
      foundModelName = true;
      v4rModelName = nameMap_itr->second;
      break;
    }
  }

  if (!foundModelName) {
    LOG4CXX_ERROR(logger, "Did not find a valid V4R model name. Returning without tracking.");
    return;
  }


  v4r::ArticulatedObject::Ptr model;
  //ade::V4RTrackingModelIO modelIO;
  //modelIO.readRecognitionStructure();
  if (v4r::io::read(modelDir_ + "/" + v4rModelName + "/" + v4rModelName + ".ao", model)) {
    tracker->setObjectModel(model);
  } else if (createTrackingModelFromRecognitionModel(modelDir_, v4rModelName, model)) {
    tracker->setObjectModel(model);
  } else {
    throw std::runtime_error("Tracking model file not found!");
  }

  //finally, add it to local tracker map
  trackers[newMemObj->getId()] = tracker;
}

void V4RTracker::stopTracking(const MemoryObject::Ptr &existingMemObj) {
  LOG4CXX_DEBUG(logger, "[stopTracking] method entered.");
  boost::lock_guard<boost::mutex> lock(tracker_mutex);

  //explicitly call base class method
  ObjectTracker::stopTracking(existingMemObj);

  long long id = existingMemObj->getId();
  ObjectTrackerMonoPtrMap::iterator tracker_itr = trackers.find(id);

  // check if MO is a top-level MO being tracked (or just a child that's not explicitly tracked)
  if (tracker_itr == trackers.end()) {
    LOG4CXX_DEBUG(logger, boost::format("Tracker does not exist for tokenID: %d.") % id);
    return;
  } else {
    LOG4CXX_DEBUG(logger, boost::format("Removing Tracker for tokenID: %d.") % id);
  }

  //clean up tracker
  trackers.erase(tracker_itr);
}

bool
V4RTracker::createTrackingModelFromRecognitionModel(const std::string &model_dir, const std::string &model_name,
                                                    v4r::ArticulatedObject::Ptr &model) {
  LOG4CXX_DEBUG(logger, "[createTrackingModelFromRecognitionModel] method entered.");

  readRecognitionStructure(model_dir + "/" + model_name, model, intrinsic, dist_coeffs);

  // TODO: make this configurable
  bool create_codebook = true;
  float thr_desc_rnn = 0.55;
  if (create_codebook) {
    v4r::CodebookMatcher cm = v4r::CodebookMatcher(v4r::CodebookMatcher::Parameter(thr_desc_rnn));

    for (unsigned i = 0; i < model->views.size(); i++) {
      cm.addView(model->views[i]->descs, i);
    }

    cm.createCodebook(model->cb_centers, model->cb_entries);
  }

  v4r::io::write(model_dir + "/" + model_name + "/" + model_name + ".ao", model);

  if (logger->isDebugEnabled()) {
    unsigned cnt = 0;
    for (unsigned i = 0; i < model->views.size(); i++) {
      cnt += model->views[i]->keys.size();

      LOG4CXX_DEBUG(logger, boost::format("Stored: %s, with %d keyframes and %d keypoints.")
                            % model_name % model->views.size() % cnt);
    }
  }

  return true;
}

void V4RTracker::readRecognitionStructure(const std::string &dir, v4r::ArticulatedObject::Ptr &model,
                                          const cv::Mat_<double> &intrinsic,
                                          const cv::Mat_<double> &dist_coeffs) {
  LOG4CXX_DEBUG(logger, "[readRecognitionStructure] method entered.");
  std::vector<std::string> pcd_files;
  std::string pose_file, indices_file, mask_file;
  std::string so_far = "";
  std::string pattern = ".*cloud_.*.pcd";
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
  pcl::PointCloud<IndexPoint>::Ptr object_indices(new pcl::PointCloud<IndexPoint>());
  cv::Mat_<unsigned char> mask;
  Eigen::Matrix4f pose, inv_pose;
  cv::Mat_<cv::Vec3b> im;
  cv::Mat_<unsigned char> im_gray;
  v4r::DataMatrix2D<Eigen::Vector3f>::Ptr kp_cloud(new v4r::DataMatrix2D<Eigen::Vector3f>());
  v4r::DataMatrix2D<Eigen::Vector3f>::Ptr kp_normals(new v4r::DataMatrix2D<Eigen::Vector3f>());

  // get files
  pcd_files = v4r::io::getFilesInDirectory(dir, pattern, true);
  //v4r::utils::getFilesInDirectory(dir, pcd_files, so_far, pattern, false);

  // int model
  model.reset(new v4r::ArticulatedObject());
  if (!intrinsic.empty())
    model->addCameraParameter(intrinsic, dist_coeffs);

  // init normal estimation and feature detection
  v4r::ZAdaptiveNormals::Parameter n_param;
  n_param.adaptive = true;
  v4r::ZAdaptiveNormals::Ptr nest(new v4r::ZAdaptiveNormals(n_param));

  //v4r::FeatureDetector_KD_FAST_IMGD::Parameter param(10000, 1.32, 4, 15);
  //v4r::FeatureDetector_KD_FAST_IMGD::Parameter param(10000, 1.44, 2, 17);
  v4r::FeatureDetector_KD_FAST_IMGD::Parameter param(10000, 1.2, 6, 13);

  param.do_feature_selection = true;
  keyDet.reset(new v4r::FeatureDetector_KD_FAST_IMGD(param));
  keyDesc = keyDet;

  // process frames
  for (unsigned i = 0; i < pcd_files.size(); i++) {
    pose_file = indices_file = mask_file = pcd_files[i];

    boost::replace_last(pose_file, ".pcd", ".txt");
    boost::replace_last(pose_file, "cloud_", "pose_");
    boost::replace_last(indices_file, ".pcd", ".txt");
    boost::replace_last(indices_file, "cloud_", "object_indices_");
    boost::replace_last(mask_file, ".pcd", ".png");
    boost::replace_last(mask_file, "cloud_", "mask_");

    if (pcl::io::loadPCDFile(dir + "/" + pcd_files[i], *cloud) == -1) continue;

    mask = cv::imread(dir + "/" + mask_file, CV_LOAD_IMAGE_GRAYSCALE);

    if (mask.empty()) {
      if (!loadObjectIndices(dir + "/" + indices_file, mask, cv::Size(cloud->width, cloud->height)))
        continue;
    }

    if (v4r::readPose(dir + "/" + pose_file, inv_pose)) {
      v4r::convertImage(*cloud, im);

      if (im.type() != CV_8U) cv::cvtColor(im, im_gray, CV_RGB2GRAY);
      else im_gray = im;

      v4r::convertCloud(*cloud, *kp_cloud);
      nest->compute(*kp_cloud, *kp_normals);

      v4r::invPose(inv_pose, pose);

      if (addObjectView(*kp_cloud, *kp_normals, im_gray, mask, pose, *model))
        cout << ".. added keyframe " << pcd_files[i] << " with " << model->views.back()->keys.size() << " keypoints!"
             << endl;
    }
  }
}

bool V4RTracker::addObjectView(const v4r::DataMatrix2D <Eigen::Vector3f> &cloud,
                               const v4r::DataMatrix2D <Eigen::Vector3f> &normals,
                               const cv::Mat_<unsigned char> &im, const cv::Mat_<unsigned char> &mask,
                               const Eigen::Matrix4f &pose, v4r::ArticulatedObject &model) {
  LOG4CXX_DEBUG(logger, "[addObjectView] method entered.");
  // get and transform 3d points
  Eigen::Matrix4f inv_pose;
  Eigen::Vector3f pt_model, n_model, vr_model;
  static const unsigned MIN_KEYPOINTS = 20;

  v4r::invPose(pose, inv_pose);

  Eigen::Matrix3f R = inv_pose.topLeftCorner<3, 3>();
  Eigen::Vector3f t = inv_pose.block<3, 1>(0, 3);

  std::vector<cv::KeyPoint> keys;
  cv::Mat descs;
  unsigned cnt = 0;

  // detect keypoints
  keyDet->detect(im, keys);
  keyDesc->extract(im, keys, descs);

  for (unsigned i = 0; i < keys.size(); i++) {
    cv::KeyPoint &key = keys[i];

    int idx = int(key.pt.y + .5) * cloud.cols + int(key.pt.x + .5);

    const Eigen::Vector3f &pt = cloud[idx];
    const Eigen::Vector3f &n = normals[idx];

    if (!isnan(pt[0]) && !isnan(n[0]) && mask(key.pt.y, key.pt.x) > 128)
      cnt++;
  }


  if (cnt < MIN_KEYPOINTS) return false;

  v4r::ObjectView &view = model.addObjectView(pose, im);


  for (unsigned i = 0; i < keys.size(); i++) {
    cv::KeyPoint &key = keys[i];

    int idx = int(key.pt.y + .5) * cloud.cols + int(key.pt.x + .5);

    const Eigen::Vector3f &pt = cloud[idx];
    const Eigen::Vector3f &n = normals[idx];

    if (!isnan(pt[0]) && !isnan(n[0]) && mask(key.pt.y, key.pt.x) > 128) {
      pt_model = R * pt + t;
      n_model = (R * n).normalized();
      vr_model = -(R * pt).normalized();

      view.add(keys[i], &descs.at<float>(i, 0), descs.cols, pt_model, n_model, vr_model);
    }
  }

  view.descs.copyTo(descs);
  view.descs = descs;
  return true;
}

bool
V4RTracker::loadObjectIndices(const std::string &_filename, cv::Mat_<unsigned char> &_mask, const cv::Size &_size) {
  int idx;
  std::vector<int> indices;

  std::ifstream mi_f(_filename);
  if (mi_f.is_open()) {
    while (mi_f >> idx)
      indices.push_back(idx);
    mi_f.close();

    _mask = cv::Mat_<unsigned char>::zeros(_size);
    int size = _mask.rows * _mask.cols;

    for (unsigned i = 0; i < indices.size(); i++) {
      if (indices[i] < size)
        _mask(indices[i] / _mask.cols, indices[i] % _mask.cols) = 255;
    }
    return true;
  }

  return false;
}
