/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * Class for cluster classification based on V4R library: https://rgit.acin.tuwien.ac.at/v4r/v4r.
 *
 * @author Evan Krause
 * @date November 2017
 */

#ifndef V4RDETECTOR_HPP
#define V4RDETECTOR_HPP

#include "ObjectDetector.hpp"

#include <boost/thread/mutex.hpp>
#include <boost/filesystem.hpp>

// TODO: keep only needed v4r headers
#include <boost/serialization/vector.hpp>
#include <v4r/apps/CloudSegmenter.h>
#include <v4r/apps/ObjectRecognizerParameter.h>
#include <v4r/apps/visualization.h>
#include <v4r/common/normals.h>
#include <v4r/core/macros.h>
#include <v4r/io/filesystem.h>
#include <v4r/recognition/local_recognition_pipeline.h>
#include <v4r/recognition/multi_pipeline_recognizer.h>
#include <v4r/recognition/hypotheses_verification.h>

class V4RDetector : public ObjectDetector {

public:
  typedef boost::shared_ptr<V4RDetector> Ptr;
  typedef boost::shared_ptr<const V4RDetector> ConstPtr;
  //typedef pcl::PointXYZ PointT;
  typedef pcl::PointXYZRGB PointT;

  V4RDetector(const long long &processorId, const unsigned int imgWidth, const unsigned int imgHeight);

  ~V4RDetector();
  virtual void loadConfig(const std::string &config);
  //virtual void saveConfig(const std::string& config);
  virtual void init();

protected:
  virtual void handleCaptureNotification(CaptureNotification::ConstPtr notification);
  //virtual void handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification);
  //virtual void handleLearningNotification(LearningNotification::ConstPtr notification);

private:

  std::vector<std::string> v4rArgs_;
  std::string v4rConfigDir_;
  std::unordered_map<std::string, std::string> nameMap_;

  typename v4r::RecognitionPipeline<PointT>::Ptr mrec_; ///< multi-pipeline recognizer
  typename v4r::LocalRecognitionPipeline<PointT>::Ptr local_recognition_pipeline_; ///< local recognition pipeline (member variable just because of visualization of keypoints)
  typename v4r::HypothesisVerification<PointT, PointT>::Ptr hv_; ///< hypothesis verification object
  typename v4r::NormalEstimator<PointT>::Ptr normal_estimator_;    ///< normal estimator used for computing surface normals (currently only used at training)

  typename v4r::ObjectRecognitionVisualizer<PointT>::Ptr rec_vis_; ///< visualization object

  typename v4r::apps::CloudSegmenter<PointT>::Ptr cloud_segmenter_; ///< cloud segmenter for plane removal (if enabled)

  bool visualize_; ///< if true, visualizes objects
  bool skip_verification_; ///< if true, will only generate hypotheses but not verify them
  std::string models_dir_;

  v4r::apps::ObjectRecognizerParameter param_;

  v4r::Camera::ConstPtr camera_;

  typename v4r::Source<PointT>::Ptr model_database_;

  typename pcl::PointCloud<PointT>::Ptr registered_scene_cloud_;  ///< registered point cloud of all processed input clouds in common camera reference frame

  std::vector<std::pair<std::string, float> > elapsed_time_; ///< measurements of computation times for various components

  // MULTI-VIEW STUFF
  class View {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typename pcl::PointCloud<PointT>::ConstPtr cloud_;
    typename pcl::PointCloud<PointT>::Ptr processed_cloud_;
    typename pcl::PointCloud<PointT>::Ptr removed_points_;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_;
    std::vector<std::vector<float> > pt_properties_;
    Eigen::Matrix4f camera_pose_;
  };

  std::vector<View> views_; ///< all views in sequence

  // config file -- set during load config file
  std::string configFile_;

  /**
   * @brief initialize initialize Object recognizer (sets up model database, recognition pipeline and hypotheses verification)
   * @param arguments
   */
  void initialize(std::vector<std::string> &command_line_arguments, const boost::filesystem::path &config_folder = bf::path("cfg"));

  /**
   * @brief detectChanges detect changes in multi-view sequence (e.g. objects removed or added to the scene within observation period)
   * @param v current view
   */
  void detectChanges(View &v);

  /**
   * @brief recognize recognize objects in point cloud
   * @param cloud (organized) point cloud
   * @return
   */
  std::vector<v4r::ObjectHypothesesGroup > recognize(const typename pcl::PointCloud<PointT>::ConstPtr &cloud);


  void displayResults(const ade::stm::MemoryObject::VecPtr &mos);

};

#endif  //V4RDETECTOR_HPP
