/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * Class for cluster classification based on global features.
 *
 * @author Evan Krause
 * @date July 2015
 */

#ifndef GLOBALFEATUREVALIDATOR_HPP
#define GLOBALFEATUREVALIDATOR_HPP

#include "ObjectValidator.hpp"
#include "../global_features/GlobalFeaturesModel.hpp"

#include <boost/thread/mutex.hpp>
#include <pcl/recognition/hv/hv_go.h>
#include <pcl/visualization/histogram_visualizer.h>

class GlobalFeatureValidator : public ObjectValidator {
  typedef pcl::PointXYZ PointType;
  typedef pcl::Normal NormalType;
  typedef pcl::ReferenceFrame RFType;
  typedef pcl::VFHSignature308 DescriptorType;
  typedef GlobalFeaturesModel<PointType, NormalType, DescriptorType> ModelType;

public:
  typedef boost::shared_ptr<GlobalFeatureValidator> Ptr;
  typedef boost::shared_ptr<const GlobalFeatureValidator> ConstPtr;

  GlobalFeatureValidator(const long long& processorId, const unsigned int imgWidth, const unsigned int imgHeight);
  ~GlobalFeatureValidator();

  virtual void loadConfig(const std::string& config);
  virtual void saveConfig(const std::string& config);

  virtual bool hasLearned() const;
  virtual void resetHasLearned();

protected:
  virtual void handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification);
  virtual void handleLearningNotification(LearningNotification::ConstPtr notification);

private:
  std::vector<ModelType> classifyPointCloud(const pcl::PointCloud<PointType>::ConstPtr& scene_cloud,
                                            const pcl::PointCloud<PointType>::ConstPtr& object_cloud);
  void displayResults(const pcl::PointCloud<PointType>::ConstPtr &scene_cloud,
                      const std::vector<pcl::PointCloud<PointType>::ConstPtr> &model_clouds_aligned,
                      const std::vector<ModelType> &matching_models,
                      const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &transforms);


  bool show_matches_;
  int feature_type_;
  boost::mutex models_mutex_;
  std::vector<ModelType> models_;
  pcl::PointCloud<DescriptorType>::Ptr model_descriptors_;
  std::map<int, int> descriptor_to_model_;
  std::map<std::string, std::vector<int> > type_to_model_;
  pcl::KdTreeFLANN<DescriptorType> match_search_;
  pcl::GlobalHypothesesVerification<PointType, PointType> ghv_;

  // config file -- set during load config file
  std::string configFile;

  // has learned flag
  mutable boost::recursive_mutex has_learned_mutex_;
  bool has_learned_flag_;

  ////////////////////////////////////////////////////////////////////////////////
  /////////////// TODO: put everything below this in separate util file //////////
  ////////////////////////////////////////////////////////////////////////////////

  bool createModel(const std::string& model_file, bool use_color, ModelType& model);
  // bool createModel(const std::string& model_file, const std::string& image_file, bool use_color, ModelType& model);

  bool populateModel(pcl::PointCloud<PointType>::ConstPtr cloud, ModelType& model);

  void printMatrix4f(const Eigen::Matrix4f& matrix, const std::string& matrix_name);

  void transRotToPose(const Eigen::Vector4f& translation, const Eigen::Quaternionf& orientation, Eigen::Matrix4f& pose);

  float computeMeshResolution(pcl::PointCloud<PointType>::ConstPtr input);

  void makeUniformResolution(pcl::PointCloud<PointType>::ConstPtr cloud,
          pcl::PointCloud<PointType>::Ptr& cloud_resampled);

  void computeVFHDescriptors(pcl::PointCloud<PointType>::ConstPtr cloud,
          pcl::PointCloud<NormalType>::ConstPtr normals,
          pcl::PointCloud<DescriptorType>::Ptr& descriptors);

  //void computeOURCVFHDescriptors(pcl::PointCloud<PointType>::ConstPtr cloud,
  //        pcl::PointCloud<NormalType>::ConstPtr normals,
  //        pcl::PointCloud<DescriptorType>::Ptr descriptors);

  void computeCRH(pcl::PointCloud<PointType>::ConstPtr cloud,
          pcl::PointCloud<NormalType>::ConstPtr normals,
          Eigen::Vector4f& centroid,
          pcl::PointCloud<pcl::Histogram<90> >::Ptr& histogram);

  bool estimatePose(pcl::PointCloud<PointType>::Ptr model,
          pcl::PointCloud<PointType>::Ptr scene,
          const Eigen::Vector4f& model_centroid,
          const Eigen::Vector4f& scene_centroid,
          pcl::PointCloud<pcl::Histogram<90> >::Ptr model_crh,
          pcl::PointCloud<pcl::Histogram<90> >::Ptr scene_crh,
          Eigen::Matrix4f& pose);

  bool populateModel(pcl::PointCloud<PointType>::ConstPtr cloud,pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudRGB, bool use_color, ModelType &model);
  // std::vector<ModelType> filterByColor(std::vector<ModelType> matching_models, cv::Mat objectImage);
  std::vector<ModelType> filterByColor(std::vector<ModelType> matching_models, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &object_cloudRGB);
  // cv::Mat computeImageHist(cv::Mat object_image);
  cv::Mat computeCloudHist(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloudRGB, const int h_bins, const int s_bins);
};

#endif  //GLOBALFEATUREVALIDATOR_HPP
