/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef TEMPLATE_ALIGNMENT_HPP_
#define TEMPLATE_ALIGNMENT_HPP_

#include "feature_cloud.hpp"

#include <limits>
#include <vector>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/registration/ia_ransac.h"
#include "Eigen/Core"

class TemplateAlignment
{
public:

  // A struct for storing alignment results
  struct Result
  {
    float fitness_score;
    //Eigen::Matrix4f final_transformation;
  public:
      //  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  TemplateAlignment () :
    min_sample_distance_ (0.05),
    max_correspondence_distance_ (0.01*0.01),
    nr_iterations_ (500)
  {
    // Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm
    sac_ia_.setMinSampleDistance (min_sample_distance_);
    sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
    sac_ia_.setMaximumIterations (nr_iterations_);
  }

  ~TemplateAlignment () {}

  // Set the given cloud as the target to which the templates will be aligned
  void setTargetCloud (FeatureCloud<pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>, pcl::FPFHSignature33> &target_cloud)
  {
    target_ = target_cloud;
    sac_ia_.setInputTarget (target_cloud.getPointCloud ());
    sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());
  }

  // Add the given cloud to the list of template clouds
  void addTemplateCloud (FeatureCloud<pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>, pcl::FPFHSignature33> &template_cloud)
  {
    templates_.push_back (template_cloud);
  }

  // Align the given template cloud to the target specified by setTargetCloud ()
  void align (FeatureCloud<pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>, pcl::FPFHSignature33> &template_cloud, TemplateAlignment::Result &result)
  {
    sac_ia_.setInputSource (template_cloud.getPointCloud ());
    sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());

    pcl::PointCloud<pcl::PointXYZ> registration_output;
    sac_ia_.align (registration_output);

    result.fitness_score = sac_ia_.getFitnessScore (max_correspondence_distance_);
    //result.final_transformation = sac_ia_.getFinalTransformation ();
  }

  // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
  void alignAll (std::vector<TemplateAlignment::Result> &results)
  {
    results.resize (templates_.size ());
    for (size_t i = 0; i < templates_.size (); ++i)
    {
      align (templates_[i], results[i]);
    }
  }

  // Align all of template clouds to the target cloud to find the one with best alignment score
  int findBestAlignment (TemplateAlignment::Result &result)
  {
    // Align all of the templates to the target cloud
    std::vector<Result> results;
    alignAll (results);

    // Find the template with the best (lowest) fitness score
    float lowest_score = std::numeric_limits<float>::infinity ();
    int best_template = 0;
    for (size_t i = 0; i < results.size (); ++i)
    {
      const Result &r = results[i];
      if (r.fitness_score < lowest_score)
      {
        lowest_score = r.fitness_score;
        best_template = i;
      }
    }

    // Output the best alignment
    result = results[best_template];
    return (best_template);
  }

private:
  // A list of template clouds and the target to which they will be aligned
  std::vector<FeatureCloud<pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>, pcl::FPFHSignature33> > templates_;
  FeatureCloud<pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>, pcl::FPFHSignature33> target_;

  // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
  pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
  float min_sample_distance_;
  float max_correspondence_distance_;
  float nr_iterations_;
};

#endif // #ifndef TEMPLATE_ALIGNMENT_HPP_
