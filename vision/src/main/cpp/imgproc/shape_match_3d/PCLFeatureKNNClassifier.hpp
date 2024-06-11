/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   PCLFeatureKNNClassifier.hpp
 * Author: evan
 *
 * Created on December 17, 2013, 3:37 PM
 */

#ifndef PCLFEATUREKNNCLASSIFIER_HPP
#define	PCLFEATUREKNNCLASSIFIER_HPP

#include <limits>
#include <vector>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "Eigen/Core"
#include "flann/flann.h"
#include <boost/unordered_map.hpp>
#include <log4cxx/logger.h>

template<typename FeatureT>
class PCLFeatureKNNClassifier {
public:

  /**
   * Struct to store feature/label pair.
   */
  struct PCLFeature {
    FeatureT feature;
    std::string type;
  };

  /**
   * A struct for storing classification results
   */
  struct Results {

    Results(int num_features, int k)
    : num_features_(num_features), k_(k) {
      k_indices_ = flann::Matrix<int>(new int[num_features_ * k], num_features_, k);
      k_distances_ = flann::Matrix<float>(new float[num_features_ * k], num_features_, k);
      std::vector<std::string> tempStrVec;
      tempStrVec.resize(k);
      k_types_.resize(num_features, tempStrVec);
      //k_types_ = flann::Matrix<std::string>(new std::string[num_features_ * k], num_features_, k);
      logger = log4cxx::Logger::getLogger("diarc.imgproc.shape_match_3d.PCLFeatureKNNClassifier.Results");
    }

    ~Results() {
      delete[] k_indices_.ptr();
      delete[] k_distances_.ptr();
    }

    int num_features_;
    int k_;
    flann::Matrix<int> k_indices_;
    flann::Matrix<float> k_distances_;
    std::vector<std::vector<std::string> > k_types_;
    log4cxx::LoggerPtr logger;

    void print() {
      LOG4CXX_INFO(logger, "Classification Results:");
      boost::unordered_map<std::string, float> dist;
      boost::unordered_map<std::string, FILE*> file; //DEBUG ONLY
      boost::unordered_map<std::string, int> count;
      boost::unordered_map<std::string, float>::iterator dist_itr;
      for (int i = 0; i < num_features_; ++i) {
        
        dist_itr = dist.find(k_types_[i][0]);
        if (dist_itr == dist.end()) {
          dist[k_types_[i][0]] = k_distances_[i][0];
          count[k_types_[i][0]] = 1;

          std::string filename = k_types_[i][0] + ".txt";
          FILE* f = fopen(filename.c_str(), "wt");
          fprintf(f, "%f, ", k_distances_[i][0]);
          file[k_types_[i][0]] = f;
        } else {
          dist[k_types_[i][0]] += k_distances_[i][0];
          count[k_types_[i][0]] += 1;
          
          fprintf(file[k_types_[i][0]], "%f, ", k_distances_[i][0]);
        }

        //        for (int j = 0; j < k_; ++j) {
        //          LOG4CXX_INFO(logger, boost::format("%d - %d - %s index %d with a distance of: %f.")
        //                  % i % j % k_types_[i][j] % k_indices_[i][j] % k_distances_[i][j]);
        //        }
      }
      //print summary
      for (dist_itr = dist.begin(); dist_itr != dist.end(); ++dist_itr) {
        int index_count = count[dist_itr->first];
        float avg_dist = dist_itr->second / static_cast<float> (index_count);
        LOG4CXX_INFO(logger, boost::format("type: %s. count: %d. avg dist: %f.") % dist_itr->first % index_count % avg_dist);
        
        FILE* f = file[dist_itr->first];
        fprintf(f, "\n");
        fclose(f);
      }
    }
  };

  PCLFeatureKNNClassifier(const int featureLength)
  : feature_length(featureLength) {
    logger = log4cxx::Logger::getLogger("diarc.imgproc.shape_match_3d.PCLFeatureKNNClassifier");
  }

  ~PCLFeatureKNNClassifier() {
  }

  /**
   * Add new feature.
   * @param feature new feature 
   * @param feature_type type label of feature
   */
  void addFeature(FeatureT feature, const std::string& feature_type) {
    PCLFeature newFeature;
    newFeature.feature = feature;
    newFeature.type = feature_type;
    features_.push_back(newFeature);
  }

  /**
   * Add new features.
   * @param features vector of new feature s
   * @param feature_type type label of features
   */
  void addFeatures(const std::vector<FeatureT, Eigen::aligned_allocator<FeatureT> >& features, const std::string& feature_type) {
    typename std::vector<FeatureT, Eigen::aligned_allocator<FeatureT> >::const_iterator feature_itr;
    for (feature_itr = features.begin(); feature_itr != features.end(); ++feature_itr) {
      addFeature(*feature_itr, feature_type);
    }
  }

  /** 
   *  Build flann classifier from all added features.
   */
  void buildClassifier() {
    // Convert data into FLANN format
    //TODO: delete "data"
    flann::Matrix<float> data = flann::Matrix<float>(new float[features_.size() * feature_length], features_.size(), feature_length);

    for (size_t i = 0; i < data.rows; ++i)
      for (size_t j = 0; j < data.cols; ++j)
        data[i][j] = features_[i].feature.descriptor[j];


    //Finally, we create the flann index
    LOG4CXX_INFO(logger, "[buildClassifier] creating flann index.");
    flan_index = FLANNIndexPtr(new FLANNIndex(data, flann::LinearIndexParams()));
    //flan_index = FLANNIndexPtr(new FLANNIndex(data, flann::KMeansIndexParams()));
    //flan_index = FLANNIndexPtr(new FLANNIndex(data, flann::KDTreeIndexParams (4)));

    LOG4CXX_INFO(logger, "[buildClassifier] building classifier.");
    flan_index->buildIndex();
    //flan_index->save(kdtree_idx_file_name);
  }

  /** 
   * Perform k-nn search on new features
   * @param features_to_classify
   * @param k
   * @return classification results
   */
  Results nearestKSearch(const std::vector<FeatureT, Eigen::aligned_allocator<FeatureT> >& features_to_classify, const int k) {
    Results results(features_to_classify.size(), k);

    // fill flann matrix with features
    LOG4CXX_INFO(logger, "[nearestKSearch] filling flann matrix");
    flann::Matrix<float> p = flann::Matrix<float>(new float[features_to_classify.size() * feature_length], features_to_classify.size(), feature_length);
    for (size_t i = 0; i < features_to_classify.size(); ++i) {
      memcpy(p[i], &features_to_classify[i].descriptor[0], p.cols * sizeof (float));
    }

    LOG4CXX_INFO(logger, "[nearestKSearch] performing knnSearch...");
    flan_index->knnSearch(p, results.k_indices_, results.k_distances_, results.k_, flann::SearchParams(512));
    delete[] p.ptr();
    LOG4CXX_INFO(logger, "[nearestKSearch] done.");

    //DEBUG ONLY

    //    for (size_t i = 0; i < features_to_classify.size(); ++i) {
    //      if (std::isnan(results.k_distances_[i][0])) {
    //        
    //        //check individual elements for nan
    //        int match_idx = results.k_indices_[i][0];
    //        int nanCount_toClassify = 0;
    //        int nanCount_target = 0;
    //        for (int j = 0; j < feature_length; ++j) {
    //          if (std::isnan(features_to_classify[i].descriptor[j])) {
    //            ++nanCount_toClassify;
    //            //LOG4CXX_INFO(logger, boost::format("[nearestKSearch] %d feature to classify nan at %d.") % i % j);
    //          }
    //          if (std::isnan(features_[match_idx].feature.descriptor[j])) {
    //            ++nanCount_target;
    //            //LOG4CXX_INFO(logger, boost::format("[nearestKSearch] %d target feature nan at %d.") % i % j);
    //          }
    //        }
    //        
    //        LOG4CXX_INFO(logger, boost::format("[nearestKSearch] nan counts toClassify: %d index: %d. target: %d index: %d.")
    //                % nanCount_toClassify % i % nanCount_target % match_idx);
    //        
    //        //L2 norm
    //      }
    //    }
    //DEBUG ONLY

    //set k types
    LOG4CXX_INFO(logger, "[nearestKSearch] setting type labels in results.");
    for (size_t i = 0; i < features_to_classify.size(); ++i) {
      for (size_t j = 0; j < results.k_; ++j) {
        results.k_types_[i][j] = features_[results.k_indices_[i][j]].type;
      }
    }

    return results;
  }

  /**
   * Get number of features that have been added.
   * @return 
   */
  int getNumberOfFeatures() const {
    return features_.size();
  }

private:
  typedef flann::Index< flann::ChiSquareDistance<float> > FLANNIndex;
  typedef boost::shared_ptr<FLANNIndex> FLANNIndexPtr;

  // number of histogram bins in FeatureT
  int feature_length;

  // A list of template clouds and the target to which they will be classified against
  std::vector<PCLFeature> features_;

  // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
  FLANNIndexPtr flan_index;

  log4cxx::LoggerPtr logger;
};

#endif	/* PCLFEATUREKNNCLASSIFIER_HPP */

