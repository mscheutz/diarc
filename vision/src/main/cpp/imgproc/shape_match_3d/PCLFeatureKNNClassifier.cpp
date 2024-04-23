/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "PCLFeatureKNNClassifier.hpp"

PCLFeatureKNNClassifier::PCLFeatureKNNClassifier() {
}

PCLFeatureKNNClassifier::~PCLFeatureKNNClassifier() {
}

void PCLFeatureKNNClassifier::addFeature(FeatureT feature, const std::string& feature_type) {
  PCLFeature newFeature;
  newFeature.feature = feature;
  newFeature.type = feature_type;
  features_.push_back(newFeature);
}

void PCLFeatureKNNClassifier::addFeatures(std::vector<FeatureT> features, const std::string& feature_type) {
  std::vector<FeatureT>::const_iterator feature_itr;
  for (feature_itr = features.begin(); feature_itr != features.end(); ++feature_itr) {
    addFeature(*feature_itr, feature_type);
  }
}

void PCLFeatureKNNClassifier::buildClassifier() {
  // Convert data into FLANN format
  //TODO: delete "data"
  flann::Matrix<float> data = flann::Matrix<float>(new float[features_.size() * feature_length], features_.size(), feature_length);

  for (size_t i = 0; i < data.rows; ++i)
    for (size_t j = 0; j < data.cols; ++j)
      data[i][j] = features_[i].feature.histogram[j];


  //Finally, we create the KdTree
  flan_index = FLANNIndexPtr(new FLANNIndex(data, flann::LinearIndexParams()));
  //flan_index = FLANNIndexPtr(new FLANNIndex(data, flann::KDTreeIndexParams (4)));
  flan_index->buildIndex();
  //flan_index->save(kdtree_idx_file_name);
}

Results PCLFeatureKNNClassifier::nearestKSearch(pcl::PointCloud<FeatureT> &features_to_classify, const int k) {
  Results results(features_to_classify.size(), k);

  // Query point
  flann::Matrix<float> p = flann::Matrix<float>(new float[features_to_classify.size() * feature_length], features_to_classify.size(), feature_length);
  memcpy(p.ptr(), &target_cloud.getLocalFeatures()->points[0].histogram[0], p.cols * p.rows * sizeof (float));

  flan_index->knnSearch(p, results.k_indices_, results.k_distances_, results.k_, flann::SearchParams(512));
  delete[] p.ptr();

  //set k types
  for (size_t i = 0; i < result.k; ++i) {
    results.k_types[i] = features_[results.k_indices[0][i]].type;
  }

  return results;
}

int PCLFeatureKNNClassifier::getNumberOfFeatures() const {
  return features_.size();
}

