/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef VFH_CLASSIFIER_HPP
#define VFH_CLASSIFIER_HPP

#include "feature_cloud.hpp"

#include <limits>
#include <vector>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "Eigen/Core"
#include "flann/flann.h"

class VFHClassifier {
public:

    // A struct for storing classification results

    struct Result {

        Result(int k_)
        : k(k_) {
            k_indices = flann::Matrix<int>(new int[k], 1, k);
            k_distances = flann::Matrix<float>(new float[k], 1, k);
            k_types.resize(k);
        }

        ~Result() {
            delete[] k_indices.ptr();
            delete[] k_distances.ptr();
        }
        flann::Matrix<int> k_indices;
        flann::Matrix<float> k_distances;
        std::vector<std::string> k_types;
        int k;

        void print() {
            printf("VFH Classification Results: \n");
            for (int i = 0; i < k; ++i)
                printf("    %d - %s index %d with a distance of: %f\n", i, k_types[i].c_str(), k_indices[0][i], k_distances[0][i]);

        }
    };

    VFHClassifier() {
    }

    ~VFHClassifier() {
    }

    // Add the given cloud to the list of template clouds
    void addTemplateCloud(FeatureCloud<pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308>, pcl::VFHSignature308> &template_cloud) {
        templates_.push_back(template_cloud);
    }

    //build the flann classifier from all added templates
    void buildClassifier() {
        // Convert data into FLANN format (308 is number of histogram bins)
        //TODO: delete "data"
        flann::Matrix<float> data = flann::Matrix<float>(new float[templates_.size()*308], templates_.size(), 308);

        for (size_t i = 0; i < data.rows; ++i)
            for (size_t j = 0; j < data.cols; ++j)
                data[i][j] = templates_[i].getLocalFeatures()->points[0].histogram[j];


        //Finally, we create the KdTree
        flan_index = FLANNIndexPtr(new FLANNIndex(data, flann::LinearIndexParams()));
        //flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));
        flan_index->buildIndex();
        //flan_index->save(kdtree_idx_file_name);
    }

    void nearestKSearch(FeatureCloud<pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308>, pcl::VFHSignature308> &target_cloud, VFHClassifier::Result &result) {
        // Query point
        flann::Matrix<float> p = flann::Matrix<float>(new float[308], 1, 308);
        memcpy(p.ptr(), &target_cloud.getLocalFeatures()->points[0].histogram[0], p.cols * p.rows * sizeof (float));

        flan_index->knnSearch(p, result.k_indices, result.k_distances, result.k, flann::SearchParams(512));
        delete[] p.ptr();

        //set k types
        for (size_t i = 0; i < result.k; ++i) {
            result.k_types[i] = templates_[result.k_indices[0][i]].getType();
        }
    }

    int getNumberOfTemplates() {
        return templates_.size();
    }

private:
    typedef flann::Index< flann::ChiSquareDistance<float> > FLANNIndex;
    typedef boost::shared_ptr<FLANNIndex> FLANNIndexPtr;

    // A list of template clouds and the target to which they will be classified against
    std::vector<FeatureCloud<pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308>, pcl::VFHSignature308> > templates_;
    FeatureCloud<pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308>, pcl::VFHSignature308> target_;

    // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
    FLANNIndexPtr flan_index;
};

#endif // #ifndef VFH_CLASSIFIER_HPP
