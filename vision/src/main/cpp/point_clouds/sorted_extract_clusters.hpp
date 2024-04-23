/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef PCL_SORTED_EXTRACT_CLUSTERS_H_
#define PCL_SORTED_EXTRACT_CLUSTERS_H_

#include <pcl/pcl_base.h>
#include <pcl/kdtree/kdtree.h>

#include "detector/ClusterDetector.hpp"

template <typename PointT> void 
extractSortedEuclideanClusters(//ClusterDetector::Ptr processor,
                               const typename pcl::PointCloud<PointT>::ConstPtr cloud,
                               const std::vector<IndexedValueType> &indices,
#if PCL_VERSION_COMPARE(>=, 1, 12, 0)
                               const std::shared_ptr<pcl::search::Search<PointT> > &tree,
#else
                               const boost::shared_ptr<pcl::search::Search<PointT> > &tree,
#endif
                               std::vector<pcl::PointIndices> &clusters,
                               float tolerance, unsigned int min_pts_per_cluster,
                               unsigned int max_pts_per_cluster,
                               bool isIncremental = false)
{
  clusters.clear();
  
  if (tree->getInputCloud()->points.size() != cloud->points.size()) 
  {
    PCL_ERROR("[pcl::extractSortedEuclideanClusters] Tree built for a different point cloud dataset (%lu) than the input cloud (%lu)!\n", 
              (unsigned long) tree->getInputCloud()->points.size(), (unsigned long) cloud->points.size());
    return;
  }
  if (tree->getIndices()->size() != indices.size ())
  {
    PCL_ERROR ("[pcl::extractEuclideanClusters] Tree built for a different set of indices (%lu) than the input set (%lu)!\n", 
               (unsigned long)tree->getIndices()->size(), (unsigned long)indices.size ());
    return;
  }
  
  // Create a bool vector of processed point indices, and initialize it to false
  std::vector<bool> processed (cloud->points.size(), false);
  
  std::vector<int> nn_indices;
  std::vector<float> nn_distances;
  // Process all points in the indices vector
  for (size_t i = 0; i < indices.size (); ++i)
  {
    if(isIncremental && (indices.at(i).value <= 0))
    {
      break;
    }
    
    if (processed.at(indices.at(i).index)) {
      //i++;
      continue;
    }
            
    std::vector<int> seed_queue;
    int sq_idx = 0;
    seed_queue.push_back(indices.at(i).index);

    processed.at(indices.at(i).index) = true;

    while (sq_idx < (int) seed_queue.size()) {
      // Search for sq_idx
      int result = tree->radiusSearch(*cloud,seed_queue.at(sq_idx), tolerance, nn_indices, nn_distances); 
      if(result == -1) 
      {
        PCL_ERROR("[pcl::extractEuclideanClusters] Received error code -1 from radiusSearch\n");
        return;
      }
      if(!result)
      {
        sq_idx++;
        continue;
      }
                
      for (size_t j = 1; j < nn_indices.size(); ++j) // nn_indices[0] should be sq_idx
      {
        if (processed.at(nn_indices.at(j))) // Has this point been processed before ?
          continue;

        // Perform a simple Euclidean clustering
        seed_queue.push_back(nn_indices.at(j));
        processed.at(nn_indices.at(j)) = true;
      }

      sq_idx++;
    }
    
    // If this queue is satisfactory, add to the clusters
    if (seed_queue.size() >= min_pts_per_cluster && seed_queue.size() <= max_pts_per_cluster) 
    {
      pcl::PointIndices r;
      r.indices.resize(seed_queue.size());
      for (size_t j = 0; j < seed_queue.size(); ++j)
      {
        r.indices.at(j) = seed_queue.at(j);
      }

      sort(r.indices.begin(), r.indices.end());
      r.indices.erase(std::unique(r.indices.begin(), r.indices.end()), r.indices.end());

      r.header = cloud->header;
      //if (processor->checkClusterValidity(r))
      //{
        //processor->sendClusterAway(r);
	clusters.push_back(r); // We could avoid a copy by working directly in the vector
        //return;
      //}
    }
  }
}


template <typename PointT>
class SortedEuclideanClusterExtraction : public pcl::PCLBase<PointT> {
  typedef pcl::PCLBase<PointT> BasePCLBase;

  public:
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    typedef typename pcl::search::Search<PointT> KdTree;
    typedef typename pcl::search::Search<PointT>::Ptr KdTreePtr;

    typedef pcl::PointIndices::Ptr PointIndicesPtr;
    typedef pcl::PointIndices::ConstPtr PointIndicesConstPtr;

    SortedEuclideanClusterExtraction() : 
      tree_(), min_pts_per_cluster_(1), max_pts_per_cluster_(std::numeric_limits<int>::max()) {
    };

    inline void setSearchMethod(const KdTreePtr &tree) {
      tree_ = tree;
    }

    inline KdTreePtr getSearchMethod() {
      return (tree_);
    }

    inline void setClusterTolerance(double tolerance) {
      cluster_tolerance_ = tolerance;
    }

    inline double getClusterTolerance() {
      return (cluster_tolerance_);
    }

    inline void setMinClusterSize(int min_cluster_size) {
      min_pts_per_cluster_ = min_cluster_size;
    }

    inline int getMinClusterSize() {
      return (min_pts_per_cluster_);
    }

    inline void setMaxClusterSize(int max_cluster_size) {
      max_pts_per_cluster_ = max_cluster_size;
    }

    inline int getMaxClusterSize() {
      return (max_pts_per_cluster_);
    }

    inline void setSortedIndices(std::vector<IndexedValueType> &sorted_indices) {
      sorted_indices_ = sorted_indices;
    }

    inline std::vector<IndexedValueType> getSortedIndices() {
      return (sorted_indices_);
    }
        
    void extract(std::vector<pcl::PointIndices> &clusters, bool isIncremental = false);

  protected:
    // Members derived from the base class
    using BasePCLBase::input_;
    using BasePCLBase::indices_;
    using BasePCLBase::initCompute;
    using BasePCLBase::deinitCompute;

    KdTreePtr tree_;
    double cluster_tolerance_;
    int min_pts_per_cluster_;
    int max_pts_per_cluster_;

    virtual std::string getClassName() const {
      return ("SortedEuclideanClusterExtraction");
    }

    std::vector<IndexedValueType> sorted_indices_;

};

template <typename PointT> void
SortedEuclideanClusterExtraction<PointT>::extract(std::vector<pcl::PointIndices> &clusters, bool isIncremental) 
{
  if(!initCompute() ||
     (input_ != 0 && input_->points.empty()) ||
     (indices_ != 0 && indices_->empty())) 
  {
    clusters.clear();
    return;
  }

  // Initialize the spatial locator
  if (!tree_) 
  {
    if (input_->isOrganized())
      tree_.reset(new pcl::search::OrganizedNeighbor<PointT > ());
    else
      tree_.reset(new pcl::search::KdTree<PointT > (false));
  }

  // Send the input dataset to the spatial locator
  tree_->setInputCloud(input_, indices_);
  extractSortedEuclideanClusters(input_, sorted_indices_, tree_, clusters, 
                                 cluster_tolerance_, min_pts_per_cluster_, max_pts_per_cluster_, isIncremental);

  deinitCompute();
}

#define PCL_INSTANTIATE_SortedEuclideanClusterExtraction(T) template class PCL_EXPORTS SortedEuclideanClusterExtraction<T>;
#define PCL_INSTANTIATE_extractSortedEuclideanClusters(T) template void PCL_EXPORTS extractSortedEuclideanClusters<T>(ClusterDetector::Ptr, \\
typename const pcl::PointCloud<T>::ConstPtr, const std::vector<IndexedValueType> &, const std::shared_ptr<search::Search<PointT> > &, \\
std::vector<pcl::PointIndices> & , float , unsigned int, unsigned int, bool);

#endif
