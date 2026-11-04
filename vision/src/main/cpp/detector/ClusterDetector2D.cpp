/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "ClusterDetector2D.hpp"

#include "display/Display.hpp"
#include "capture/util/CaptureUtilities.hpp"
#include <cfloat>

using namespace diarc::stm;

bool IndexedValueTypeSortFunction2D(IndexedValueType2D i, IndexedValueType2D j) {
  return (i.value > j.value);
  //return (i.value < j.value);//to reverse sort (largest valued pixel, last)
}

ClusterDetector2D::ClusterDetector2D(const long long& processorId, const int imgWidth, const int imgHeight)
: ObjectDetector(processorId, imgWidth, imgHeight),
haveNewSalmap(false),
saliencyMap(),
sorted_indices(),
processed_indices(),
background_dist(0.1),
writeFramesToFile(false) {
  visionProcessName = "ClusterDetector2D";
  logger = log4cxx::Logger::getLogger("diarc.detector.ClusterDetector2D");
}

ClusterDetector2D::~ClusterDetector2D() {

}

void ClusterDetector2D::setSortedSaliencyMap(const cv::Mat& salmap) {
  if (salmap.cols != img_width || salmap.rows != img_height) {
    LOG4CXX_WARN(logger, "[sortSaliencyMap] saliency map of incorrect size.");
    return;
  }

  //fill based on saliency map, starting with bottom right of image, so popping
  //stack results in row-wise processing from top-left (only matters for pixels
  //with  equal saliency values)
  sorted_indices.clear();
  sorted_indices.resize(img_width * img_height);
  int index_counter = 0;
  for (int r = 0; r < img_height; ++r) {
    for (int c = 0; c < img_width; ++c) {
      //for (int r = img_height - 1; r >= 0; --r) {
      //  for (int c = img_width - 1; c >= 0; --c) {
      sorted_indices.at(index_counter++) = IndexedValueType2D(salmap.at<float>(r, c), r, c, true);
    }
  }

  //sort indices based on saliency value
  std::vector<IndexedValueType2D>::iterator it = sorted_indices.begin();
  std::vector<IndexedValueType2D>::iterator it_end = sorted_indices.end();
  std::sort(it, it_end, IndexedValueTypeSortFunction2D);
}

void ClusterDetector2D::setUniformSaliencyMap() {
  //fill with all 1s, starting with bottom right of image, so popping stack
  //results in row-wise from top-left processing
  sorted_indices.clear();
  sorted_indices.resize(img_width * img_height);
  int index_counter = 0;
  for (int r = img_height - 1; r >= 0; --r) {
    for (int c = img_width - 1; c >= 0; --c) {
      sorted_indices.at(index_counter++) = IndexedValueType2D(1.0f, r, c, true);
    }
  }
}

void ClusterDetector2D::handleSaliencyNotification(SaliencyNotification::ConstPtr notification) {
  const unsigned long long& frameNum = notification->frameNumber;
  const cv::Mat& newSaliencyMap = notification->saliencyMap;
  LOG4CXX_DEBUG(logger, boost::format("[haveNewSaliencyMap] frame num: %llu.") % frameNum);
  newSaliencyMap.copyTo(saliencyMap);
  haveNewSalmap = true;

  //  haveNewImage(frameNum);
}

void ClusterDetector2D::handleCaptureNotification(CaptureNotification::ConstPtr notification) {
  const long long frameNum = notification->captureData->frameNumber;
  LOG4CXX_DEBUG(logger, boost::format("[haveNewImage] method entered. frame num: %llu.") % frameNum);
  /////////////////////////////////
  //TODO: check that saliency results and image have same framenum
  //what to do if they don't ???
  /////////////////////////////////

  //reset which pixels have been processed
  processed_indices = std::vector<bool>(img_height*img_width, false);

  //reset
  currSortedIndex = 0;

  if (haveNewSalmap) {
    setSortedSaliencyMap(saliencyMap);
  } else {
    return;
    //if there's no saliency information, set uniform saliency 
    //setUniformSaliencyMap();
  }

  const cv::Mat image = notification->captureData->frame;
  bool firstDisplayIteration = true;

  // perform clustering (one cluster at a time)
  LOG4CXX_DEBUG(logger, "[haveNewImage] starting clustering.");
  std::vector<int> cluster;
  while (nextCluster(image, cluster)) {
    LOG4CXX_DEBUG(logger, boost::format("[haveNewImage] have new cluster. frame num: %llu.") % frameNum);

    //create memory object out of cluster
    MemoryObject::Ptr newCluster;
    DescriptorsByTypeConstPtr types = getTypes();
    DescriptorsByType::const_iterator types_iter;
    for (types_iter = types->begin(); types_iter != types->end(); ++types_iter) {
      if (types_iter->second.begin() != types_iter->second.end()) {
        long long typeId = types_iter->first;
        std::string variableName = types_iter->second.begin()->getArg(0);
        newCluster = MemoryObject::Ptr(new MemoryObject(typeId, variableName, notification->captureData, cluster));

        // send newly detected object notifications
        sendDetectionNotifications(newCluster);
      } else {
        LOG4CXX_ERROR(logger, "[haveNewImage] No descriptors found.");
      }
    }

    //display
    if (getDisplayFlag()) {

      //if new frame, reset display frame to remove old boxes
      if (firstDisplayIteration) {
        firstDisplayIteration = false;
        image.copyTo(displayFrame);
      }

      //draw on cluster box
      const cv::Rect& rect = newCluster->getDetectionMask()->getBoundingBox();
      cv::rectangle(displayFrame, cv::Point(rect.x, rect.y),
              cv::Point(rect.x + rect.width, rect.y + rect.height),
              CV_RGB(0, 0, 0),
              2, 8, 0);

      diarc::Display::displayFrame(displayFrame, getDisplayName());

      //write image to file
      if (writeFramesToFile) {
        char a[6];
        sprintf(a, "%d", (int) frameNum);
        std::string name = "detector";
        cv::imwrite(name + a + ".ppm", displayFrame);
      }
    }

    //TODO: allow saliency notifications to interrupt clustering
    //LOG4CXX_DEBUG(logger, "[haveNewImage] checking for saliency notifications.");
  }
  LOG4CXX_DEBUG(logger, "[haveNewImage] finished clustering.");
}

//
//void ClusterDetector2D::cluster(const cv::Mat &image, std::vector<std::vector<int> >& clusters,
//        int min_pts_per_cluster, int max_pts_per_cluster) {
//  clusters.clear();
//
//  int sizeNeighbourhood = 4;
//  int dx[4] = {0, 1, 0, -1};
//  int dy[4] = {-1, 0, 1, 0};
//
//  // Create a bool vector of processed point indices, and initialize it to false
//  std::vector<bool> processed(img_height*img_width, false);
//
//  // Process all points in the indices vector
//  for (size_t i = 0; i < sorted_indices.size(); ++i) {
//    int index = sorted_indices.at(i).index_r * img_width + sorted_indices.at(i).index_c;
//
//    if (processed.at(index)) {
//      continue;
//    }
//
//    if (!sorted_indices.at(i).valid) {
//      continue;
//    }
//
//    std::vector<int> seed_queue;
//    int sq_idx = 0;
//    seed_queue.push_back(index);
//    std::vector<int> nn_indices;
//
//    processed.at(index) = true;
//
//    //build cluster, one seed at time
//    while (sq_idx < (int) seed_queue.size()) {
//      int x = seed_queue.at(sq_idx) % img_width;
//      int y = seed_queue.at(sq_idx) / img_width;
//      if (isBackgroundPixel(image, x, y)) {
//        continue;
//      }
//
//      //find neighborhood
//      for (int dxy = 0; dxy < sizeNeighbourhood; ++dxy) {
//        int x_new = x + dx[dxy];
//        int y_new = y + dy[dxy];
//
//        if ((x_new >= 0) && (x_new < img_width) && (y_new >= 0) && (y_new < img_height)) {
//          int new_index = y_new * img_width + x_new;
//          nn_indices.push_back(new_index);
//        }
//      }
//
//      // nn_indices[0] should be sq_idx
//      for (size_t j = 1; j < nn_indices.size(); ++j) {
//
//        // Has this point been processed before ?
//        if (processed.at(nn_indices.at(j))) {
//          continue;
//        }
//
//        int x = seed_queue.at(sq_idx) % img_width;
//        int y = seed_queue.at(sq_idx) / img_width;
//        if (isBackgroundPixel(image, x, y)) {
//          continue;
//        }
//
//        // Perform a simple Euclidean clustering
//        seed_queue.push_back(nn_indices.at(j));
//        processed.at(nn_indices.at(j)) = true;
//
//        //        image.at<uchar>(y, 2 * c + 0) = 0;
//        //        image.at<uchar>(y, 2 * c + 1) = 0;
//        //        image.at<uchar>(y, 2 * c + 2) = 0;
//      }
//
//      sq_idx++;
//    }
//
//    // If this queue is satisfactory, add to the clusters
//    if (seed_queue.size() >= min_pts_per_cluster && seed_queue.size() <= max_pts_per_cluster) {
//      std::vector<int> cls = seed_queue;
//
//      sort(cls.begin(), cls.end());
//      cls.erase(std::unique(cls.begin(), cls.end()), cls.end());
//
//      clusters.push_back(cls); // We could avoid a copy by working directly in the vector
//    }
//  }
//}

bool ClusterDetector2D::nextCluster(const cv::Mat &image, std::vector<int>& cluster,
        int min_pts_per_cluster, int max_pts_per_cluster) {
  LOG4CXX_DEBUG(logger, "[nextCluster] method entered.");

  int sizeNeighbourhood = 4;
  int dx[4] = {0, 1, 0, -1};
  int dy[4] = {-1, 0, 1, 0};

  // Process all points in the indices vector
  LOG4CXX_DEBUG(logger, boost::format("[nextCluster] finding next cluster starting at sorted index %d.") % currSortedIndex);
  bool clusterFound = false;
  //while (!clusterFound && sorted_indices.size() > 0) {
  //IndexedValueType2D sortedIndex = sorted_indices.back();
  //sorted_indices.pop_back();
  while (!clusterFound && currSortedIndex < sorted_indices.size()) {
    IndexedValueType2D sortedIndex = sorted_indices[currSortedIndex++];
    int index = sortedIndex.index_r * img_width + sortedIndex.index_c;

    if (processed_indices.at(index)) {
      continue;
    }

    std::vector<int> seed_queue;
    int sq_idx = 0;
    seed_queue.push_back(index);
    std::vector<int> nn_indices;

    processed_indices.at(index) = true;

    //build cluster, one seed at time
    while (sq_idx < (int) seed_queue.size()) {
      int x = seed_queue.at(sq_idx) % img_width;
      int y = seed_queue.at(sq_idx) / img_width;
      if (isBackgroundPixel(image, x, y)) {
        ++sq_idx;
        continue;
      }

      //find neighborhood
      for (int dxy = 0; dxy < sizeNeighbourhood; ++dxy) {
        int x_new = x + dx[dxy];
        int y_new = y + dy[dxy];

        if ((x_new >= 0) && (x_new < img_width) && (y_new >= 0) && (y_new < img_height)) {
          int new_index = y_new * img_width + x_new;
          nn_indices.push_back(new_index);
        }
      }

      // nn_indices[0] should be sq_idx
      for (size_t j = 1; j < nn_indices.size(); ++j) {

        // Has this point been processed before ?
        if (processed_indices.at(nn_indices.at(j))) {
          continue;
        }

        int x = seed_queue.at(sq_idx) % img_width;
        int y = seed_queue.at(sq_idx) / img_width;
        if (isBackgroundPixel(image, x, y)) {
          continue;
        }

        // Perform a simple Euclidean clustering
        seed_queue.push_back(nn_indices.at(j));
        processed_indices.at(nn_indices.at(j)) = true;

        //        image.at<uchar>(y, 2 * c + 0) = 0;
        //        image.at<uchar>(y, 2 * c + 1) = 0;
        //        image.at<uchar>(y, 2 * c + 2) = 0;
      }

      sq_idx++;
    }

    // If this queue is satisfactory, add to the clusters
    if (seed_queue.size() >= min_pts_per_cluster && seed_queue.size() <= max_pts_per_cluster) {

      cluster = seed_queue;

      sort(cluster.begin(), cluster.end());
      cluster.erase(std::unique(cluster.begin(), cluster.end()), cluster.end());
      clusterFound = true;
    }
  }

  return clusterFound;
}

bool ClusterDetector2D::isBackgroundPixel(const cv::Mat& image, const int x, const int y) {
  //background of images from spivey dataset is approximately white, use
  //that as the background model to segment foreground objects
  //calculate dist to white
  float r_color = image.at<uchar>(y, 3 * x + 0);
  float g_color = image.at<uchar>(y, 3 * x + 1);
  float b_color = image.at<uchar>(y, 3 * x + 2);
  r_color /= 255;
  g_color /= 255;
  b_color /= 255;
  float dist = sqrt((1 - r_color)*(1 - r_color) + (1 - g_color)*(1 - g_color) + (1 - b_color)*(1 - b_color));

  if (dist > background_dist) {
    return false;
  }

  return true;
}
