/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "SiftDetector.hpp"

#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "capture/util/CaptureUtilities.hpp"
#include "display/Display.hpp"
#include "imgproc/siftFeatures/SiftFeatures.hpp"
#include "imgproc/SiftProcessor.hpp"
#include "common/notification/SiftNotification.hpp"

using namespace diarc::stm;

SiftDetector::SiftDetector(const long long& processorId, const int imgWidth, const int imgHeight)
: ObjectDetector(processorId, imgWidth, imgHeight),
SIFT_MATCH_THRESH(10) {
  visionProcessName = "SiftDetector";
  logger = log4cxx::Logger::getLogger("diarc.detector.SiftDetector");
}

SiftDetector::~SiftDetector() {

}

void SiftDetector::cleanup() {
  saveSiftsToFile = true;
}

void SiftDetector::registerForCaptureNotification() {
  // intentionally empty
}

void SiftDetector::unregisterForCaptureNotification() {
  // intentionally empty
}

void SiftDetector::loadConfig(const std::string& configFile) {
  //get directory
  unsigned found = configFile.find_last_of("/\\");
  std::string dir = configFile.substr(0, found + 1);

  // populate tree structure pt
  using boost::property_tree::ptree;
  ptree pt;
  read_xml(configFile, pt);

   std::string predicateName;
  // parse xml file

  BOOST_FOREACH(ptree::value_type const& predicateNode, pt.get_child("processor")) {
    if (predicateNode.first.compare("predicate") == 0) {
      LOG4CXX_DEBUG(logger, boost::format("[loadConfig] name: %s.") % predicateNode.first);

      predicateName = predicateNode.second.get<std::string> ("<xmlattr>.name", "unknown");
      LOG4CXX_DEBUG(logger, boost::format("[loadConfig] predicateName: %s.") % predicateName);
      
      BOOST_FOREACH(ptree::value_type const& dataNode, predicateNode.second) {
        if (dataNode.first.compare("config") == 0) {

          std::string siftFilename = dataNode.second.get<std::string> ("<xmlattr>.filename", "unknown");
          sift_filenames[predicateName] = dir + siftFilename;
          LOG4CXX_DEBUG(logger, boost::format("[loadConfig] siftFilename: %s.") % sift_filenames[predicateName]);
        }
      }
    }
  }
}

void SiftDetector::handleSiftNotification(SiftNotification::ConstPtr notification) {

  //get sift results
  SiftFeatures::ConstPtr siftFromImg = notification->siftFeatures;
  CaptureData::ConstPtr captureData = notification->captureData;
  if (siftFromImg->vlfeat_keys->size() == 0) {
    LOG4CXX_WARN(logger, "Did not receive any SIFT features from FE: make sure SIFT is running!\n");
    return;
  }

  // HACK to save sift features to file
  if (saveSiftsToFile) {
    writeKeyFile("tmpSifts.keys", siftFromImg->vlfeat_keys);
    saveSiftsToFile = false;
  }
  // END HACK

  MemoryObject::VecPtr newSiftObjects(new MemoryObject::Vec()); //for display
  TypesByDescriptorConstPtr descriptors = getDescriptors();
  TypesByDescriptor::const_iterator descriptor_iter;
  std::tr1::unordered_set<long long>::const_iterator typeIds_itr;
  for (descriptor_iter = descriptors->begin(); descriptor_iter != descriptors->end(); ++descriptor_iter) {
    //load sift target from file if not already loaded
    SiftFeatureVectPtr target = target_sifts[descriptor_iter->first.getName()];
    if (!target) {
      std::string filename = sift_filenames[descriptor_iter->first.getName()];
      target = loadKeyFile(filename);
      if (!target || target->empty()) {
        LOG4CXX_ERROR(logger, boost::format("Could not load sift target: %s.") % filename);
        sift_filenames.erase(descriptor_iter->first.getName());
        continue;
      } else {
        target_sifts[descriptor_iter->first.getName()] = target;
      }
    }

    //TODO: rest of work here
    if (target->size() == 0) {
      LOG4CXX_WARN(logger, "Did not receive any SIFT features from file. Make sure file is populated for type.\n");
      continue;
    }

    MemoryObject::Ptr newSiftObject;
    cv::Rect boundingBox;
    if (findSiftObject(target, siftFromImg, captureData, boundingBox)) {
      // construct MO for each typeId
      for (typeIds_itr = descriptor_iter->second.begin(); typeIds_itr != descriptor_iter->second.end(); ++typeIds_itr) {
        LOG4CXX_DEBUG(logger, boost::format("Sift object detected: %s with bb (%d,%d,%d,%d).")
                % descriptor_iter->first.getArg(0) % boundingBox.x % boundingBox.y % boundingBox.width % boundingBox.height);
        newSiftObject = MemoryObject::Ptr(new MemoryObject(*typeIds_itr, descriptor_iter->first.getArg(0), captureData, boundingBox));
        newSiftObject->addValidationResult(0.9, descriptor_iter->first);
        newSiftObjects->push_back(newSiftObject);
        // send notifications with newly detected object
        sendDetectionNotifications(newSiftObject);
      }

    }
  }

  //draw results if should be displayed
  if (getDisplayFlag()) {
    //get captured image and data
    const cv::Mat currFrame = captureData->frame;
    currFrame.copyTo(displayFrame);

    MemoryObject::Vec::const_iterator newObjItr;
    for (newObjItr = newSiftObjects->begin(); newObjItr != newSiftObjects->end(); ++newObjItr) {

      const cv::Rect& r = (*newObjItr)->getDetectionMask()->getBoundingBox();
      cv::rectangle(displayFrame, cv::Point(r.x, r.y),
              cv::Point(r.x + r.width, r.y + r.height),
              CV_RGB(255, 0, 0), 2, 8, 0);
    }

    diarc::Display::displayFrame(displayFrame, getDisplayName());
  }
}

bool SiftDetector::findSiftObject(const SiftFeatureVectPtr& targetSifts,
        SiftFeatures::ConstPtr& imgSifts,
        CaptureData::ConstPtr captureData, cv::Rect & boundingBox) {

  //find matches
  SiftFeatureVectPtr::element_type::const_iterator imgFeatItr;
  SiftFeature::Ptr match;
  SiftFeatureVectPtr matches(new SiftFeatureVect());
  SiftFeatureVectPtr nonMatches(new SiftFeatureVect()); //for display purposes only
  double xMean = 0;
  double yMean = 0;
  double xMean2 = 0;
  double yMean2 = 0;
  int matchCount = 0;
  for (imgFeatItr = imgSifts->vlfeat_keys->begin(); imgFeatItr != imgSifts->vlfeat_keys->end(); ++imgFeatItr) {
    match = SiftFeatureExtractor::CheckForMatch((*imgFeatItr), targetSifts);

    if (match != SiftFeature::Ptr()) {
      matches->push_back((*imgFeatItr));
      xMean = (xMean * matchCount + (*imgFeatItr)->x) / (matchCount + 1);
      yMean = (yMean * matchCount + (*imgFeatItr)->y) / (matchCount + 1);
      xMean2 = (xMean2 * matchCount + (*imgFeatItr)->x * (*imgFeatItr)->x) / (matchCount + 1);
      yMean2 = (yMean2 * matchCount + (*imgFeatItr)->y * (*imgFeatItr)->y) / (matchCount + 1);
      matchCount++;
    } else {
      nonMatches->push_back((*imgFeatItr));
    }
  }

  double xStddev = sqrt(xMean2 - xMean * xMean);
  double yStddev = sqrt(yMean2 - yMean * yMean);

  //TODO: remove outliers here

  if (logger->isDebugEnabled()) {
    cv::Mat debugFrame = captureData->frame.clone();
    SiftFeatureVectPtr::element_type::const_iterator matchItr;
    for (matchItr = matches->begin(); matchItr != matches->end(); ++matchItr) {
      //            cvCircle(displayImage, cvPoint((int)(*matchItr)->x, (int)(*matchItr)->y), (int)(*matchItr)->sigma, CV_RGB(255, 0, 0), 1, 8, 0);
      cv::circle(debugFrame, cv::Point((int) (*matchItr)->x, (int) (*matchItr)->y), (int) (*matchItr)->sigma, CV_RGB(255, 0, 0), 1, 8, 0);
    }
    SiftFeatureVectPtr::element_type::const_iterator nonMatchItr;
    for (nonMatchItr = nonMatches->begin(); nonMatchItr != nonMatches->end(); ++nonMatchItr) {
      //            cvCircle(displayImage, cvPoint((int)(*matchItr)->x, (int)(*matchItr)->y), (int)(*matchItr)->sigma, CV_RGB(255, 0, 0), 1, 8, 0);
      cv::circle(debugFrame, cv::Point((int) (*nonMatchItr)->x, (int) (*nonMatchItr)->y), (int) (*nonMatchItr)->sigma, CV_RGB(255, 255, 0), 1, 8, 0);
    }
    diarc::Display::createWindowIfDoesNotExist(getDisplayName() + "_debug");
    diarc::Display::displayFrame(debugFrame, getDisplayName() + "_debug");
  }

  //create detected object from matches
  //cout << type << " matches: " << matches->size() << " / " << siftFromImg->size() << " From db: " << siftFromDB->size() << endl;
  if (matches->size() > SIFT_MATCH_THRESH) {
    boundingBox = cv::Rect(xMean - xStddev, yMean - yStddev, 2 * xStddev, 2 * yStddev);

    return true;
  }

  return false;
}

/* Read keypoints from the given file and save them in the vector of
   keypoints.  The file format starts with 2 integers giving the total
   number of keypoints and the size of descriptor vector for each
   keypoint (currently assumed to be 128). Then each keypoint is
   specified by 4 floating point numbers giving subpixel row and
   column location, scale, and orientation (in radians from -PI to
   PI). Then the descriptor vector for each keypoint is given as a
   list of floats in range [0,255].
 */
SiftFeatureVectPtr SiftDetector::loadKeyFile(const std::string & filenamePath) {
  FILE *file;

  file = fopen(filenamePath.c_str(), "r");
  if (!file) {
    LOG4CXX_ERROR(logger, boost::format("Could not open file: %s.") % filenamePath);
    return SiftFeatureVectPtr();
  }

  int i, j, num, len;
  vl_sift_pix val;
  SiftFeature::Ptr newFeat; //(new SiftFeature());// = NULL;

  if (fscanf(file, "%d %d", &num, &len) != 2) {
    LOG4CXX_ERROR(logger, "Invalid keypoint file beginning.");
    return SiftFeatureVectPtr();
  }

  if (len != 128) {
    LOG4CXX_ERROR(logger, "Keypoint descriptor length invalid (should be 128).");
    return SiftFeatureVectPtr();
  }

  //overwrites sift features for existing types.  having duplicate descriptors invalidates possible matches against the duplicated feature
  SiftFeatureVectPtr results = SiftFeatureVectPtr(new SiftFeatureVect());
  results.reset(new SiftFeatureVect());

  for (i = 0; i < num; i++) {
    newFeat.reset(new SiftFeature());

    if (fscanf(file, "%f %f %f %f", &(newFeat->y), &(newFeat->x), &(newFeat->sigma), &(newFeat->ori)) != 4) {
      LOG4CXX_ERROR(logger, "Invalid keypoint file format.");
      continue;
    }

    for (j = 0; j < len; j++) {
      if (fscanf(file, "%f", &val) != 1 || val < 0 || val > 255) {
        LOG4CXX_ERROR(logger, boost::format("Invalid keypoint file value: %f.") % val);
        continue;
      }
      newFeat->descriptor[j] = (vl_sift_pix) val;
    }
    results->push_back(newFeat);
  }

  fclose(file);
  LOG4CXX_INFO(logger, boost::format("Loaded %d SIFT keypoints.") % results->size());

  return results;
}

void SiftDetector::writeKeyFile(const std::string & filenamePath, SiftFeatureVectPtr siftFeatures) const {
  FILE *file = fopen(filenamePath.c_str(), "wt");

  if (!file) {
    LOG4CXX_ERROR(logger, boost::format("Could not open file: %s.") % filenamePath);
    return;
  }

  int num = siftFeatures->size();
  int len = 128;
  fprintf(file, "%d %d\n", num, len);

  for (int i = 0; i < num; i++) {
    SiftFeature::Ptr currFeat = siftFeatures->at(i);
    fprintf(file, "%f %f %f %f\n", currFeat->y, currFeat->x, currFeat->sigma, currFeat->ori);
    for (int j = 0; j < len; j++) {
      fprintf(file, "%f ", currFeat->descriptor[j]);
    }
    fprintf(file, "\n");
  }

  fclose(file);
  LOG4CXX_INFO(logger, boost::format("Saved %d SIFT keypoints.") % num);
}
