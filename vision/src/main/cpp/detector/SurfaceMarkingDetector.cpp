/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * Detect markings (e.g., red cross) in an image.
 *
 * @author Evan Krause
 * @date May, 2015
 */

#include "SurfaceMarkingDetector.hpp"
#include "capture/util/CaptureUtilities.hpp"
#include "display/Display.hpp"

#include <iostream>
#include <algorithm>
#include <boost/format.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/nonfree/nonfree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <boost/smart_ptr/intrusive_ptr.hpp>

using namespace ade::stm;
using namespace std;
using namespace cv;

#define DEBUG

SurfaceMarkingDetector::SurfaceMarkingDetector(const long long& processorId, const unsigned int imgWidth,
        const unsigned int imgHeight)
: ObjectDetector(processorId, imgWidth, imgHeight) {
  visionProcessName = "SurfaceMarkingDetector";
  logger = log4cxx::Logger::getLogger("ade.imgproc.detector.SurfaceMarkingDetector");
}

void SurfaceMarkingDetector::loadConfig(const string& config) {
  LOG4CXX_TRACE(logger, "[loadConfig] method entered.");

  //get directory
  unsigned found = config.find_last_of("/\\");
  string dir = config.substr(0, found + 1);

  // populate tree structure pt
  using boost::property_tree::ptree;
  ptree pt;
  read_xml(config, pt);

  // traverse pt
  string imageFilename, descriptorName, functorName;
  int arity;

  BOOST_FOREACH(ptree::value_type const& predicateNode, pt.get_child("processor")) {
    if (predicateNode.first.compare("predicate") == 0) {
      functorName = predicateNode.second.get<string> ("<xmlattr>.functorname", "unknown");
      arity = predicateNode.second.get<int> ("<xmlattr>.arity", -1);
      LOG4CXX_DEBUG(logger, boost::format("[loadConfig] name: %s. functor name: %s. arity: %d.") % predicateNode.first % functorName % arity);

      vector<string> imageFiles;

      BOOST_FOREACH(ptree::value_type const& descriptorNode, predicateNode.second) {
        imageFiles.clear();
        if (descriptorNode.first.compare("descriptor") == 0) {
          descriptorName = descriptorNode.second.get<string> ("<xmlattr>.name", "unknown");
          LOG4CXX_DEBUG(logger, boost::format("[loadConfig] descriptorName: %s.") % descriptorName);

          //get all filenames for descriptor

          BOOST_FOREACH(ptree::value_type const& imageNode, descriptorNode.second) {
            if (imageNode.first.compare("image") == 0) {
              string image = static_cast<string> (imageNode.second.data());
              LOG4CXX_DEBUG(logger, boost::format("[loadConfig] dir: %s. image: %s") % dir % image);
              imageFilename = dir + image;
              imageFiles.push_back(imageFilename);
            }
          }

          //build shape model
          ShapeModel model;
          trainModel(model, imageFiles);
          surfaceMarkingMap[descriptorName] = model;
        }
      }
    }
  }
}

void SurfaceMarkingDetector::handleCaptureNotification(CaptureNotification::ConstPtr capture) {
  // get image
  const cv::Mat frame = capture->captureData->frame;

  // HACK
  /*// write out all projected surface images to collect training data
  char filename[200];
  static int cnt = 0;
  for(size_t i = 0; i < surfaceObject->getSurfaceImages().size(); i++) {
    snprintf(filename, 200, "surf%03d.png", cnt);
    cnt++;
    imwrite(filename, surfaceObject->getSurfaceImages()[i]);
  }*/
  // HACK END

  // get current descriptors to process
  TypesByDescriptorConstPtr descriptors = getDescriptors();
  if (descriptors->size() == 0) {
    LOG4CXX_ERROR(logger, "No descriptors set, returning.");
    return;
  }

  float confidence = 0.0f;

  TypesByDescriptor::const_iterator bestmatch_iter = descriptors->end();

  // for all descriptors
  TypesByDescriptor::const_iterator descriptor_iter;
  std::tr1::unordered_set<long long>::const_iterator typeIds_itr;
  for (descriptor_iter = descriptors->begin(); descriptor_iter != descriptors->end(); ++descriptor_iter) {
    // for all surface markings
    // NOTE: we check each surface against all possible markings to find the nearest neighbor
    map<string, ShapeModel>::iterator surfaceMarking_iter = surfaceMarkingMap.find(descriptor_iter->first.getName());
    // for all surfaces of the object
    float conf = 0.;
    bool detected = detect(surfaceMarking_iter->second, frame, conf);
#ifdef DEBUG
    if (detected) {
      cout << "***** ";
      cout << " descriptors: " << descriptors->size() << ", markings: " << surfaceMarkingMap.size();
      cout << " - descr '" << descriptor_iter->first.toString() << "' marking '" << surfaceMarking_iter->first << "'   ";
      cout << " have '" << surfaceMarking_iter->first << "' conf: " << conf << endl;
      static int cnt = 0;
      char filename[200];
      snprintf(filename, 200, "%s%03d-%.6f.png", surfaceMarking_iter->first.c_str(), cnt, conf);
      imwrite(filename, frame);
      cnt++;
    }
#endif
    if (detected) {
      if (conf > confidence) {
        confidence = conf;
        bestmatch_iter = descriptor_iter;
      }
    }
  }

  if (bestmatch_iter != descriptors->end()) {
    LOG4CXX_INFO(logger, boost::format("Frame: %llu. Found '%s'. Conf: %f.")
            % capture->frameNumber % bestmatch_iter->first.toString() % confidence);

    // TODO: make proper rect
    cv::Rect tmpRect(img_width / 2, img_height / 2, img_width / 4, img_height / 4);


    // create MemoryObject for each relevant typeId
    for (typeIds_itr = descriptor_iter->second.begin(); typeIds_itr != descriptor_iter->second.end(); ++typeIds_itr) {
      MemoryObject::Ptr newMemoryObject(new MemoryObject(*typeIds_itr, bestmatch_iter->first.getArg(0), capture->captureData, tmpRect));
      newMemoryObject->addValidationResult(confidence, bestmatch_iter->first);

      sendDetectionNotifications(newMemoryObject);
    }
  }
}

void SurfaceMarkingDetector::learn(ShapeModel &model, const Mat & img) {
  ShapeContext sc;
  Mat crop, gray, edge;
  calculateShapeContextBGR(img, sc, crop, gray, edge);
  //calculateShapeContextBGR(img, sc);
  edge.copyTo(sc.image);
  model.train(sc);
}

bool SurfaceMarkingDetector::detect(ShapeModel &model, const Mat &img, float &confidence) {
  ShapeContext sc;
  Mat crop, gray, edge;
  calculateShapeContextBGR(img, sc, crop, gray, edge);
  ade::Display::createWindowIfDoesNotExist("img");
  ade::Display::displayFrame(img, "img");
  ade::Display::createWindowIfDoesNotExist("crop");
  ade::Display::displayFrame(crop, "crop");
  ade::Display::createWindowIfDoesNotExist("gray");
  ade::Display::displayFrame(gray, "gray");
  ade::Display::createWindowIfDoesNotExist("edge");
  ade::Display::displayFrame(edge, "edge");
  sleep(2);
  //calculateShapeContextBGR(img, sc);
  float score = 0.0;
  ShapeContext matching_sc;
  bool detected = model.match(sc, score, confidence, matching_sc);

  // DEBUG
  //if (detected) {

  LOG4CXX_INFO(logger, boost::format("Detected: %s. Score: %f. Conf: %f.")
          % (detected ? "true" : "false") % score % confidence);

  //Mat debugImg = matchAndAlignEdgeImages(matching_sc.image, edge);
  Mat debugImg = Mat::zeros(edge.rows, edge.cols, edge.type());
  MYPOINT *points = NULL;
  int nPoints;
  binaryEdgeImageToPoints(edge, points, nPoints);
  drawResults(points, nPoints, debugImg);
  ade::Display::createWindowIfDoesNotExist("debugImg");
  ade::Display::displayFrame(debugImg, "debugImg");
  //}

  // DEBUG
  //  LOG4CXX_INFO(logger, boost::format("Score: %f. Conf: %f.")
  //          % score % confidence);
  //
  //  string filename = "surface_" + boost::lexical_cast<string>(score) + "_" + boost::lexical_cast<string>(confidence) + ".png";
  //  imwrite(filename, img);
  return detected;
}
