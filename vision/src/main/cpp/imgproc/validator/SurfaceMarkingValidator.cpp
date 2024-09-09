/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * Detect markings (e.g., red cross) on a surface.
 *
 * @author Michael Zillich
 * @date August, 2013
 */

#include "SurfaceMarkingValidator.hpp"
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

using namespace diarc::stm;
using namespace std;
using namespace cv;

//#define DEBUG

SurfaceMarkingValidator::SurfaceMarkingValidator(const long long& processorId, const unsigned int imgWidth,
        const unsigned int imgHeight, const bool isStereo)
: ObjectValidator(processorId, imgWidth, imgHeight, isStereo) {
  visionProcessName = "SurfaceMarkingValidator";
  logger = log4cxx::Logger::getLogger("diarc.imgproc.validator.SurfaceMarkingValidator");
}

void SurfaceMarkingValidator::loadConfig(const std::string& config) {
  LOG4CXX_TRACE(logger, "[loadConfig] method entered.");

  //get directory
  unsigned found = config.find_last_of("/\\");
  std::string dir = config.substr(0, found + 1);

  // populate tree structure pt
  using boost::property_tree::ptree;
  ptree pt;
  read_xml(config, pt);

  std::string predicateName;

  // traverse pt
  std::string imageFilename, descriptorName;
  int arity;

  BOOST_FOREACH(ptree::value_type const& predicateNode, pt.get_child("processor")) {
    if (predicateNode.first.compare("predicate") == 0) {
      //functorName = predicateNode.second.get<std::string> ("<xmlattr>.functorname", "unknown");
     // arity = predicateNode.second.get<int> ("<xmlattr>.arity", -1);
      LOG4CXX_DEBUG(logger, boost::format("[loadConfig] name: %s.") % predicateNode.first );
      predicateName = predicateNode.second.get<std::string> ("<xmlattr>.name", "unknown");
      LOG4CXX_DEBUG(logger, boost::format("[loadConfig] predicateName: %s.") % predicateName);
      std::vector<std::string> imageFiles;

      //BOOST_FOREACH(ptree::value_type const& dataNode, predicateNode.second) {
        imageFiles.clear();
        //if (dataNode.first.compare("descriptor") == 0) {
         // descriptorName = descriptorNode.second.get<std::string> ("<xmlattr>.name", "unknown");
         // LOG4CXX_DEBUG(logger, boost::format("[loadConfig] descriptorName: %s.") % descriptorName);

          //get all filenames for descriptor

          BOOST_FOREACH(ptree::value_type const& imageNode, predicateNode.second) {
            if (imageNode.first.compare("image") == 0) {
              std::string image = static_cast<std::string> (imageNode.second.data());
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

void SurfaceMarkingValidator::handleMemoryObjectNotification(MemoryObjectNotification::ConstPtr notification) {
  MemoryObject::Ptr object = notification->object;
  SurfaceObject::Ptr surfaceObject = boost::dynamic_pointer_cast<SurfaceObject>(object);
  if (!surfaceObject) {
    LOG4CXX_ERROR(logger, "[haveNewObject] could not cast to SurfaceObject.");
    return;
  }

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
  string label;

  TypesByDescriptor::const_iterator bestmatch_iter;
  TypesByDescriptor::const_iterator descriptor_iter;
  std::map<std::string, ShapeModel>::iterator surfaceMarking_iter;
  // for all descriptors
  for (descriptor_iter = descriptors->begin(); descriptor_iter != descriptors->end(); ++descriptor_iter) {
    // for all surface markgins
    // NOTE: we check each sufface against all possible markings to find the nearest neighbor
    for (surfaceMarking_iter = surfaceMarkingMap.begin(); surfaceMarking_iter != surfaceMarkingMap.end(); ++surfaceMarking_iter) {
      // for all surfaces of the object
      for (size_t i = 0; i < surfaceObject->getSurfaceImages().size(); i++) {
        float c = 0.;
        bool detected = detect(surfaceMarking_iter->second, surfaceObject->getSurfaceImages()[i], c);
#ifdef DEBUG
        if (detected) {
          std::cout << "***** ";
          std::cout << " descriptors: " << descriptors->size() << ", markings: " << surfaceMarkingMap.size();
          std::cout << " - descr '" << descriptor_iter->first << "' marking '" << surfaceMarking_iter->first << "'   ";
          std::cout << " have '" << surfaceMarking_iter->first << "' conf: " << c << std::endl;
          static int cnt = 0;
          char filename[200];
          snprintf(filename, 200, "%s%03d-%.6f.png", surfaceMarking_iter->first.c_str(), cnt, c);
          imwrite(filename, surfaceObject->getSurfaceImages()[i]);
          cnt++;
        }
#endif
        if (detected) {
          if (c > confidence) {
            confidence = c;
            label = surfaceMarking_iter->first;
          }
        }
      }
    }
    // this is not the label you are looking for ...
    if (label == descriptor_iter->first.getName()) {
      bestmatch_iter = descriptor_iter;
    } else {
      label.clear();
      confidence = 0.;
    }
  }

  if (!label.empty()) {
    LOG4CXX_INFO(logger, boost::format("Frame: %llu. Found '%s'. Conf: %f.")
            % surfaceObject->getFrameNumber() % label % confidence);
    surfaceObject->addValidationResult(confidence, bestmatch_iter->first);
    sendValidationNotifications(surfaceObject);
  }
}

void SurfaceMarkingValidator::learn(ShapeModel &model, const Mat & img) {
  ShapeContext sc;
  calculateShapeContextBGR(img, sc);
  model.train(sc);
}

bool SurfaceMarkingValidator::detect(ShapeModel &model, const Mat &img, float &confidence) {
  ShapeContext sc;
  Mat crop, gray, edge;
  calculateShapeContextBGR(img, sc, crop, gray, edge);
  //  diarc::Display::createWindowIfDoesNotExist("img");
  //  diarc::Display::displayFrame(img, "img");
  //  diarc::Display::createWindowIfDoesNotExist("crop");
  //  diarc::Display::displayFrame(crop, "crop");
  //  diarc::Display::createWindowIfDoesNotExist("gray");
  //  diarc::Display::displayFrame(gray, "gray");
  //  diarc::Display::createWindowIfDoesNotExist("edge");
  //  diarc::Display::displayFrame(edge, "edge");
  //  sleep(2);
  //calculateShapeContextBGR(img, sc);
  float score = 0.0;
  ShapeContext match;
  bool detected = model.match(sc, score, confidence, match);
  //  LOG4CXX_INFO(logger, boost::format("Score: %f. Conf: %f.")
  //          % score % confidence);
  //
  //  std::string filename = "surface_" + boost::lexical_cast<std::string>(score) + "_" + boost::lexical_cast<std::string>(confidence) + ".png";
  //  imwrite(filename, img);
  return detected;
}
