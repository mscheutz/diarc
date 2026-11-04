
/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "OpPoseDetector.hpp"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>

#include "display/Display.hpp"
#include "capture/util/CaptureUtilities.hpp"
#include "common/notification/CaptureNotification.hpp"


// C++ std library dependencies
#include <chrono> // `std::chrono::` functions and classes, e.g. std::chrono::milliseconds
#include <string>
#include <thread> // std::this_thread
#include <vector>
// Other 3rdparty dependencies
#include <gflags/gflags.h> // DEFINE_bool, DEFINE_int32, DEFINE_int64, DEFINE_uint64, DEFINE_double, DEFINE_string
#include <openpose/headers.hpp>


using namespace diarc::stm;

OpPoseDetector::OpPoseDetector(const long long &processorId, const int imgWidth, const int imgHeight)
        : ObjectDetector(processorId, imgWidth, imgHeight),
          opWrapper(),
          wrapperStructPose(),
          wrapperStructFace(),
          wrapperStructHand(),
          wrapperStructOutput(),
          jointMap() {
  visionProcessName = "OpPoseDetector";
  logger = log4cxx::Logger::getLogger("diarc.detector.OpPoseDetector");
}

OpPoseDetector::~OpPoseDetector()
{ 
  if (opWrapper != NULL) {
    delete opWrapper;
  }
  return;
}

void OpPoseDetector::init() {
  //Configure OpenPose
 if (opWrapper == NULL) {
    // not a template error
    // opWrapper = new op::Wrapper<std::vector<op::Datum>>{op::ThreadManagerMode::Asynchronous};
    // LOG4CXX_ERROR(logger, "opWrapper was NULL, and thus cannot be configured");
    opWrapper = new op::Wrapper{op::ThreadManagerMode::Asynchronous};
 }

  // Configure wrapper
  opWrapper->configure(wrapperStructPose);
  opWrapper->configure(wrapperStructFace);
  opWrapper->configure(wrapperStructHand); 
  opWrapper->configure(op::WrapperStructInput{});
  opWrapper->configure(wrapperStructOutput);

  //start the threads
  opWrapper->start();

}

void OpPoseDetector::cleanup() {
  opWrapper->stop();
}


void OpPoseDetector::loadConfig(const std::string &configFile) {
  //get directory
  unsigned found = configFile.find_last_of("/\\");
  std::string dir = configFile.substr(0, found + 1);

  // populate tree structure pt
  using boost::property_tree::ptree;
  ptree pt;
  read_xml(configFile, pt);

  // temp joints container
  std::vector<std::pair<int, std::string>> joints;

  // parse xml file
  BOOST_FOREACH(ptree::value_type const& predicateNode, pt.get_child("processor")) {
          if (predicateNode.first.compare("predicate") == 0) {
            std::string functorName = predicateNode.second.get<std::string>("<xmlattr>.name", "unknown");
            LOG4CXX_DEBUG(logger, boost::format("[loadConfig] functorName: %s.") % functorName);

            joints.clear();
            BOOST_FOREACH(ptree::value_type const& jointNode, predicateNode.second) {
                    if (jointNode.first.compare("joint") == 0) {

                      int jointIndex = jointNode.second.get<int>("<xmlattr>.index", -1);
                      std::string jointName = jointNode.second.get<std::string>("<xmlattr>.name", "unknown");
                      LOG4CXX_DEBUG(logger,
                                    boost::format("[loadConfig] joint pair: (%d, %s).") % jointIndex % jointName);
                      joints.push_back(std::pair<int, std::string>(jointIndex, jointName));
                    }
                  }
            jointMap[functorName].push_back(joints);
          } else if (predicateNode.first.compare("config") == 0) {
            std::string openposeConfigFile =
                    dir + predicateNode.second.get<std::string>("<xmlattr>.configOpenpose", "unknown");
            loadOpenPoseConfig(openposeConfigFile);

          }
        }

}

void OpPoseDetector::loadOpenPoseConfig(const std::string &config) {
  LOG4CXX_DEBUG(logger, boost::format("[loadOpenPoseConfig] configFileName: %s.") % config);

  // populate tree structure pt
  using boost::property_tree::ptree;
  ptree pt;
  read_xml(config, pt);

  BOOST_FOREACH(ptree::value_type const& dataNode, pt.get_child("OpenPose")) {
          if (dataNode.first.compare("pose_configuration") == 0) {

            wrapperStructPose.netInputSize = {(int) dataNode.second.get<int>("net_input_length"),
                                              (int) dataNode.second.get<int>("net_input_height")};
            wrapperStructPose.outputSize = {dataNode.second.get<int>("output_length"),
                                            dataNode.second.get<int>("output_height")};
            wrapperStructPose.keypointScaleMode = op::flagsToScaleMode(dataNode.second.get<int>("keypoint_scale"));
            wrapperStructPose.gpuNumber = dataNode.second.get<int>("gpu_number");
            wrapperStructPose.gpuNumberStart = dataNode.second.get<int>("gpu_number_start");
            wrapperStructPose.scalesNumber = dataNode.second.get<int>("scales_number");
            wrapperStructPose.scaleGap = dataNode.second.get<float>("scale_gap");
            wrapperStructPose.renderMode = op::flagsToRenderMode(dataNode.second.get<int>("render_mode"), 0);
            wrapperStructPose.poseModel = op::flagsToPoseModel(op::String(dataNode.second.get<std::string>("pose_model")));
            wrapperStructPose.blendOriginalFrame = dataNode.second.get<bool>("blend_original_frame");
            wrapperStructPose.alphaKeypoint = dataNode.second.get<float>("alpha_keypoint");
            wrapperStructPose.alphaHeatMap = dataNode.second.get<float>("alpha_heat_map");
            wrapperStructPose.defaultPartToRender = dataNode.second.get<int>("default_part_to_render");
            

            wrapperStructPose.modelFolder = op::String(dataNode.second.get<std::string>("model_folder"));


            wrapperStructPose.heatMapTypes = op::flagsToHeatMaps(dataNode.second.get<bool>("heat_map_parts"),
                                                                 dataNode.second.get<bool>("heat_map_background"),
                                                                 dataNode.second.get<bool>("heat_map_PAFs"));


            int poseHeatMapScale = dataNode.second.get<int>("heat_map_scale");
            op::checkBool(poseHeatMapScale >= 0 && poseHeatMapScale <= 2, "Non valid `heatmaps_scale`.", __LINE__,
                      __FUNCTION__,
                      __FILE__);
            const auto heatMapScale = (poseHeatMapScale == 0 ? op::ScaleMode::PlusMinusOne
                                                             : (poseHeatMapScale == 1 ? op::ScaleMode::ZeroToOne
                                                                                      : op::ScaleMode::UnsignedChar));

            wrapperStructPose.heatMapScaleMode = heatMapScale;
            wrapperStructPose.renderThreshold = dataNode.second.get<float>("render_threshold");

          } else if (dataNode.first.compare("face_configuration") == 0) {
            wrapperStructFace.enable = dataNode.second.get<bool>("face_enabled");
            wrapperStructFace.netInputSize = {dataNode.second.get<int>("net_face_input_length"),
                                              dataNode.second.get<int>("net_face_input_height")};
            wrapperStructFace.renderMode = op::flagsToRenderMode(dataNode.second.get<int>("face_render_mode"), 0);
            wrapperStructFace.alphaKeypoint = dataNode.second.get<float>("face_alpha_keypoint");
            wrapperStructFace.alphaHeatMap = dataNode.second.get<float>("face_alpha_heat_map");
            wrapperStructFace.renderThreshold = dataNode.second.get<float>("face_render_threshold");
          } else if (dataNode.first.compare("hand_configuration") == 0) {
            wrapperStructHand.enable = dataNode.second.get<bool>("hand_enabled");
            wrapperStructHand.netInputSize = {dataNode.second.get<int>("net_hand_input_length"),
                                              dataNode.second.get<int>("net_hand_input_height")};
            wrapperStructHand.scalesNumber = dataNode.second.get<int>("hand_scales_number");
            wrapperStructHand.scaleRange = dataNode.second.get<float>("hand_scale_range");

            // member variable does not exist: there seems to be a "detector" option, but no tracking option
            // wrapperStructHand.tracking = dataNode.second.get<bool>("hand_tracking");

            wrapperStructHand.renderMode = op::flagsToRenderMode(dataNode.second.get<int>("hand_render_mode"), 0);
            wrapperStructHand.alphaKeypoint = dataNode.second.get<float>("hand_alpha_keypoint");
            wrapperStructHand.alphaHeatMap = dataNode.second.get<float>("hand_alpha_heat_map");
            wrapperStructHand.renderThreshold = dataNode.second.get<float>("hand_render_threshold");
          } else if (dataNode.first.compare("output_configuration") == 0) {
            /**
             * Both of the commented out functions have issues regarding the member not existing. 
             * TODO: Fix
             */
            //wrapperStructOutput.displayMode = static_cast<op::DisplayMode>(dataNode.second.get<int>("display_gui"));
            
            wrapperStructOutput.verbose = dataNode.second.get<bool>("gui_verbose");

            // member variable does not exist
            // wrapperStructOutput.fullScreen = dataNode.second.get<bool>("full_screen");
                        wrapperStructOutput.writeKeypoint = op::String(dataNode.second.get<std::string>("write_keypoint"));

            wrapperStructOutput.writeKeypointFormat = op::stringToDataFormat(
                    dataNode.second.get<std::string>("write_keypoint_format"));
            
            wrapperStructOutput.writeJson = op::String(dataNode.second.get<std::string>("write_json"));

            wrapperStructOutput.writeCocoJson = op::String(dataNode.second.get<std::string>("write_coco_json"));
            wrapperStructOutput.writeImages = op::String(dataNode.second.get<std::string>("write_images"));
            wrapperStructOutput.writeImagesFormat = op::String(dataNode.second.get<std::string>("write_image_format"));
            wrapperStructOutput.writeVideo = op::String(dataNode.second.get<std::string>("write_video"));
            wrapperStructOutput.writeHeatMaps = op::String(dataNode.second.get<std::string>("write_heat_maps"));
            wrapperStructOutput.writeHeatMapsFormat = op::String(dataNode.second.get<std::string>("write_heat_maps_format"));
          }

        }
}


void OpPoseDetector::handleCaptureNotification(CaptureNotification::ConstPtr notification) {
  //get captured frames info
  cv::Mat currFrame = notification->captureData->frame;

  // Push frame to op::Datum
  op::Matrix opCurrFrame = OP_CV2OPMAT(currFrame);
  // auto datumToProcess = createDatum(currFrame);

  // if (datumToProcess == nullptr) {
  //   LOG4CXX_ERROR(logger, "Did not get any data to process.");
  //   return;
  // }

  if (opCurrFrame.empty()) {
    LOG4CXX_ERROR(logger, "Did not get any data to process.");
    return;
  }

  const op::Matrix imageToProcess = OP_CV2OPCONSTMAT(currFrame);

  // auto successfullyEmplaced = opWrapper->waitAndEmplace(datumToProcess);
  auto successfullyEmplaced = opWrapper->emplaceAndPop(imageToProcess);
  if (!successfullyEmplaced) {
    LOG4CXX_ERROR(logger, "Failed to push/pop current frame");
  }
  LOG4CXX_DEBUG(logger, "just popped");
  LOG4CXX_DEBUG(logger, successfullyEmplaced->at(0)->poseKeypoints.toString());

  // // Pop frame
  // std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> datumProcessed;

  // LOG4CXX_DEBUG(logger, "OpPose waiting for frame to be processed.");
  // if (successfullyEmplaced && opWrapper->waitAndPop(datumProcessed)) {
  //   LOG4CXX_DEBUG(logger, "OpPose completed iteration.");
  // } else {
  //   LOG4CXX_ERROR(logger, "OpenPose failed failed to process frame.");
  //   return;
  // }

  // create new memory objects from detection results and send detection notifications
  MemoryObject::VecPtr newDetectionResults = createMemoryObjects(notification->captureData, successfullyEmplaced);
  sendDetectionNotifications(newDetectionResults);
  // draw results
  if (getDisplayFlag()) {
    LOG4CXX_DEBUG(logger, "Drawing results.");
    op::Matrix opDisplayFrame = OP_CV2OPMAT(displayFrame);
    // datumProcessed->at(0)->cvOutputData.copyTo(opDisplayFrame);
    successfullyEmplaced->at(0)->cvOutputData.copyTo(opDisplayFrame);

    diarc::Display::displayFrame(OP_OP2CVMAT(opDisplayFrame), getDisplayName());
  }
}

MemoryObject::VecPtr OpPoseDetector::createMemoryObjects(const CaptureData::ConstPtr &capture,
                                                         const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> &datumsPtr) {
  MemoryObject::VecPtr newDetectionResults(new MemoryObject::Vec());

  if (datumsPtr != nullptr && !datumsPtr->empty()) {
    // get pose information
    const auto &poseKeypoints = datumsPtr->at(0)->poseKeypoints;
    // LOG4CXX_DEBUG(logger, datumsPtr->at(0)->poseKeypoints.toString());

    // get current descriptors to process
    DescriptorsByTypeConstPtr types = getTypes();
    DescriptorsByType::const_iterator types_iter;
    PredicateHelper::Set::const_iterator descriptors_itr;
    std::tr1::unordered_map<std::string, std::vector<std::vector<std::pair<int, std::string>>>>::const_iterator jointMap_itr;
    std::vector<std::vector<std::pair<int, std::string>>>::const_iterator jointOptions_itr;

    // iterate through all typeIds
    for (types_iter = types->begin(); types_iter != types->end(); ++types_iter) {
      LOG4CXX_DEBUG(logger, boost::format("type: %lld.") % types_iter->first);
      long long typeId = types_iter->first;

      // iterate through all detected people
      for (auto person = 0; person < poseKeypoints.getSize(0); person++) {
        int maxJointIndex = poseKeypoints.getSize(1);

        // iterate through all descriptors for a typeId
        for (descriptors_itr = types_iter->second.begin();
             descriptors_itr != types_iter->second.end(); ++descriptors_itr) {
          const std::string &descriptor = descriptors_itr->getName();

          jointMap_itr = jointMap.find(descriptor);
          if (jointMap_itr == jointMap.end()) {
            LOG4CXX_ERROR(logger, "No joint in joint map for: " + descriptor);
            continue;
          }

          // iterate through all joint options for specified descriptor
          // e.g., "head" descriptor might have (0,nose) AND (1,neck) joint pairs as an option
          // while "hand" might (4,rightHand) OR (7,leftHand) options
          for (jointOptions_itr = jointMap_itr->second.begin();
               jointOptions_itr != jointMap_itr->second.end(); ++jointOptions_itr) {
            MemoryObjectPtr rootMO = createMemoryObject(capture, typeId, *descriptors_itr,
                                                        poseKeypoints, person, *jointOptions_itr);

            // finally add root MO to detection results
            newDetectionResults->push_back(rootMO);
          }
        }
      }
    }


    // debugging output for all detected people
    if (logger->isDebugEnabled() && false) {
      LOG4CXX_DEBUG(logger, "Keypoints:");
      // Accesing each element of the keypoints
      const auto &poseKeypoints = datumsPtr->at(0)->poseKeypoints;
      LOG4CXX_DEBUG(logger, "Person pose keypoints:");
      for (auto person = 0; person < poseKeypoints.getSize(0); person++) {
        LOG4CXX_DEBUG(logger, "Person " + std::to_string(person) + " (x, y, score):");
        for (auto bodyPart = 0; bodyPart < poseKeypoints.getSize(1); bodyPart++) {
          auto x = poseKeypoints[{person, bodyPart, 0}];
          auto y = poseKeypoints[{person, bodyPart, 1}];
          auto score = poseKeypoints[{person, bodyPart, 2}];
          if (logger->isDebugEnabled()) {
            std::string valueToPrint = std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(score);
            LOG4CXX_DEBUG(logger, valueToPrint);
          }
        }
      }
    }

  } else {
    LOG4CXX_DEBUG(logger, "Nullptr or empty datumsPtr found.");
  }

  LOG4CXX_DEBUG(logger, boost::format("MOs detected: %d.") % newDetectionResults->size());
  return newDetectionResults;
}


MemoryObjectPtr OpPoseDetector::createMemoryObject(const CaptureData::ConstPtr &capture,
                                                   const long long &typeId,
                                                   const PredicateHelper &descriptor,
                                                   const op::Array<float> &poseKeypoints,
                                                   const int &person,
                                                   const std::vector<std::pair<int, std::string>> &jointPairs) {

  int maxJointIndex = poseKeypoints.getSize(1);

  // body part bounding box size in pixels
  auto width = 20;
  auto widthHalf = width / 2;
  auto scoreThresh = 0.1f;

  const std::string &variable = descriptor.getArg(0);

  // iterate through all joint pairs for specified descriptor (i.e., head descriptor might have (0,nose) and (1,neck) joint pairs)
  MemoryObject::VecPtr children(new MemoryObject::Vec());
  cv::Rect rootMOBB; // root MO bounding box

  std::vector<std::pair<int, std::string>>::const_iterator jointPairs_itr;
  for (jointPairs_itr = jointPairs.begin(); jointPairs_itr != jointPairs.end(); ++jointPairs_itr) {

    int jointIndex = jointPairs_itr->first;
    const std::string &jointName = jointPairs_itr->second;

    if (jointIndex < maxJointIndex && poseKeypoints[{person, jointIndex, 2}] > scoreThresh) {
      // create new memory object for each body part with a size of width x width pixels
      auto x = poseKeypoints[{person, jointIndex, 0}];
      auto y = poseKeypoints[{person, jointIndex, 1}];
      cv::Rect bb = cv::Rect_<int>(x - widthHalf, y - widthHalf, width, width);

      // TODO: should this be the same variable as the top-level descriptor?
      MemoryObjectPtr jointMO(new MemoryObject(typeId, variable, capture, bb));
      jointMO->addValidationResult(poseKeypoints[{person, jointIndex, 2}],
                                   PredicateHelper(jointName + "(" + variable + ")"));

      // calculate root MO bounding box based on all children's bounding boxes
      const cv::Rect &otherBB = jointMO->getDetectionMask()->getBoundingBox();
      if (children->empty()) {
        rootMOBB = otherBB;
      } else {
        // expand rootMOBB to encompass both rootMOBB and otherBB
        rootMOBB.x = (otherBB.x < rootMOBB.x) ? otherBB.x : rootMOBB.x;
        rootMOBB.y = (otherBB.y < rootMOBB.y) ? otherBB.y : rootMOBB.y;
        rootMOBB.width = (otherBB.br().x > rootMOBB.br().x) ? (otherBB.br().x - rootMOBB.x) : rootMOBB.width;
        rootMOBB.height = (otherBB.br().y > rootMOBB.br().y) ? (otherBB.br().y - rootMOBB.y) : rootMOBB.height;
      }

      // add joint MO to children list
      children->push_back(jointMO);

    }
  }

  // create separate scene graph for each descriptor for each typeId
  MemoryObjectPtr rootMO;
  if (children->empty()) {
    LOG4CXX_WARN(logger, boost::format("[createMemoryObject] no joints found for type: %s.") % descriptor.toString());
  //} else if (children->size() == 1) {
  //  // if only one child, no reason to have separate top-level MO (e.g., hand and rightHand)
  //  rootMO = childnew->at(0);
  } else {
    rootMO = MemoryObjectPtr(new MemoryObject(typeId, variable, capture, rootMOBB));

    // add all children to root
    MemoryObject::Vec::const_iterator children_itr;
    for (children_itr = children->begin(); children_itr != children->end(); ++children_itr) {
      rootMO->addChild(*children_itr);
    }
  }

  if (rootMO) {
    float confidence = 0.9f;  // TODO: calculate this by combining joint scores
    rootMO->addValidationResult(confidence, descriptor);
  }

  return rootMO;
}

std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> OpPoseDetector::createDatum(cv::Mat currFrame) {
  // Create new datum
  // auto datumsPtr = std::make_shared<std::vector<op::Datum>>();
  auto datumsPtr = std::make_shared<std::vector<std::shared_ptr<op::Datum>>>();
  
  datumsPtr->emplace_back();

  auto &datum = datumsPtr->at(0);

  // Fill datum
  op::Matrix opMat = OP_CV2OPCONSTMAT(currFrame);

  // Note: cvInputData is of type op::Matrix, THIS LINE IS BROKEN
  datum->cvInputData = opMat; //OP_CV2OPMAT(currFrame);

  LOG4CXX_ERROR(logger, "checking if empty");
  // If empty frame -> return nullptr
  if (datum->cvInputData.empty()) {
    datumsPtr = nullptr;
  }

  return datumsPtr;
}