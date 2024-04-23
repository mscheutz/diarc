/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef OPPOSEDETECTOR_HPP
#define OPPOSEDETECTOR_HPP

#include "ObjectDetector.hpp"
#include <openpose/headers.hpp>

class OpPoseDetector : public ObjectDetector {
public:
  typedef boost::shared_ptr<OpPoseDetector> Ptr;
  typedef boost::shared_ptr<const OpPoseDetector> ConstPtr;

  OpPoseDetector(const long long &processorId, const int imgWidth, const int imgHeight);

  ~OpPoseDetector();

  //set which haar profile to load on initHaarCascades(). needs to be full path.
  virtual void loadConfig(const std::string &configFile);

  virtual void init();

  virtual void cleanup();


protected:
  virtual void handleCaptureNotification(CaptureNotification::ConstPtr notification);

private:

  std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> createDatum(cv::Mat currFrame);

  ade::stm::MemoryObject::VecPtr createMemoryObjects(const CaptureData::ConstPtr &capture,
                                                     const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> &datumsPtr);

  ade::stm::MemoryObjectPtr createMemoryObject(const CaptureData::ConstPtr &capture,
                                               const long long &typeId,
                                               const PredicateHelper &descriptor,
                                               const op::Array<float> &poseKeypoints,
                                               const int &person,
                                               const std::vector<std::pair<int, std::string>> &jointPairs);

  void loadOpenPoseConfig(const std::string &config);

  // op::Wrapper<std::vector<op::Datum>> *opWrapper;
  op::Wrapper *opWrapper = new op::Wrapper{op::ThreadManagerMode::Asynchronous};
  op::WrapperStructPose wrapperStructPose;
  op::WrapperStructFace wrapperStructFace;
  op::WrapperStructHand wrapperStructHand;
  op::WrapperStructOutput wrapperStructOutput;
  std::tr1::unordered_map<std::string, std::vector<std::vector<std::pair<int, std::string>>>> jointMap;

};

#endif  //OPPOSEDETECTOR_HPP