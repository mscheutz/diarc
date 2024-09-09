/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "PersonDetector.hpp"

#include "capture/util/CaptureUtilities.hpp"
#include "common/notification/CaptureNotification.hpp"
#include "stm/util/StmUtilities.hpp"
#include "display/Display.hpp"

PersonDetector::PersonDetector(const long long &processorId, const int imgWidth, const int imgHeight)
        : NeuralDetector(processorId, imgWidth, imgHeight)
{
    visionProcessName = "PersonDetector";
    logger = log4cxx::Logger::getLogger("diarc.detector.PersonDetector");
}

PersonDetector::~PersonDetector() {}

void PersonDetector::handleCaptureNotification(CaptureNotification::ConstPtr notification) {
      // gets classIDs, bounding boxes, and confidences; Note that DetectedObject struct is defined in the .hpp
    std::vector<DetectedObject> objects = getDetections(notification);
    removeDuplicates(objects, .85);

    // converts to memory objects
    TypesByDescriptorConstPtr descriptors = getDescriptors();
    TypesByDescriptor::const_iterator descriptor_iter;
    std::tr1::unordered_set<long long>::const_iterator typeIds_itr;
    diarc::stm::MemoryObject::VecPtr newObjects(new diarc::stm::MemoryObject::Vec());

    for (descriptor_iter = descriptors->begin(); descriptor_iter != descriptors->end(); ++descriptor_iter) {
        std::string currTypeName = descriptor_iter->first.getName();
        for (int i = 0; i < objects.size(); i++){
            DetectedObject currObj = objects[i];
            if (currObj.name == currTypeName) {
                for (typeIds_itr = descriptor_iter->second.begin();
                    typeIds_itr != descriptor_iter->second.end(); ++typeIds_itr) {
                    diarc::stm::MemoryObject::Ptr newObject(
                    new diarc::stm::MemoryObject(*typeIds_itr, descriptor_iter->first.getArg(0), notification->captureData, currObj.rect));
                    newObject->addValidationResult(currObj.confidence, descriptor_iter->first);
                    newObjects->push_back(newObject);
                }
            }
        }
    }

    sendDetectionNotifications(newObjects);

    // display detections
    cv::Mat img = notification->captureData->frame;
    if (getDisplayFlag()) {
        img.copyTo(displayFrame);

        diarc::stm::MemoryObject::Vec::const_iterator newObjItr;
        for (newObjItr = newObjects->begin(); newObjItr != newObjects->end(); ++newObjItr) {
            const cv::Rect &objRect = (*newObjItr)->getDetectionMask()->getBoundingBox();
            cv::rectangle(displayFrame, cv::Point(objRect.x, objRect.y),
                            cv::Point(objRect.x + objRect.width, objRect.y + objRect.height),
                            CV_RGB(255, 0, 0), 2, 8, 0);
        }

        diarc::Display::displayFrame(displayFrame, getDisplayName());
    }
}
