/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "ButtonDetector.hpp"

#include "capture/util/CaptureUtilities.hpp"
#include "common/notification/CaptureNotification.hpp"
#include "stm/util/StmUtilities.hpp"
#include "display/Display.hpp"

ButtonDetector::ButtonDetector(const long long &processorId, const int imgWidth, const int imgHeight)
        : NeuralDetector(processorId, imgWidth, imgHeight)
{
  visionProcessName = "ButtonDetector";
  logger = log4cxx::Logger::getLogger("ade.detector.ButtonDetector");
}

ButtonDetector::~ButtonDetector() {}

void ButtonDetector::handleCaptureNotification(CaptureNotification::ConstPtr notification) {
    std::vector<DetectedObject> objects = getDetections(notification);

    cv::Mat img = notification->captureData->frame;

    // display raw detections from onnx detector
    if (getDisplayFlag()) {
        cv::Mat imgCopy;
        img.copyTo(imgCopy);
        displayDetectedObjects(imgCopy, objects, "buttonDetectorAll");
    }

    removeDuplicates(objects, .5);
    rows = img.rows;
    cols = img.cols;

    bool emptyButtonsDetected = false;
    for (int i = 0; i < objects.size(); i++) {
        if (objects[i].name == "emptybutton") {
            emptyButtonsDetected = true;
        } else if (objects[i].name == "starfirstfloorbutton") { // combine 1stfloorbutton detections
            objects[i].name = "button_1";
        }
    }

    // determine whether search includes a specific floor button
    // and if not, whether were going up or down
    bool lookingForFloorButton = true;
    std::string desiredButtonName;
    TypesByDescriptorConstPtr descriptors = getDescriptors();
    TypesByDescriptor::const_iterator descriptor_iter_;
    // note: this assumes the only thing we will ever search for is up/down/#thfloorbutton/LL, and that
    //       only one button search is occuring at one time
    for (descriptor_iter_ = descriptors->begin(); descriptor_iter_ != descriptors->end(); ++descriptor_iter_) {
        desiredButtonName = descriptor_iter_->first.getName();
        if (desiredButtonName == "button_up" || desiredButtonName == "button_down") {
            lookingForFloorButton = false;
        } else if (desiredButtonName == "emptybutton") { // effectively just for testing
            lookingForFloorButton = false;
        }
    }

    if (lookingForFloorButton) {
        if (emptyButtonsDetected) { // match empty button to numbered button
            floorButtonWithEmptyButtons(objects, desiredButtonName);
        } else { // if detection is rectangular, take the right half
            for (int i = 0; i < objects.size(); i++) {
                cv::Rect rect = objects[i].rect;
                if ((double)rect.width / (double)rect.height > 1.4) {
                    int x, y, width, height;
                    x = rect.x + (rect.width - rect.height);
                    y = rect.y;
                    width = rect.height;
                    height = rect.height;
                    objects[i].rect = cv::Rect(x, y, width, height);
                }
            }
        }
    } else if (desiredButtonName == "button_up" || desiredButtonName == "button_down") { // looking for up/down button
        reclassifyUpDownButtons(objects, desiredButtonName, emptyButtonsDetected);
    }

    // populate memory objects
    TypesByDescriptor::const_iterator descriptor_iter;
    std::tr1::unordered_set<long long>::const_iterator typeIds_itr;
    ade::stm::MemoryObject::VecPtr newObjects(new ade::stm::MemoryObject::Vec());

    for (descriptor_iter = descriptors->begin(); descriptor_iter != descriptors->end(); ++descriptor_iter) {
        std::string currTypeName = descriptor_iter->first.getName();
        for (int i = 0; i < objects.size(); i++){
            DetectedObject currObj = objects[i];
            if (currObj.name == currTypeName) {
                for (typeIds_itr = descriptor_iter->second.begin();
                    typeIds_itr != descriptor_iter->second.end(); ++typeIds_itr) {
                    ade::stm::MemoryObject::Ptr newObject(
                    new ade::stm::MemoryObject(*typeIds_itr, descriptor_iter->first.getArg(0), notification->captureData, currObj.rect));
                    newObject->addValidationResult(currObj.confidence, descriptor_iter->first);
                    newObjects->push_back(newObject);
                }
            }
        }
    }

    sendDetectionNotifications(newObjects);

    // display detections
    if (getDisplayFlag()) {
        img.copyTo(displayFrame);

        ade::stm::MemoryObject::Vec::const_iterator newObjItr;
        for (newObjItr = newObjects->begin(); newObjItr != newObjects->end(); ++newObjItr) {
            const cv::Rect &objRect = (*newObjItr)->getDetectionMask()->getBoundingBox();
            cv::rectangle(displayFrame, cv::Point(objRect.x, objRect.y),
                            cv::Point(objRect.x + objRect.width, objRect.y + objRect.height),
                            CV_RGB(255, 0, 0), 2, 8, 0);
        }

        ade::Display::displayFrame(displayFrame, getDisplayName());
    }
}

// in case of empty buttons with labels on their right,
// match desired button to nearest right adjacent empty button
void ButtonDetector::floorButtonWithEmptyButtons(std::vector<DetectedObject> &objects, std::string desiredButtonName) {
    for (int i = 0; i < objects.size(); i++) {
        if (objects[i].name == desiredButtonName) {
            cv::Rect numberRect = objects[i].rect;
            double x1 = numberRect.x + (numberRect.width);
            double y1 = numberRect.y + (numberRect.height / 2);
            double minDist = 99999999;
            int indexOfMin = -1;
            for (int j = 0; j < objects.size(); j++) {
                if (i == j) continue;
                else if (objects[j].name != "emptybutton" && objects[j].name != "button_up" 
                         && objects[j].name != "button_down" && objects[j].name != "unknownbutton") continue;
                double x2 = objects[j].rect.x;
                if (x2 < x1) continue; // only want empty buttons on the right
                double y2 = objects[j].rect.y + (objects[j].rect.height / 2);
                if (abs((y2 - y1) / rows) > .06) continue; // not horizontally alligned
                if (x2 > (x1 + (numberRect.width * 2))) continue;
                double distance = sqrt((x2 - x1) * (x2 - x1) * (y2 - y1) * (y2 - y1));
                if (distance < minDist && distance != 0) {
                    minDist = distance;
                    indexOfMin = j;
                }
            }

            if (indexOfMin == -1) {
                LOG4CXX_DEBUG(logger, "could not find good empty button match for number, sending best guess");
                // manipulate number rect
                objects.erase(std::remove_if(objects.begin(), objects.end(), [desiredButtonName](DetectedObject obj){
                    return obj.name == desiredButtonName;
                }), objects.end());
                DetectedObject obj;
                obj.name = desiredButtonName;
                obj.rect = cv::Rect(x1 + (.3 * numberRect.width), y1 - (numberRect.height / 2), numberRect.width, numberRect.height);
                obj.confidence = .2; // because I am like 30% sure this is actually a button
                objects.push_back(obj);
                return; // something went wrong
            } 
            cv::Rect corRect = objects[indexOfMin].rect;
            float confidence = objects[indexOfMin].confidence;

            // remove empty button detection
            objects.erase(objects.begin()+indexOfMin);
        
            // remove all detections of desired button
            objects.erase(std::remove_if(objects.begin(), objects.end(), [desiredButtonName](DetectedObject obj){
                return obj.name == desiredButtonName;
            }), objects.end());

            // add back empty button detection as desired button
            DetectedObject correctButton;
            correctButton.name = desiredButtonName;
            correctButton.rect = corRect;
            correctButton.confidence = confidence;
            objects.push_back(correctButton);
            return;
        }
    }
}

// eliminate any unknowns, and verify known button detections using their relative position to other buttons
void ButtonDetector::reclassifyUpDownButtons(std::vector<DetectedObject> &objects, std::string desiredButtonName, bool emptyButtonsDetected) {
    // get centers of buttons (center is defined in .hpp)
    std::vector<center> centers;
    for (int i = 0; i < objects.size(); i++) {
        if (objects[i].name == "unknownbutton" || objects[i].name == "button_down" || objects[i].name == "button_up" || objects[i].name == "emptybutton") {
            center currCenter;
            currCenter.point = cv::Point(objects[i].rect.x + (int)(.5 * objects[i].rect.width), 
                        objects[i].rect.y + (int)(.5 * objects[i].rect.height));
            currCenter.index = i;
            centers.push_back(currCenter);
        }
    }

    // if only one button, it is either up or down, so mark it as desired button
    if (centers.size() == 1) {
        cv::Rect rect = objects[centers[0].index].rect;
        float confidence = objects[centers[0].index].confidence;
        objects.erase(objects.begin() + centers[0].index);
        DetectedObject button;
        button.rect = rect;
        button.confidence = confidence;
        button.name = desiredButtonName;
        objects.push_back(button);   
        return; 
    } else if (emptyButtonsDetected) { // otherwise, re-classify up/down/unknown buttons
        for (int i = 0; i < centers.size(); i++) {
            for (int j = 0; j < centers.size(); j++) {
                if (i == j) continue;
                // current vertical alignment threshold = one button width
                if (abs((double)(centers[i].point.x - centers[j].point.x) / cols) <= objects[centers[i].index].rect.width) {
                    if (objects[centers[i].index].rect.y < objects[centers[j].index].rect.y
                        && ade::stm::util::calculateBoundBoxOverlap(objects[centers[i].index].rect, objects[centers[j].index].rect) == 0) {
                        objects[centers[i].index].name = "button_up";
                    } else if (ade::stm::util::calculateBoundBoxOverlap(objects[centers[i].index].rect, objects[centers[j].index].rect) == 0) {
                        objects[centers[i].index].name = "button_down";
                    }
                }
            }
        }
    }
    // if you desire an up button, take the top right button otherwise take bottom left down button
    cv::Rect bestRect(0, cols, 0, 0);
    float bestConf;
    int bestIndex = -1;
    if (desiredButtonName == "button_up") { // take top right up button
        for (int i = 0; i < objects.size(); i++) {
            if (objects[i].name == "button_up") {
                if (objects[i].rect.x > bestRect.x && objects[i].rect.y < bestRect.y) {
                    bestIndex = i;
                    bestConf = objects[i].confidence;
                    bestRect = objects[i].rect;
                }
            }
        }
    } else if (desiredButtonName == "button_down") { // take bottom right down button
        bestRect.y = 0;
        for (int i = 0; i < objects.size(); i++) {
            if (objects[i].name == "button_down") {
                if (objects[i].rect.x > bestRect.x && objects[i].rect.y > bestRect.y) {
                    bestIndex = i;
                    bestConf = objects[i].confidence;
                    bestRect = objects[i].rect;
                }
            }
        }
    }
    if (bestIndex != -1) { // remove all detections of desired button and add back best candidate
        objects.erase(std::remove_if(objects.begin(), objects.end(), [desiredButtonName](DetectedObject obj){
            return obj.name == desiredButtonName;
        }), objects.end());
        DetectedObject bestUpDown;
        bestUpDown.name = desiredButtonName;
        bestUpDown.rect = bestRect;
        bestUpDown.confidence = bestConf;
        objects.push_back(bestUpDown);
    }

    // failure guard
    // since we know we are looking at a button panel, it would be suspicious 
    // if we saw empty buttons but did not classify them in the above section
    // note: real life testing required
    int numDesired = 0;
    for (auto obj : objects) {
        if (obj.name == desiredButtonName) {
            numDesired++;
        }
    }
    if (numDesired == 0) {
        for (int i = 0; i < objects.size(); i++) {
            if (objects[i].name == "emptybutton") {
                objects[i].name = desiredButtonName;
                break;
            }
        }
    }
}