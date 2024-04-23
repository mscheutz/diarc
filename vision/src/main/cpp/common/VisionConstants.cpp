/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "VisionConstants.hpp"


unsigned int VisionConstants::imgWidth = 320;       //set to defaults - can be reset at runtime
unsigned int VisionConstants::imgHeight = 240;
bool VisionConstants::isStereo = false;

//consts
const unsigned int VisionConstants::maxFrameHistoryLength;
const float VisionConstants::mPerSquare = 0.108;// 0.023;     //size of grid square in camera calibration grid. used to get real world units in depth maps

const std::string VisionConstants::dataPath = "com/vision/native/data/";    //relative from ade root

const unsigned long int VisionConstants::tolerance = 10;
const float VisionConstants::saliency_confidence_level = 0.6;
