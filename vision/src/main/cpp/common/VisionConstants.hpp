/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef VISIONCONSTANTS_HPP
#define VISIONCONSTANTS_HPP

//#include "capture/firewire/FireWireHelper.hpp"      //for CAM_TYPE - is this necessary?

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string>

#include <pthread.h>

#define PI 3.141592653

class VisionConstants {
public:
    static unsigned int imgWidth;
    static unsigned int imgHeight;
    static bool isStereo;

    //TODO: adjust according to slowest process.
    //max number of frames to keep at one time in objectTracker's frame queue
    //currently should only increase this during runtime!!
    static const unsigned int maxFrameHistoryLength = 100;

    //TODO: there's an identical RAD_PER_SECTORvariable in ShortTermMemory.java
    //so they really need to be unified somehow ??
    static const double RAD_PER_SECTOR; // the number of radians in a sector
    static const float mPerSquare;// = 0.023;//75;  //size of grid square in camera calibration grid. used to get real world units in depth maps

    static const std::string dataPath;

    //notifications
    static const unsigned long int tolerance;
    //saliency confidence level
    static const float saliency_confidence_level;
};
#endif
