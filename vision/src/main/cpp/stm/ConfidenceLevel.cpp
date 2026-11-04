/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "ConfidenceLevel.hpp"

ConfidenceLevel::ConfidenceLevel()
: decayPerMilSec(0.0001),
level(0.95) {
    //init timestamp
    gettimeofday(&tv, NULL);
    lasttime = (unsigned) (tv.tv_sec * 1000) + (unsigned) (tv.tv_usec / 1000.0); //convert to milliseconds
}

ConfidenceLevel::ConfidenceLevel(const float& conf_level, const float& decayPerSec)
: decayPerMilSec(decayPerSec / 1000),
level(conf_level) {
    //init timestamp
    gettimeofday(&tv, NULL);
    lasttime = (unsigned) (tv.tv_sec * 1000) + (unsigned) (tv.tv_usec / 1000.0); //convert to milliseconds
}

ConfidenceLevel::~ConfidenceLevel() {

}

void ConfidenceLevel::decay() {
    // get time duration between last update
    gettimeofday(&tv, NULL);
    unsigned long long currenttime = (unsigned) (tv.tv_sec * 1000) + (unsigned) (tv.tv_usec / 1000.0); //convert to milliseconds
    timediff = currenttime - lasttime;

    // decay level based on time duration
    level -= timediff * decayPerMilSec;

    // update lasttime
    lasttime = currenttime;

}

void ConfidenceLevel::setLevel(const float& newLevel) {
    //update timestamp
    gettimeofday(&tv, NULL);
    lasttime = (unsigned) (tv.tv_sec * 1000) + (unsigned) (tv.tv_usec / 1000.0); //convert to milliseconds

    //bounds check and set level
    if (newLevel >= 1.0) {
        level = 1.0;
    } else if (newLevel <= 0.0) {
        level = 0.0;
    } else {
        level = newLevel;
    }
}

void ConfidenceLevel::setDecayRate(const float& decayRate) {
    decayPerMilSec = decayRate;
}

float ConfidenceLevel::getLevel() const {
    return level;
}

float ConfidenceLevel::getDecayRate() const {
    return decayPerMilSec;
}