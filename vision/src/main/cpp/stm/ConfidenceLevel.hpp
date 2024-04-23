/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef CONFIDENCELEVEL_HPP
#define CONFIDENCELEVEL_HPP

#include <stdio.h>
#include <sys/time.h>
/**
 * Representation of tracked object confidence (i.e., confidence that the
 * object is still there).
 * 
 * EAK: this currently uses time duration between updates to reduce confidence.
 */
class ConfidenceLevel {
public:
    ConfidenceLevel();
    ConfidenceLevel(const float& conf_level, const float& decayPerSec = 0.1f);
    ~ConfidenceLevel();
    
    void decay();       // decay confidence level based on time since last update (ie, setLevel or decay)
    bool isBelowThreshold() const;
    
    void setLevel(const float& newLevel);
    void setDecayRate(const float& decayRate);
    
    float getLevel() const;
    float getDecayRate() const;

private:
    float decayPerMilSec;
    float level;

    unsigned long long lasttime;
    unsigned long long timediff;
    struct timeval tv;
};

#endif  //CONFIDENCELEVEL_HPP