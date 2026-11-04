/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   KCFTracker.hpp
 * Author: Evan Krause
 *
 * Created on May 15 2023
 *
 */

#ifndef TLDTRACKER_HPP
#define	TLDTRACKER_HPP

#include "OpenCVTracker.hpp"

class TLDTracker : public OpenCVTracker
{
public:
  TLDTracker(const long long& processorId, const int imgWidth, const int imgHeight);
  ~TLDTracker();
  
  void loadConfig(const std::string& config);
};


#endif	/* TLDTRACKER_HPP */

