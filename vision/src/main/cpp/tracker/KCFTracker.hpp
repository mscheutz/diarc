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

#ifndef KCFTRACKER_HPP
#define	KCFTRACKER_HPP

#include "OpenCVTracker.hpp"

class KCFTracker : public OpenCVTracker
{
public:
  KCFTracker(const long long& processorId, const int imgWidth, const int imgHeight);
  ~KCFTracker();
  
  void loadConfig(const std::string& config);
};


#endif	/* KCFTRACKER_HPP */

