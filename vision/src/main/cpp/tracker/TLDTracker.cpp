/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "TLDTracker.hpp"

using namespace diarc::stm;

TLDTracker::TLDTracker(const long long &processorId, const int imgWidth, const int imgHeight)
        : OpenCVTracker("TLD", processorId, imgWidth, imgHeight) {
  visionProcessName = "TLDTracker";
  logger = log4cxx::Logger::getLogger("diarc.tracker.TLDTracker");
}

TLDTracker::~TLDTracker() {
}

void TLDTracker::loadConfig(const std::string &config) {
}