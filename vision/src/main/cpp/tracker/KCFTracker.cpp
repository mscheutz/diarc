/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "KCFTracker.hpp"

using namespace diarc::stm;

KCFTracker::KCFTracker(const long long &processorId, const int imgWidth, const int imgHeight)
        : OpenCVTracker("KCF", processorId, imgWidth, imgHeight) {
  visionProcessName = "KCFTracker";
  logger = log4cxx::Logger::getLogger("diarc.tracker.KCFTracker");
}

KCFTracker::~KCFTracker() {
}

void KCFTracker::loadConfig(const std::string &config) {
}