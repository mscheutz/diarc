/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.visionproc;

import edu.tufts.hrilab.vision.Vision;
import edu.tufts.hrilab.vision.common.swig.CommonModule;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public enum Requirement {
  AGILEGRASP,
  DEPTH,
  OPENPOSE,
  V4R,
  V4R_V0,
  ZBAR,
  OPENCV_TRACKING;

  protected static Logger log = LoggerFactory.getLogger(Requirement.class);

  /**
   * Checks the String based requirement by first calling toUpperCase
   * on the String to convert to enum, and then checking
   * thje requirement against vision's runtime configuration.
   * @param requirement
   * @return
   */
  public static boolean isRequirementSatisfied(String requirement) {
    Requirement req = null;
    try {
      req = Requirement.valueOf(requirement.toUpperCase());
    } catch (IllegalArgumentException e) {
      log.error(requirement + " is not a valid Requirement Enum.");
      return false;
    }

    switch (req) {
      case AGILEGRASP:
        return CommonModule.hasAgileGrasp();
      case DEPTH:
        return Vision.camera.hasDepth();
      case OPENPOSE:
        return CommonModule.hasOpenPose();
      case V4R:
        return CommonModule.hasV4R();
      case V4R_V0:
        return CommonModule.hasV4RV0();
      case ZBAR:
        return CommonModule.hasZBar();
      case OPENCV_TRACKING:
        return CommonModule.hasOpenCVTracking();
      default:
        log.error(requirement + " is not a valid Requirement Enum.");
        return false;
    }
  }
}
