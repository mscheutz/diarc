/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.learn;

import edu.tufts.hrilab.vision.imgproc.swig.ImageProcessorType;
import edu.tufts.hrilab.vision.learn.swig.NativeLearner.LearnerType;
import edu.tufts.hrilab.vision.visionproc.VisionProcessDetail;

/**
 *
 * @author evankrause
 */
public class LearnerDetail extends VisionProcessDetail<LearnerType> {

  private ImageProcessorType validatorType;

  LearnerDetail(final LearnerType detectorType, final int imgWidth, final int imgHeight, ImageProcessorType validatorType) {
    super(detectorType, imgWidth, imgHeight);
    this.validatorType = validatorType;
  }

  ImageProcessorType getImageProcessorType() {
    return validatorType;
  }

}
