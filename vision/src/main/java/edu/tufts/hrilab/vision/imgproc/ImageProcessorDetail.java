/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.imgproc;

import edu.tufts.hrilab.vision.imgproc.swig.ImageProcessorType;
import edu.tufts.hrilab.vision.visionproc.VisionProcessDetail;

/**
 *
 * @author evankrause
 */

//Thread-safe

public class ImageProcessorDetail extends VisionProcessDetail<ImageProcessorType> {
    private final boolean is_stereo;

    ImageProcessorDetail(final ImageProcessorType procType, final int imgWidth, final int imgHeight, final boolean isStereo) {
        super(procType, imgWidth, imgHeight);
        is_stereo = isStereo;
    }

    boolean isStereo() {
        return is_stereo;
    }
}
