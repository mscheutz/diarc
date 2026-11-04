/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.tracker;

import edu.tufts.hrilab.vision.detector.swig.NativeDetector.DetectorType;
import edu.tufts.hrilab.vision.tracker.swig.NativeTracker.TrackerType;
import edu.tufts.hrilab.vision.visionproc.VisionProcessDetail;
import java.util.ArrayList;

/**
 *
 * @author evankrause
 */

//Thread-safe

public class TrackerDetail extends VisionProcessDetail<TrackerType> {
		private String configFile;
    
    TrackerDetail(final TrackerType trackerType, final int imgWidth, final int imgHeight) {
        super(trackerType, imgWidth, imgHeight);
    }
		
		synchronized void setConfig(String filename) {
        configFile = filename;
    }

    synchronized String getConfig() {
        return configFile;
    }
}
