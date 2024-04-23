/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.detector;

import edu.tufts.hrilab.vision.detector.swig.NativeDetector.DetectorType;
import edu.tufts.hrilab.vision.tracker.swig.NativeTracker.TrackerType;
import edu.tufts.hrilab.vision.visionproc.VisionProcessDetail;
import java.util.ArrayList;
import java.util.List;

/**
 *
 * @author evankrause
 */

//Thread-safe

public class DetectorDetail extends VisionProcessDetail<DetectorType> {
    private List<TrackerType> usableTrackerTypes = new ArrayList();   //this detector type only works with theses tracker types
    
    DetectorDetail(final DetectorType detectorType, final int imgWidth, final int imgHeight) {
        super(detectorType, imgWidth, imgHeight);
    }

    //available tracker methods
    synchronized void addUsableTracker(TrackerType usableTrackerType) {
        if (!usableTrackerTypes.contains(usableTrackerType)) {
            usableTrackerTypes.add(usableTrackerType);
        }
    }

    synchronized boolean canUseTracker(TrackerType trackerType) {
        return usableTrackerTypes.contains(trackerType);
    }

    synchronized List<TrackerType> geUsabletTrackers() {
        return usableTrackerTypes;
    }
    //end usable tracker methods
}
