/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.tufts.hrilab.vision.reflection;

import edu.tufts.hrilab.vision.detector.Detector;
import edu.tufts.hrilab.vision.tracker.Tracker;
import java.util.Collection;

/**
 *
 * @author evankrause
 */
public interface TaskAnalysisPolicy {

    //possible results from analysis
    public enum Result {
        CONTINUE,   //constrainst met -> continue task
        FIND_ALT,   //constraints failed -> try to find alternative
        QUIT,        //constraints failed -> stop task
        PAUSE_MOTION  //request to temporarily robot motion
    }

    /**
     * Defines analysis rules for vision and can be defined per memory object type.
     * @param performanceInfo - current performance info contains introspection data
     * @return - Collection of Enum results
     */
    Collection<Result> analyse(TaskPerformanceInformation performanceInfo);
}
