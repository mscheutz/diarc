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
import java.util.ArrayList;

/**
 *
 * @author evankrause
 */
public class TaskAnalysisPolicyDefault implements TaskAnalysisPolicy {
    //constraint data
    long maxTimeSinceLastClientUse = 10000; //milliseconds

    public Collection<Result> analyse(TaskPerformanceInformation performanceInfo) {
        ArrayList<Result> results = new ArrayList();
        long currTime = System.currentTimeMillis();

        if ((currTime - performanceInfo.getTimeOfLastClientUse()) > maxTimeSinceLastClientUse) {
            results.add(Result.QUIT);
        } else {
            results.add(Result.CONTINUE);
        }

        return results;
    }
}
