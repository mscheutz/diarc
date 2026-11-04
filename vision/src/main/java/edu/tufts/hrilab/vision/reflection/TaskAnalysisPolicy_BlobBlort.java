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
import edu.tufts.hrilab.vision.detector.swig.NativeDetector.DetectorType;
import java.util.Collection;
import java.util.ArrayList;

/**
 *
 * @author evankrause
 */
public class TaskAnalysisPolicy_BlobBlort implements TaskAnalysisPolicy {

    public Collection<Result> analyse(TaskPerformanceInformation performanceInfo) {
//    public Collection<Result> analyse(Detector detector, Tracker tracker) {
        ArrayList<Result> results = new ArrayList<>();
//        long currTime = System.currentTimeMillis();
//
//        if (detector.isType(DetectorType.BLOB) && detector.getPerformanceInfo().getNumProcessedObjects() > 0) {
//        //if (detector.isType(DetectorType.BLOB) && currTime - detector.getPerformanceInfo().getTimeOfLastResult() > 6000) {
//            //System.out.println("blob blort: " + detector.getPerformanceInfo().getNumProcessedObjects());
//            results.add(Result.PAUSE_MOTION);
//            results.add(Result.FIND_ALT);
//        } else if (detector.isType(DetectorType.SIFT3D)
//                && (currTime - detector.getPerformanceInfo().getTimeOfLastResult() > 15000)      //been n sec since last detection
//                && (currTime - tracker.getPerformanceInfo().getTimeOfLastResult() > 15000)) {    //been n seconds since last track
//            results.add(Result.FIND_ALT);
//        } else {
//            results.add(Result.CONTINUE);
//        }
//        
        return results;
    }
}
