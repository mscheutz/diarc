/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision;


import java.util.ArrayList;

public class FramesPerSecondTracker {
    //times are in miliseconds
    private ArrayList<Long> myLooptimes = new ArrayList<Long> ();  //looptimes in miliseconds
    private long myLooptimeStart = System.currentTimeMillis();   //time that last registerThread Completion was called
    private long accumDuration = 0;     //accumulating duration. only want to keep track of time between actual work done. don't want to count workless cycles.
    private long lastLooptime;       //last calculated FPS
    private String stringId; //type + " " + stage
   


    FramesPerSecondTracker(String stringID) {
        stringId = stringID;
    }

    /**
     * 
     * @return time in milliseconds since last registerStageCompletion call
     */
    public synchronized long registerThreadCompletion(boolean dataProcessed) {
        long now = System.currentTimeMillis();
        long duration = now - myLooptimeStart;
        accumDuration += duration;
        if (dataProcessed) {
            lastLooptime = accumDuration;
            myLooptimes.add(accumDuration);
            accumDuration = 0;
        }
        myLooptimeStart = now;
        return duration;
    }

    /**
     * gets looptime in miliseconds
     * @return miliseconds
     */
    public synchronized long getLooptime() {
        return lastLooptime;
    }
    /**
     * uses data collected by registerThreadCompletion to calcuate avg fps over a window of time
     * also clears the data collected and starts a new window of time
     * @return FPS
     */
    public synchronized int calculateFPS() {
        long avg = Misc.Average(myLooptimes);
        myLooptimes.clear();

        if (avg == 0)
            return 0;
        else
            return (int)(1000L / avg);

    }

    @Override
    public String toString() {
        return stringId;
    }

}
