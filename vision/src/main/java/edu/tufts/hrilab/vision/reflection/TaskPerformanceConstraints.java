
/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.reflection;

import java.util.ArrayList;

/**
 *
 * @author evankrause
 */
public class TaskPerformanceConstraints {
    private ArrayList<String> availableHardware;   //available hardware for this task (e.g. "gpu", "camera")
    private long maxMemory;                     //max memory allowed by task (in MB)
    private long maxLooptime;                   //max time allowed to complete one iteration of task (miliseconds)
    private long maxTimeSinceLastClientUse;     //max time allowed to pass since client last requested results from this task (miliseconds)
    private long maxTimeSinceLastResult;        //max time allowed to pass since this task last produced desired result (e.g. detected/tracked on object)

    public TaskPerformanceConstraints() {
        this.availableHardware = new ArrayList<String>();  //TODO: how should this really be initialized?
        this.maxMemory = -1;                            //TODO: how should this really be initialized?
        this.maxLooptime = 250;                     //250ms = 4fps
        this.maxTimeSinceLastClientUse = 10000;     //milliseconds
        this.maxTimeSinceLastResult = 10000;        //milliseconds
    }

    public TaskPerformanceConstraints(ArrayList<String> availableHardware, long maxMemory, long maxLooptime, long maxTimeSinceLastClientUse, long maxTimeSinceLastResult) {
        this.availableHardware = new ArrayList<String>(availableHardware); //shallow copy
        this.maxMemory = maxMemory;
        this.maxLooptime = maxLooptime;
        this.maxTimeSinceLastClientUse = maxTimeSinceLastClientUse;
        this.maxTimeSinceLastResult = maxTimeSinceLastResult;
    }

    public TaskPerformanceConstraints(TaskPerformanceConstraints constraints) {
        availableHardware = new ArrayList<String>(constraints.getHardware());  //shallow copy, but Strings are immutable
        maxMemory = constraints.getMaxMemory();
        maxLooptime = constraints.getMaxLooptime();
        maxTimeSinceLastClientUse = constraints.getMaxTimeSinceLastClientUse();
        maxTimeSinceLastResult = constraints.getMaxTimeSinceLastResult();
    }

    public synchronized ArrayList<String> getHardware() {
        ArrayList<String> availableHardwareCopy = new ArrayList<String>(availableHardware);
        return availableHardwareCopy;
    }

    public synchronized void setHardware(ArrayList<String> hardware) {
        this.availableHardware = hardware;
    }

    public synchronized long getMaxLooptime() {
        return maxLooptime;
    }

    public synchronized void setMaxLooptime(long maxLooptime) {
        this.maxLooptime = maxLooptime;
    }

    public synchronized long getMaxMemory() {
        return maxMemory;
    }

    public synchronized void setMaxMemory(long maxMemory) {
        this.maxMemory = maxMemory;
    }

    public synchronized long getMaxTimeSinceLastClientUse() {
        return maxTimeSinceLastClientUse;
    }

    public synchronized void setMaxTimeSinceLastClientUse(long maxTimeSinceLastClientUse) {
        this.maxTimeSinceLastClientUse = maxTimeSinceLastClientUse;
    }

    public synchronized long getMaxTimeSinceLastResult() {
        return maxTimeSinceLastResult;
    }

    public synchronized void setMaxTimeSinceLastResult(long maxTimeSinceLastResult) {
        this.maxTimeSinceLastResult = maxTimeSinceLastResult;
    }

}
