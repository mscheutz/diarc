/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.reflection;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Serializable;
import java.io.Writer;
import java.util.ArrayList;
import java.util.Collections;

/**
 * @date 9/16/2010
 * @author evankrause
 */
/**
 * this class is currently used to keep track of performance requirements,
 * (hardware, memory, etc..), and runtime statistics (looptime, etc..) for
 * vision related tasks. this is currently tailored to vision related tasks, but
 * could potentially be generalized to use in other DIARC components as a means of
 * keeping track of what a server can introspect on.
 */
//TODO: replace synchronized methods with fine grained locks for each get/set method pair
public class TaskPerformanceInformation implements Serializable {

    private String name = null;
    private long startTime;
    private ArrayList<String> hardware;        //requirments (e.g. "gpu", "camera") - not yet used
    private long memory;                    //requirments (in MB) - not yet used
    private long looptime;                  //avg ms required to complete one iteration of task
    private long maxLooptime;               //during lifetime of program. not reset after "start"
    private long minLooptime;               //during lifetime of program. not reset after "start"
    private long timeOfLastClientUse;       //time client last requested results from this task (from currentTimeMillis)
    private long timeOfLastResult;          //time (from currentTimeMillis) this task last produced desired result (e.g. detected/tracked an object)
    private long timeOfLastIteration;          //time (from currentTimeMillis) this task last finished an iteration
    private long numProcessedObjectsSinceLastStart; //same as totalProcessedObjects, but gets reset after every restart
    private ArrayList<Long> timeProcessedObjectsSinceLastStart; //last "n" timestamps of numProcessedObjectsSinceLastStart
    private long numIterationsSinceLastStart;       //same as totalProcessedObjects, but gets reset after every restart
    private long totalProcessedObjects;     //for detector: num of detnumProcessedObjectsSinceLastStartected objects (same object in different frames increases count).
    //for tracker: num of unique objects tracked (basically counts objects added to tracker).
    private long totalIterations;           //total number of frames processed
    //helper fields
    private long looptimeAccum = 0;             //keeps accumulated "window" of looptimes from which looptime periodically gets updated
    private int looptimeWindowCount = 0;        //how many looptime accumulations so far
    private final int looptimeWindowSize = 10;  //how many looptimes to use in looptime calculation
    private int timeProcessedIndx = 0;
    private final int timeProcessedVecLen = 5;
    //print stats - currently only used for printing timing info - could be expanded for future needs
    private static boolean printStats = false;
    private String statsBaseFilename = "visionStats";
    private String statsMessage = new String();

    public TaskPerformanceInformation(final String taskName) {
        name = taskName;
        startTime = System.currentTimeMillis();
        hardware = new ArrayList<String>();
        memory = -1;
        looptime = -1;
        maxLooptime = -1;
        minLooptime = -1;
        timeOfLastClientUse = startTime;
        timeOfLastResult = startTime;
        numProcessedObjectsSinceLastStart = 0;
        timeProcessedObjectsSinceLastStart = new ArrayList<Long>(timeProcessedVecLen);
        numIterationsSinceLastStart = 0;
        totalProcessedObjects = 0;
        totalIterations = 0;
    }

    @Override
    public String toString() {
        return "startTime = " + startTime + ". hardware = " + hardware + ". memory: = " + memory + ". looptime = " + looptime + ". timeOfLastClientUse = " + timeOfLastClientUse + ". timeOfLastResult = " + timeOfLastResult;
    }

    //turn the automatic printing of stats to file on/off at task completion (ie, finish())
    public static synchronized void setPrintStats(boolean printFlag) {
        printStats = printFlag;
    }

    public synchronized ArrayList<String> getHardware() {
        //make and return deep copy
        ArrayList<String> hardwareCopy = new ArrayList<String>(hardware);
        Collections.copy(hardwareCopy, hardware);
        return hardwareCopy;
    }

    public synchronized long getLooptime() {
        return looptime;
    }

    public synchronized long getMemory() {
        return memory;
    }

    public synchronized long getTimeOfLastClientUse() {
        return timeOfLastClientUse;
    }

    public synchronized long getTimeOfLastResult() {
        return timeOfLastResult;
    }

    public synchronized long getNumProcessedObjects() {
        return numProcessedObjectsSinceLastStart;
    }

    public synchronized long getTotalProcessedObjects() {
        return totalProcessedObjects;
    }

    public synchronized long getNumIterationsSinceLastStart() {
        return numIterationsSinceLastStart;
    }

    public synchronized void setHardware(ArrayList<String> hardware) {
        this.hardware = hardware;
    }

    public synchronized void setTimeOfLastClientUse(long timeOfLastClientUse) {
        //Thread.dumpStack();
        this.timeOfLastClientUse = timeOfLastClientUse;
    }

    private synchronized void setLooptime(long looptime) {
        looptimeAccum += looptime;
        ++looptimeWindowCount;
        if (looptimeWindowCount >= looptimeWindowSize) {
            long avgLooptime = looptimeAccum / looptimeWindowCount;
            looptimeAccum = 0;
            looptimeWindowCount = 0;

            //update data
            this.looptime = avgLooptime;
            if (looptime > maxLooptime) {
                maxLooptime = looptime;
            }
            if (minLooptime < 0 || looptime < minLooptime) {
                minLooptime = looptime;  //((minLooptime < 0) bc minLooptime is init to -1.
            }
        }
    }

    private synchronized void setMemory(long memory) {
        this.memory = memory;
    }

    private synchronized void setTimeOfLastResult(long timeOfLastResult) {
        this.timeOfLastResult = timeOfLastResult;
    }

    //this should be called every time the specified task is starting
    //to reinitialise any necesary data
    public synchronized void start() {
        startTime = System.currentTimeMillis();
        timeOfLastClientUse = startTime;
        timeOfLastResult = startTime;
        timeOfLastIteration = startTime;    //should this be set to -1 ?
        numProcessedObjectsSinceLastStart = 0;
        numIterationsSinceLastStart = 0;
        timeProcessedObjectsSinceLastStart = new ArrayList<Long>(timeProcessedVecLen);
        timeProcessedIndx = 0;
    }

    //convenience function - updates processing results, not (client) use results
    public synchronized void update(long looptime, boolean taskSuccess, int numProcessedObjects) {
        long currTime = System.currentTimeMillis();

        setLooptime(looptime);
        if (taskSuccess) {
            setTimeOfLastResult(currTime);
        }

        numProcessedObjectsSinceLastStart += numProcessedObjects;
        ++numIterationsSinceLastStart;

        timeProcessedObjectsSinceLastStart.add(timeProcessedIndx, currTime);
        timeProcessedIndx = timeProcessedIndx++ % timeProcessedVecLen;

        totalProcessedObjects += numProcessedObjects;
        ++totalIterations;
    }

    public synchronized void updateExperimentStats(boolean dataProcessed) {
        long currTime = System.currentTimeMillis();

        //add stats to string to be printed
        if (printStats) {
            if (dataProcessed) {
                long iterationtime = currTime - timeOfLastIteration;
                statsMessage += timeOfLastIteration + " " + currTime + " " + iterationtime + "\n";
            } else {
                statsMessage += "busy loop\n";
            }
            timeOfLastIteration = currTime;
            //statsMessage += "iteration time (ms): " + iterationtime + " task success: " + taskSuccess + "\n";
        }
    }

//this should be called every time the specified task is stoping
//to finisih up any incomplete calculations (ie. looptime calculations, etc..)
    public synchronized void finish() {
        if (looptimeWindowCount > 0) {
            long avgLooptime = looptimeAccum / looptimeWindowCount;
            looptimeAccum = 0;
            looptimeWindowCount = 0;
            this.looptime = avgLooptime;
            if (looptime > maxLooptime) {
                maxLooptime = looptime;
            }
            if (minLooptime < 0 || looptime < minLooptime) {
                minLooptime = looptime;  //((minLooptime < 0) bc minLooptime is init to -1.
            }
        }

        //Thread.dumpStack();

        //print stats to file
        if (printStats) {
            try {
                String statsFilename = statsBaseFilename + "_" + name + ".txt";
                FileWriter fstream = new FileWriter(statsFilename, true);
                BufferedWriter out = new BufferedWriter(fstream);
                out.write("<start>,<end>,<time>\n");
                out.write(statsMessage);
                out.close();

                statsMessage = "";
            } catch (Exception e) {//Catch exception if any
                System.err.println("Error: " + e.getMessage());
            }
        }
    }

    //print current info to file
    public synchronized boolean print(String filenamePath) {
        StringBuilder out = new StringBuilder();
        out.append("currentTime \t" + System.currentTimeMillis() + "\n");
        out.append("startTime \t" + startTime + "\n");
        out.append("hardware \t" + hardware + "\n");
        out.append("memory \t" + memory + "\n");
        out.append("looptime \t" + looptime + "\n");
        out.append("minLooptime \t" + minLooptime + "\n");
        out.append("maxLooptime \t" + maxLooptime + "\n");
        out.append("timeOfLastClientUse \t" + timeOfLastClientUse + "\n");
        out.append("timeOfLastResult \t" + timeOfLastResult + "\n");
        out.append("totalProcessedObjects \t" + totalProcessedObjects + "\n");
        out.append("totalIterations \t" + totalIterations + "\n");

        Writer fileWriter;
        try {
            fileWriter = new BufferedWriter(new FileWriter(filenamePath, true));
            fileWriter.append(out.toString());
            fileWriter.close();
        } catch (IOException e) {
            return false;
        }

        return true;
    }
    //EAK: a most likely bad generalization idea
//    public Object get(String fieldName) {
//        Class c = this.getClass();
//        try {
//            Field fld = c.getField(fieldName);
//            return fld.get(this);
//        } catch (NoSuchFieldException e) {
//            System.err.print(e);
//        } catch (IllegalAccessException e) {
//            System.err.print(e);
//        }
//        return null;
//    }
//
//    public boolean set(String fieldName, Object value) {
//        Class c = this.getClass();
//        try {
//            Field fld = c.getField(fieldName);
//            fld.set(this, value);
//            return true;
//        } catch (NoSuchFieldException e) {
//            System.err.print(e);
//        } catch (IllegalAccessException e) {
//            System.err.print(e);
//        }
//        return false;
//    }
}
