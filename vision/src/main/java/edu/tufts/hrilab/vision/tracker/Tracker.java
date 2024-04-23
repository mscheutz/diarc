/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.tracker;

import edu.tufts.hrilab.vision.Vision;
import edu.tufts.hrilab.vision.imgproc.ImageProcessor;
import edu.tufts.hrilab.vision.detector.swig.NativeDetector.DetectorType;
import edu.tufts.hrilab.vision.tracker.swig.NativeTracker;
import edu.tufts.hrilab.vision.tracker.swig.NativeTracker.TrackerType;
import edu.tufts.hrilab.vision.visionproc.VisionProcess;

/**
 *
 * @author evankrause
 */
public class Tracker extends VisionProcess<TrackerType, TrackerDetail, NativeTracker> {
  //static blocks are called in order of appearance in file
  //this must be called before any JNI methods can be used
  //(ie. must appear before other static blocks that use any JNI methods)

  static {
    System.loadLibrary("tracker");
  }

  Tracker(final TrackerDetail trackerDetail) {
    super(trackerDetail,
            NativeTracker.get(trackerDetail.getType(), id_generator.getNext(), trackerDetail.getWidth(), trackerDetail.getHeight()));
  }

  @Override
  public synchronized String toString() {
    return "Tracker: " + detail.toString() + ". ID: " + processId;
  }

  @Override
  protected void perform() {
    final String trackerName = this.toString();
    System.out.println("starting new " + trackerName + " tracker task.");

    //start image processing dependecies
    for (ImageProcessor dependency : dependencies.values()) {
      dependency.start(this);
    }

    long looptime = 0;

    //"single value variables" (i.e., not arrays).
    //passed to native code by ref -> they have to be "pointers" in Java
    boolean[] objectTracked = {false};
    int[] numNewlyTrackedObjects = {0};

    try {
      //(re)start introspection for selected tracker
      performanceInfo.start();

      while (runFlag) {
        //TODO: if tracker type is changed for moType, we may need to convert all currently trackedMOs
        //to new sub-MemoryObjectType

        //do object tracking
        processor.perform();

        //TODO: get tracker stats
//        processor.getStats(objectTracked, numNewlyTrackedObjects);

        //update fps info/UI
        looptime = Vision.FPSpanel.registerThreadCompletion(trackerName, true);

        //update introspection data
        performanceInfo.update(looptime, objectTracked[0], numNewlyTrackedObjects[0]);
        performanceInfo.updateExperimentStats(true);

        Thread.sleep(1);
      }
    } catch (InterruptedException e) {
    } finally {
    }
  }

  public boolean hasIterationCompleted(long typeId) {
    return processor.hasIterationCompleted(typeId);
  }

  /**
   * This has been overriden to expose the method to the "com.vision.tracker"
   * package. The super class is in a different package, therefore its
   * "protected" members become private to this package.
   */
  @Override
  protected synchronized void terminate() {
    super.terminate();
  }
}
