/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.learn;

import edu.tufts.hrilab.vision.imgproc.ImageProcessor;
import edu.tufts.hrilab.vision.learn.swig.NativeLearner;
import edu.tufts.hrilab.vision.learn.swig.NativeLearner.LearnerType;
import edu.tufts.hrilab.vision.visionproc.VisionProcess;
import edu.tufts.hrilab.vision.Vision;

/**
 * An instantiated Learner responsible for managing a single learner thread,
 * which performs object learning via a SWIG wrapped, native ObjectLearner.
 *
 * @author evankrause
 */
public class Learner extends VisionProcess<LearnerType, LearnerDetail, NativeLearner> {
  //static blocks are called in order of appearance in file
  //this must be called before any JNI methods can be used
  //(ie. must appear before other static blocks that use any JNI methods)

  static {
    System.loadLibrary("learn");
  }

  Learner(final LearnerDetail detectorDetail) {
    super(detectorDetail,
            NativeLearner.get(detectorDetail.getType(), id_generator.getNext(), detectorDetail.getWidth(), detectorDetail.getHeight()));
  }

  @Override
  public synchronized String toString() {
    return "Learner: " + detail.toString() + ". ID: " + processId;
  }

  @Override
  protected void perform() {
    final String learnerName = this.toString();
    log.debug("starting new " + learnerName + " learner task.");

    //start image processing dependecies
    for (ImageProcessor dependency : dependencies.values()) {
      dependency.start(this);
    }

    long looptime = 0;

    //"single value variables" (i.e., not arrays).
    //passed to native code by ref -> they have to be "pointers" in Java
    boolean[] objectDetected = {false};
    int[] numDetectedObjects = {0};

    try {
      //(re)start introspection for selected detector
      performanceInfo.start();

      while (runFlag) {

        //do object detection
        processor.perform();

        //TODO: get detection results
        //processor.getStats(objectDetected, numDetectedObjects);

//        //update fps info/UI
//        //TODO: fix looptime - always 33 on first pass - probably shouldn't use UI to calculate looptime!
//        //same fix in Tracker and ImageProcessor too!!
//        log.trace("updating fps panel.");
//        looptime = Vision.FPSpanel.registerThreadCompletion(learnerName, true);
//
//        //update introspection data
//        log.trace("updating performance stats.");
//        performanceInfo.update(looptime, objectDetected[0], numDetectedObjects[0]);
//        looptime = 0;
//        log.trace("updating experiment stats.");
//        performanceInfo.updateExperimentStats(true);

        //if only want to run one iteration (that actually does some work)
        if (singleIteration) {
          log.trace("ran single iteration. breaking.");
          break;
        }

        Thread.sleep(1);
      }

    } catch (InterruptedException e) {
    } finally {
    }
  }

  /**
   * This has been overridden to expose the method to the "com.vision.detector"
   * package. The super class is in a different package, therefore its
   * "protected" members become private to this package.
   */
  @Override
  protected synchronized void terminate() {
    super.terminate();
  }
}
