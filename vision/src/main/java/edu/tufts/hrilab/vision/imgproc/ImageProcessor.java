/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.imgproc;

import edu.tufts.hrilab.vision.imgproc.swig.NativeImageProcessor;
import edu.tufts.hrilab.vision.imgproc.swig.ImageProcessorType;
import edu.tufts.hrilab.vision.visionproc.VisionProcess;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.vision.Vision;

/**
 *
 * @author evankrause
 */
public class ImageProcessor extends VisionProcess<ImageProcessorType, ImageProcessorDetail, NativeImageProcessor> {
  //static blocks are called in order of appearance in file
  //this must be called before any JNI methods can be used
  //(ie. must appear before other static blocks that use any JNI methods)

  static {
    System.loadLibrary("imgproc");
  }
  protected String name = "unknown";

  ImageProcessor(ImageProcessorDetail newTypeDetail, Term descriptor) {
    super(newTypeDetail,
            NativeImageProcessor.get(newTypeDetail.getType(), id_generator.getNext(), newTypeDetail.getWidth(), newTypeDetail.getHeight(), newTypeDetail.isStereo()));

    if (descriptor != null) {
      name = descriptor.getName();
    }
  }

  @Override
  public synchronized String toString() {
    return "ImageProcessor: " + detail.toString() + ". ID: " + processId;
  }

  @Override
  protected void perform() {
    final String threadName = this.toString();

    //start image processing dependecies
    for (ImageProcessor dependency : dependencies.values()) {
      dependency.start(this);
    }

    long looptime = 0;

    try {
      //(re)start introspection for image processor
      performanceInfo.start();

      while (runFlag) {
        log.trace("perform loop.");
        processor.perform();
        looptime = Vision.FPSpanel.registerThreadCompletion(threadName, true);

        //update introspection data
        log.trace("updating performance stats.");
        performanceInfo.update(looptime, true, 0);
        log.trace("updating experiment stats.");
        performanceInfo.updateExperimentStats(true);

        //if only want to run one iteration (that actually does some work)
        if (singleIteration) {
          log.trace("single iteration done. breaking.");
          break;
        }

        Thread.sleep(1);
      }
    } catch (InterruptedException e) {
    } finally {
    }
  }

  /**
   * This has been overriden to expose the method to the "com.vision.imgproc"
   * package. The super class is in a different package, therefore its
   * "protected" members become private to this package.
   */
  @Override
  protected void terminate() {
    super.terminate();
  }
}
