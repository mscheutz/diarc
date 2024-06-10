/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.capture;

import edu.tufts.hrilab.vision.Vision;
import edu.tufts.hrilab.vision.capture.calibration.swig.CameraCalibration;
import edu.tufts.hrilab.vision.capture.swig.CAM_MODE;
import edu.tufts.hrilab.vision.capture.swig.COLOR_SPACE;
import edu.tufts.hrilab.vision.capture.swig.Capture;
import edu.tufts.hrilab.vision.capture.swig.CaptureModule;

import java.awt.Dimension;
import java.awt.image.BufferedImage;
import java.awt.image.DataBuffer;
import java.awt.image.Raster;
import java.awt.image.WritableRaster;
import java.util.ArrayList;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.locks.*;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import javax.vecmath.Matrix4d;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Camera {
  //static blocks are called in order of appearance in file
  //this must be called before any JNI methods can be used
  //(ie. must appear before other static blocks that use any JNI methods)

  static {
    System.loadLibrary("capture");
  }

  final ReentrantLock captureDataLock = new ReentrantLock(true);
  private Capture captureDevice = null;  //native capture device (used via swig)
  private ExecutorService executor = Executors.newSingleThreadExecutor();
  private Future runFuture;       //used to resume/stop and check status of task
  private Future cleanupFuture;   //TODO: use this to resume/stop and check status of cleanup task
  private Matrix4d currVisionTransform = new Matrix4d();
  private boolean displayFlag;
  //used for passing image data from native code
  private byte[] javaImgData;
  private byte[] javaDispData;
  private byte[] javaDepthData;
  private WritableRaster raster;
  private BufferedImage buffImage;
  private String captureConfig;
  private Dimension imageSize;
  private final Lock pauseLock = new ReentrantLock();
  private final Condition pauseCondition = pauseLock.newCondition();
  private boolean pauseFlag = false;
  protected Logger log = LoggerFactory.getLogger(getClass());

  public Camera(boolean display,
                String captureConfig,
                String calibConfig) {
    super();

    this.displayFlag = display;
    this.captureConfig = captureConfig;

    //initialize capture pipeline
    captureDevice = Capture.createCaptureDevice(captureConfig);

    // set local fields based on native configuration
    this.imageSize = new Dimension();
    this.imageSize.height = captureDevice.getImageHeight();
    this.imageSize.width = captureDevice.getImageWidth();

    // init camera calibration
    if (calibConfig != null) {
      if (calibConfig.endsWith("calib2")) {
        CameraCalibration.getInstance().calibrationStereoLoad(calibConfig);
      } else if (calibConfig.endsWith("calib")) {
        CameraCalibration.getInstance().calibrationSingleCamLoad(0, calibConfig);
      }
    }

    //for passing image data though JNI. ie: passBackImageArray, passBackBlobLikeImageArray
    javaImgData = new byte[(imageSize.width) * (imageSize.height) * 3];
    javaDispData = new byte[(imageSize.width) * (imageSize.height) * 4];
    javaDepthData = new byte[(imageSize.width) * (imageSize.height) * 4];
    raster = Raster.createInterleavedRaster(DataBuffer.TYPE_BYTE, imageSize.width, imageSize.height, 3, null);
    buffImage = new BufferedImage(imageSize.width, imageSize.height, BufferedImage.TYPE_3BYTE_BGR);
  }

  /**
   * Start capture thread. Does nothing if already running.
   */
  public synchronized void start() {
    //don't start if already running
    if (runFuture != null && !runFuture.isCancelled()) {
      return;
    }

    //capture and display thread
    final String threadName = "capture";

    //add entry to FPS GUI
    Vision.FPSpanel.addEntry(threadName);

    runFuture = executor.submit(new Runnable() {
      @Override
      public void run() {
        //if thread has been stopped, the captureDevice was deleted
        //which is mainly to ensure the native destructor is called on program exit
        if (captureDevice == null) {
          captureDevice = Capture.createCaptureDevice(captureConfig);
        }

        try {
          while (true) {

            // check if thread should be paused
            try {
              pauseLock.lock();
              while (pauseFlag) {
                pauseCondition.await();
              }
            } finally {
              pauseLock.unlock();
            }

            double[] transform;
            try {
              //lock while data is copied
              captureDataLock.lock();
              //convert Matrix4d to double[] to pass to native
              transform = matrix4dToArray(currVisionTransform);
            } finally {
              captureDataLock.unlock();
            }

            log.debug("capture called");
            boolean success = captureDevice.capture(transform);
            log.debug("capture completed: " + success);
            Vision.FPSpanel.registerThreadCompletion(threadName, success);

            //TODO: fps regulation here instead of in individual Capture___.cpp classes
            Thread.sleep(1);    //significantly reduces CPU usage
          }

        } catch (InterruptedException e) {
          log.debug("capture thread caught InterruptedException");
        } finally {
          log.debug("capture thread in finally");
          if (captureDataLock.isHeldByCurrentThread()) {
            captureDataLock.unlock();
          }

          //remove entry from FPS GUI
          Vision.FPSpanel.removeEntry(threadName);

          log.debug("capture thread finally finished");
        }

        log.debug("capture thread finished");
      }
    });
  }

  /**
   * Pause the currently running capture thread.
   */
  public void pause() {
    // check if thread should be paused
    try {
      pauseLock.lock();
      if (!pauseFlag) {
        pauseFlag = true;
      }
    } finally {
      pauseLock.unlock();
    }
  }

  /**
   * Resume the paused capture thread.
   */
  public void resume() {
    // check if thread should be paused
    try {
      pauseLock.lock();
      if (pauseFlag) {
        pauseFlag = false;
        pauseCondition.signal();
      }
    } finally {
      pauseLock.unlock();
    }
  }

  /**
   * Stop camera thread and return without waiting for completion. Also performs
   * cleanup.
   */
//  public synchronized void stop() {
//    if (runFuture == null) {    //never started
//      return;
//    }
//
//    runFuture.cancel(true);
//
//    //do any memory clean up here!! (added to executor queue)
//    if (cleanupFuture == null || cleanupFuture.isDone()) {
//      cleanup();
//    }
//  }

  /**
   * Stop camera thread, do cleanup, and wait until both have completed before
   * returning.
   */
  public synchronized void stopAndWait() {
    if (runFuture == null || runFuture.isDone()) {    //never started or isn't running
      return;
    }

    log.debug("[stopAndWait] cancelling capture runFuture");
    runFuture.cancel(true);

    log.debug("[stopAndWait] interrupting capture thread");
    captureDevice.interruptWait();

    while (!runFuture.isDone()) {
      log.debug(String.format("[stopAndWait] waiting for %s to finish.", toString()));
//      captureDevice.interruptWait();
      try {
        //wait for runFuture to finish current job
        runFuture.get(1000, TimeUnit.MILLISECONDS);
      } catch (InterruptedException | ExecutionException | TimeoutException e) {
        log.debug("[stopAndWait] still waiting...");
      }
    }
    log.debug("[stopHelper] runFuture finished.");

    //do any memory clean up here!! (added to executor queue)
    if (cleanupFuture == null || cleanupFuture.isDone()) {
      cleanup();
    }

    //wait till cleanup has completed
    while (!cleanupFuture.isDone()) {
      try {
        Thread.sleep(10);
      } catch (InterruptedException e) {
      }
    }
  }

  //FIXME
  private void cleanup() {
    cleanupFuture = executor.submit(new Runnable() {
      public void run() {
        captureDevice.delete();
        captureDevice = null;
      }
    });
  }

  /**
   * Is capture running.
   *
   * @return
   */
  public synchronized boolean isRunning() {
    return (runFuture == null ? false : !runFuture.isDone());
  }

  /**
   * Write RGB frame to file.
   *
   * @param filename
   */
  public synchronized void writeFrame(String filename) {
    captureDevice.writeFrame(filename);
  }

  /**
   * Write PCD (point cloud data) frame to file. Only works if capturing or
   * generating point cloud data.
   *
   * @param filename
   */
  public synchronized void writePointCloud(String filename) {
    captureDevice.writePointCloud(filename);
  }

  /**
   * Start recording an AVI.
   *
   * @param filename
   */
  public synchronized void startAviWrite(String filename) {
    captureDevice.startAviWrite(filename);
  }

  /**
   * Finish recording the AVI and save it to file.
   */
  public synchronized void stopAviWrite() {
    captureDevice.stopAviWrite();
  }

  /**
   * Start recording an ONI (RGB-D).
   *
   * @param filename
   */
  public synchronized void startOniWrite(String filename) {
    captureDevice.startOniWrite(filename);
  }

  /**
   * Finish recording the ONI (RGB-D) and save it to file.
   */
  public synchronized void stopOniWrite() {
    captureDevice.stopOniWrite();
  }

  public synchronized void displayCapturedImages(boolean display) {
    if (!displayFlag) {
      return;
    }

    captureDevice.setImageDisplayFlag(display);
  }

  public synchronized void displayDepthData(boolean display) {
    if (!displayFlag) {
      return;
    }

    captureDevice.setDepthDisplayFlag(display);
  }

  /**
   * Pass most up to date data to camera so that it can be passed to native
   * camera code and synchronized (in time) with camera data.
   *
   * @param currTransform - current transform from vision to robot coordinate
   *                      frame
   */
  public void passAdditionalDataToCamera(Matrix4d currTransform) {
    try {
      //lock capture thread while updating its info
      captureDataLock.lock();
      currVisionTransform = currTransform;
    } finally {

      captureDataLock.unlock();
    }
  }

  private double[] matrix4dToArray(final Matrix4d transform) {
    double[] doubleArray = new double[16];
    doubleArray[0] = transform.m00;
    doubleArray[1] = transform.m01;
    doubleArray[2] = transform.m02;
    doubleArray[3] = transform.m03;
    doubleArray[4] = transform.m10;
    doubleArray[5] = transform.m11;
    doubleArray[6] = transform.m12;
    doubleArray[7] = transform.m13;
    doubleArray[8] = transform.m20;
    doubleArray[9] = transform.m21;
    doubleArray[10] = transform.m22;
    doubleArray[11] = transform.m23;
    doubleArray[12] = transform.m30;
    doubleArray[13] = transform.m31;
    doubleArray[14] = transform.m32;
    doubleArray[15] = transform.m33;

    return doubleArray;
  }

  //==========================================================================
  // START: Get image data from native side
  //==========================================================================
  final Object nativeImageDataLock = new Object();

  /**
   * This gets the captred camera frame specified by frameNum.
   *
   * @param blurAmount - how much to blur image. (0 = none)
   * @return BufferedImage of captured camera frame.
   */
  public BufferedImage getJNIFrame(int blurAmount) {
    synchronized (nativeImageDataLock) {
      //get data from JNI call
      CaptureModule.passBackImageArray(javaImgData, blurAmount);

      raster.setDataElements(0, 0, imageSize.width, imageSize.height, javaImgData);
      buffImage.setData(raster);

      return buffImage;
    }
  }

  /**
   * Get the most recent captured camera frame from native side.
   *
   * @return BufferedImage of captured camera frame.
   */
  public BufferedImage getJNIFrame() {
    synchronized (nativeImageDataLock) {
      return getJNIFrame(0);
    }
  }

  /**
   * Get raw image data from native side.
   *
   * @return byte[]
   */
  public byte[] getRawImageData() {
    synchronized (nativeImageDataLock) {
      //get data from JNI call
      CaptureModule.passBackImageArray(javaImgData, 0);
      return javaImgData;
    }
  }

  /**
   * Get raw image disparity data from native side.
   *
   * @return byte[]
   */
  public byte[] getRawDisparityData() {
    synchronized (nativeImageDataLock) {
      //get data from JNI call
      CaptureModule.passBackDisparityArray(javaDispData);
      return javaDispData;
    }
  }


  /**
   * Get raw image disparity data from native side.
   *
   * @return byte[]
   */
  public byte[] getRawDepthData() {
    synchronized (nativeImageDataLock) {
      //get data from JNI call
      CaptureModule.passBackDepthArray(javaDepthData);
      return javaDepthData;
    }
  }

  //==========================================================================
  // END: Get image data from native side
  //==========================================================================

  public CAM_MODE getCamMode() {
    return captureDevice.getCameraMode();
  }

  public int getImageWidth() {
    return captureDevice.getImageWidth();
  }

  public int getImageHeight() {
    return captureDevice.getImageHeight();
  }

  public boolean hasStereo() {
    return captureDevice.hasStereo();
  }

  public boolean hasDepth() {
    return captureDevice.hasDepth();
  }

  public boolean getUndistort() {
    return captureDevice.getUndistortFlag();
  }

  /**
   * set whether or not captures frames should be undistorted
   *
   * @param undistort
   */
  public void setUndistort(boolean undistort) {
    captureDevice.setUndistortFlag(undistort);
  }

  // for testing only
  public static void main(String args[]) {
    /*
     Camera cc = new Camera();
     Vector facevect;
     FaceData temp;
		
     cc.setBlobDetect(true); // turn blob detection on ... off by default
     // do not start the thread
     int i, j;
     for (i=0; i < 10000; i++)
     {
     facevect=(Vector)cc.faceMain(150,90,null,0);
     //System.out.println(temp.getPan());
     //for (j=0; j < facevect.size(); j++)
     //{
     //	temp = (FaceData)(facevect.elementAt(j));
     //	System.out.println(i+ " " + j + " " + temp.getPan());
     //	System.out.println(i+ " " + j + " " + temp.getX() + " " +temp.getY() + " " + temp.getArea() + " " + temp.getColor());
     //}
     System.out.println(facevect.size());
     }
     cc.cleanupFaceDetect();
     System.exit( 0 );
     */

    int[] test = null;
    System.out.println(test.length);

  }
}
