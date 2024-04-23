/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision;

import java.io.IOException;
import java.awt.Dimension;
import java.util.ArrayList;
import java.util.List;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.vision.capture.Camera;
import edu.tufts.hrilab.vision.gui.VideoDisplay;
import edu.tufts.hrilab.vision.util.CompressionUtil;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

public class VideoSendComponent extends DiarcComponent {

  private boolean display;
  private String visionDir;
  private final String configDir;
  private String captureConfig;
  private String calibConfig;
  //capture object
  private Camera camera;
  //image display
  private VideoDisplay videoDisplay;

  // ********************************************************************
  // *** Local methods
  // ********************************************************************

  /**
   * VideoSendComponent constructor.
   */
  public VideoSendComponent() {
    super();

    // default file paths
    configDir = visionDir + "/native/data/";

    //set default path for config files if it hasn't been specified
    captureConfig = setFilenamePath(captureConfig, "capture");
    calibConfig = setFilenamePath(calibConfig, "calibration");

    // init vision and start camera(s)
    camera = new Camera(false, captureConfig, calibConfig);
    camera.start();

    // The server can also set its own loop time (the default is 10x/second).
    //this.setUpdateLoopTime(this, 1); // the amount is in milliseconds.

    if (display) {
      videoDisplay = new VideoDisplay(getImageSize());
    }

    shouldRunExecutionLoop = true;
  }

  @Override
  protected void init() {
    display = false;
    visionDir = "../vision";
    captureConfig = "default.xml";
    calibConfig = null;
  }

  @Override
  protected void executionLoop() {
    if (display && (videoDisplay != null)) {
      videoDisplay.displayImage(camera.getJNIFrame());
    }
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("display").desc("display window capture frames").build());
    options.add(Option.builder("capture").hasArg().argName("file").desc("file specifying camera capture configuration").build());
    options.add(Option.builder("calib").hasArg().argName("file").desc("stereo calibration parameters (must be running stereo cameras)").build());
    options.add(Option.builder("visiondir").hasArg().argName("relative path").desc("path from diarc to vision directory").build());
    return options;
  }

  /**
   * Parse additional command-line arguments
   */
  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("display")) {
      display = true;
    }
    if (cmdLine.hasOption("capture")) {
      captureConfig = cmdLine.getOptionValue("capture");
    }
    if (cmdLine.hasOption("calib")) {
      calibConfig = cmdLine.getOptionValue("calib");
    }
    if (cmdLine.hasOption("visiondir")) {
      visionDir = cmdLine.getOptionValue("visiondir");
    }
  }

  // ***********************************************************************
  // Methods available to remote objects via RMI
  // ***********************************************************************
  // Implement here whatever interface is defined in VideoSendComponent.java

  /**
   * An example of a remote call that fetches data from the server.
   * Note that this remote call is used both by other servers and the
   * server visualization -- e.g., there is no special interface
   * that needs to be implemented for GUI-related methods.
   *
   * @return server name, followed by an incrementing counter (initially
   * seeded to a random value, so that the output of the two servers
   * is different)
   */
  @TRADEService
  public byte[] getFrame() {
    byte[] imgRaw = camera.getRawImageData();
    byte[] imgComp = null;
    try {
      imgComp = CompressionUtil.compress(imgRaw);
    } catch (IOException e) {
      log.error("Error compressing rgb image.", e);
    }
    return imgComp;
  }

  @TRADEService
  public Dimension getImageSize() {
    return new Dimension(camera.getImageWidth(), camera.getImageHeight());
  }


  /**
   * Implements the local shutdown mechanism that derived classes need to
   * implement to cleanly shutdown
   */
  protected void localshutdown() {
    camera.stopAndWait();
  }

  /**
   * Helper method to add default path if only filename had been specified.
   *
   * @param filename
   * @return
   */
  private String setFilenamePath(String filename, String defaultDirName) {
    if (filename != null && filename.indexOf("/") == -1 && filename.indexOf("\\") == -1) {
      return String.format("%s%s/%s", configDir, defaultDirName, filename);
    } else {
      return filename;
    }
  }
}
