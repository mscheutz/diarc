/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision;

import java.io.IOException;
import java.awt.Dimension;
import java.net.URL;
import java.util.ArrayList;
import java.util.List;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.util.resource.Resources;
import edu.tufts.hrilab.vision.capture.Camera;
import edu.tufts.hrilab.vision.gui.VideoDisplay;
import edu.tufts.hrilab.vision.util.CompressionUtil;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

public class VideoSendComponent extends DiarcComponent {

  private boolean display = false;
  /**
   * Default path to vision resources (configs).
   */
  private String defaultConfigPath = "config/edu/tufts/hrilab/vision";
  private String captureConfig = "default.xml";
  private String calibConfig = null;
  //capture object
  private Camera camera;
  //image display
  private VideoDisplay videoDisplay;

  /**
   * VideoSendComponent constructor.
   */
  public VideoSendComponent() {
    super();
  }

  @Override
  protected void init() {

    //set paths for config files
    captureConfig = createFilepath(defaultConfigPath, "capture", captureConfig);
    calibConfig = createFilepath(defaultConfigPath, "calibration", calibConfig);

    // init vision and start camera(s)
    camera = new Camera(false, captureConfig, calibConfig);
    camera.start();

    if (display) {
      videoDisplay = new VideoDisplay(getImageSize());
    }

    shouldRunExecutionLoop = true;
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
  }

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

  @Override
  protected void shutdownComponent() {
    camera.stopAndWait();
  }

  /**
   * Helper method to add default path if only filename had been specified.
   *
   * @param defaultDir
   * @param subDir
   * @param filename
   * @return
   */
  private String createFilepath(String defaultDir, String subDir, String filename) {
    if (filename == null || filename.isEmpty()) {
      return filename;
    }
    URL url = Resources.getResource(defaultDir + "/" + subDir, filename);
    if (url == null) {
      log.warn("Filepath resource not found: {}", filename);
      return null;
    } else {
      return url.getPath();
    }
  }
}
