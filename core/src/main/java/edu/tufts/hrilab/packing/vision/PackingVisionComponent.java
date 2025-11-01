/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2025
 */

package edu.tufts.hrilab.packing.vision;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.diarc.DiarcComponent;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.bytedeco.javacv.*;
import org.bytedeco.javacv.Frame;

import javax.imageio.ImageIO;

import static org.bytedeco.opencv.global.opencv_videoio.CAP_PROP_AUTO_EXPOSURE;
import static org.opencv.videoio.Videoio.CAP_PROP_EXPOSURE;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;


public class PackingVisionComponent extends DiarcComponent {


  private OpenCVFrameGrabber frameGrabber;
  private Java2DFrameConverter frameConverter;
  int cameraId = -1;
  int imageWidth = 640;
  int imageHeight = 480;
  Frame imageFrame;

  @Override
  public void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("camera")) {
      cameraId = Integer.parseInt(cmdLine.getOptionValue("camera"));
    }
  }

  @Override
  public java.util.List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("camera").hasArg().build());
    return options;
  }

  public void init() {
    if (cameraId != -1) {
      setupCapture();
    }
  }

  private boolean setupCapture() {
    frameGrabber = new OpenCVFrameGrabber(cameraId);
//    frameGrabber.setOption(CAP_PROP_AUTO_EXPOSURE, 1);
//    frameGrabber.setOption(CAP_PROP_EXPOSURE, 0.7);
//    frameGrabber.setOption(CAP_PROP_EXPOSURE, 100);
    frameGrabber.setImageWidth(imageWidth);
    frameGrabber.setImageHeight(imageHeight);
    frameConverter = new Java2DFrameConverter();
    try {
      frameGrabber.start();
      frameGrabber.setTriggerMode(true);
      log.info("[init] format for device images is: %s".formatted(frameGrabber.getFormat()));
    } catch (FrameGrabber.Exception e) {
      log.error("[init] error starting frame grabber on camera ID: " + cameraId, e);
      return false;
    }
    return true;
  }

  @TRADEService
  public byte[] getFrame() {
    byte[] data = null;
    try {
      frameGrabber.trigger();
      imageFrame = frameGrabber.grab();
      frameGrabber.trigger();

      BufferedImage image = frameConverter.convert(imageFrame);
      ByteArrayOutputStream baos = new ByteArrayOutputStream();
      ImageIO.write(image, "png", baos);
      data = baos.toByteArray();
    } catch (IOException e) {
      log.error("[getFrame] error grabbing frame: ", e);
    }
    return data;
  }

  @TRADEService
  public Dimension getImageSize() {
    return new Dimension(frameGrabber.getImageWidth(), frameGrabber.getImageHeight());
  }

  @TRADEService
  public boolean setCamera(int cameraId) {
    if (frameGrabber != null) {
      try {
        frameGrabber.stop();
        frameGrabber.close();
      } catch (FrameGrabber.Exception e) {
        log.error("[setCamera] exception stopping and closing frame grabber ", e);
      }
    }
    this.cameraId = cameraId;
    return setupCapture();
  }

  @TRADEService
  //todo: this is assuming modern linux as our platform.
  public List<PackingCameraInfo> getAvailableCameras() {
    boolean done = false;
    int i = 0;
    List<PackingCameraInfo> toReturn = new ArrayList<>();
    while (!done) {
      try {
        // Build the process
        ProcessBuilder builder = new ProcessBuilder("bash", "-c", "udevadm info --name=/dev/video" + ++i);
        builder.redirectErrorStream(true); // merge stdout and stderr

        // Start the process
        Process process = builder.start();

        // Read stdout from the process
        boolean valid = false;
        String model = "";
        try (BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()))) {
          String line = "";
          while ((line = reader.readLine()) != null) {
            if (line.contains("ID_V4L_CAPABILITIES") && line.contains("capture")) {
              valid = true;
            }
            if (line.contains("ID_MODEL=")) {
              List<String> results = List.of(line.split("ID_MODEL="));
              if (results.size() > 1) {
                model =  results.get(1);
              }
            }
          }
        }
        if (valid) {
          toReturn.add(new PackingCameraInfo(model, i));
        }

        // Wait for the process to finish and get the exit code
        int exitCode = process.waitFor();
        if  (exitCode != 0) {
          done = true;
        }
        System.out.println("Command exited with code: " + exitCode);

      } catch (Exception e) {
        log.error("[getAvailableCameras] error in system calls to get camera info", e);
      }
    }
    log.info("[results list] " + toReturn);
    return toReturn;
  }

  public void shutDownComponent() {
    try {
      frameGrabber.stop();
      frameGrabber.close();
    } catch (FrameGrabber.Exception e) {
      log.error("[shutDownComponent] error stopping openCV frame grabber on camera ID: " + cameraId + "during shutdown");
    }
  }
}