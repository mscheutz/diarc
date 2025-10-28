package edu.tufts.hrilab.packing;

import com.google.gson.Gson;
import edu.tufts.hrilab.packing.gson.PackingVisionSettingsJson;
import edu.tufts.hrilab.slug.dialogue.ResponseGeneration;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.*;
import java.nio.file.Paths;
import java.util.List;
import java.util.stream.Collectors;

/**
 * process a camera image to determine the necessary robot action for pickup of the box
 */
public class RobotVisionProcessor {
  private final String settingsFile;
  public int defaultCamera;
  private double boxWidthMeters;
  private double boxLengthMeters;
  private final double cameraZDistanceMeters;
  private int cannyLowThreshold = 0;
  private int cannyHighThreshold = 80;
  private int dilationIterations = 1;
  private ContourDetectionTool tool;

  private final int PIXEL_DIFFERENCE_THRESHOLD = 30;

  private final double detectedSizeThreshold = 0.25;
  private double a = -1;
  private static final Logger log = LoggerFactory.getLogger(RobotVisionProcessor.class);

  private String visionsSettingsFile = "visionSettings";

  public RobotVisionProcessor(double boxWidthMeters, double boxLengthMeters, double cameraZDistanceMeters) {
    this.boxWidthMeters = boxWidthMeters;
    this.boxLengthMeters = boxLengthMeters;
    this.cameraZDistanceMeters = cameraZDistanceMeters;

    String homeDir = System.getProperty("user.home");
    settingsFile = Paths.get(homeDir, ".diarc","visionSettings").toString();


    Gson gson = new Gson();
    StringBuilder json = new StringBuilder();
    InputStream is;
    BufferedReader br = null;
    try {
      is = new FileInputStream(settingsFile);
      br = new BufferedReader(new InputStreamReader(is));
    } catch (FileNotFoundException e) {
      log.warn("[loadLocationConfig] error loading location configuration from file at path" + settingsFile);
    }
    if (br != null) {
      List<String> jsonList = br.lines().collect(Collectors.toList());
      for (String s : jsonList) {
        json.append(s);
      }
      PackingVisionSettingsJson settings = gson.fromJson(json.toString(), PackingVisionSettingsJson.class);

      this.cannyLowThreshold = settings.lowThreshold;
      this.cannyHighThreshold = settings.highThreshold;
      this.dilationIterations = settings.dilationIterations;
      this.a = settings.alpha;
      this.defaultCamera = settings.selectedCamera;
    }
    SwingUtilities.invokeLater(() -> {
      tool = new ContourDetectionTool(this);
    });
  }

  /**
   * takes in a captured image and returns the adjustment to rotate and center the box in the image
   *
   * @param capturedImage the image from the robot camera
   * @return the required adjustments, or null if no object was found
   */
  public Vec3 calculatePickupAction(BufferedImage capturedImage) {
    if (a == -1) {
      log.error("[calculatePickupAction] calibration value 'a' has not been set");
      return null;
    }
    // call the contour detection to find the tightest rectangle
    List<Point> rectangleCorners = ContourDetector.findLargestObjectRectangle(
            capturedImage, cannyLowThreshold, cannyHighThreshold, dilationIterations);

    if (rectangleCorners == null) {
      System.out.println("Vision Processor: No object found.");
      return null; // No object found
    }

    // turn pixel coordinates into real-world actions
    Point p0 = rectangleCorners.get(0);
    Point p1 = rectangleCorners.get(1);
    Point p3 = rectangleCorners.get(3);
    double edge1 = p0.distance(p1);
    double edge2 = p0.distance(p3);

    double pixelWidth = Math.min(edge1, edge2);
    double realWidth = Math.min(boxWidthMeters, boxLengthMeters);

    Point widthEdgeP1 = (edge1 < edge2) ? p0 : p3;
    Point widthEdgeP2 = (edge1 < edge2) ? p1 : p0;

    // determine the factor "a" -- note this is computed anew on the fly every time
    // we should really track this over multiple iteration to learn the best value
    double aCalculated = pixelWidth / realWidth;
    double calculatedWidth = pixelWidth / a;
    double ratio = calculatedWidth / realWidth;

    if (ratio > (1 + detectedSizeThreshold) || ratio < (1 - detectedSizeThreshold)) {
      return null;
    }

    // calculate the center of the image and the detected rectangle
    double imageCenterX = capturedImage.getWidth() / 2.0;
    double imageCenterY = capturedImage.getHeight() / 2.0;
    double rectCenterX = (rectangleCorners.get(0).x + rectangleCorners.get(2).x) / 2.0;
    double rectCenterY = (rectangleCorners.get(0).y + rectangleCorners.get(2).y) / 2.0;

    // calculate the shift required in pixels
    double shiftX_pixels = rectCenterX - imageCenterX;
    double shiftY_pixels = rectCenterY - imageCenterY;

    // convert the pixel shift into a real-world shift in meters
    double realWorldShiftX = shiftX_pixels / a;
    double realWorldShiftY = shiftY_pixels / a;

    // calculate the rotation required in degrees
    double rotationAngleRad = Math.atan2(widthEdgeP2.y - widthEdgeP1.y, widthEdgeP2.x - widthEdgeP1.x);

    //update GUI
    tool.update(capturedImage, rectangleCorners, imageCenterX, imageCenterY, rectCenterX, rectCenterY, rotationAngleRad, a, aCalculated, realWorldShiftX, realWorldShiftY, pixelWidth);

    // return the required adjustment in x,y shift and rotation angle in degrees
    return new Vec3(realWorldShiftX, realWorldShiftY, rotationAngleRad);
  }


  // detect "significant" motion in two images
  public double detectAndVisualizeMotion(BufferedImage baseFrame, BufferedImage currentFrame) {
    if (baseFrame.getWidth() != currentFrame.getWidth() || baseFrame.getHeight() != currentFrame.getHeight()) {
      throw new IllegalArgumentException("Images must have the same dimensions.");
    }

    int width = currentFrame.getWidth();
    int height = currentFrame.getHeight();
    int changedPixels = 0;

    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        int baseRgb = baseFrame.getRGB(x, y);
        int currentRgb = currentFrame.getRGB(x, y);

        int baseGray = getGrayscale(baseRgb);
        int currentGray = getGrayscale(currentRgb);

        if (Math.abs(baseGray - currentGray) > PIXEL_DIFFERENCE_THRESHOLD) {
          changedPixels++;
        }
      }
    }
    double totalPixels = width * height;
    return (double) changedPixels / totalPixels * 100.0;
  }

  private static int getGrayscale(int rgb) {
    int r = (rgb >> 16) & 0xFF;
    int g = (rgb >> 8) & 0xFF;
    int b = rgb & 0xFF;
    return (int) (0.299 * r + 0.587 * g + 0.114 * b);
  }

  public void setBoxWidthMeters(double boxWidthMeters) {
    this.boxWidthMeters = boxWidthMeters;
  }

  public void setBoxLengthMeters(double boxLengthMeters) {
    this.boxLengthMeters = boxLengthMeters;
  }

  public void setA(double a) {
    this.a = a;
  }

  public int getCannyLowThreshold() {
    return cannyLowThreshold;
  }

  public int getCannyHighThreshold() {
    return cannyHighThreshold;
  }

  public int getDilationIterations() {
    return dilationIterations;
  }

  public void setCannyLowThreshold(int value) {
    this.cannyLowThreshold = value;
  }

  public void setCannyHighThreshold(int value) {
    this.cannyHighThreshold = value;
  }

  public void setDilationIterations(int value) {
    this.dilationIterations = value;
  }

  public void writeSettings() {
    try {
      Writer writer = new FileWriter(settingsFile);
      Gson gson = new Gson();

      PackingVisionSettingsJson settings = new PackingVisionSettingsJson(cannyLowThreshold, cannyHighThreshold, dilationIterations, a, tool.getSelectedCamera());
      gson.toJson(settings, writer);
      writer.flush();
    } catch (IOException e) {
      log.error("[writeSettings] error writing vision settings to file.", e);
    }
  }
}
