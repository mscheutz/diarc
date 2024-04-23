/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.nao;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.util.Util;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import javax.vecmath.Point3d;

import static java.lang.Math.abs;

/**
 * @author Evan Krause
 */
public class MockNaoComponent extends DiarcComponent implements MockNaoInterface {

  private List<String> possiblePostures;
  private String currentPosture;
  private boolean obstacle;
  private boolean floorSupport;

  // velocity component 
  double curTV;
  double curRV;
  double defTV;
  double defRV;
  final double maxTV = 0.3;
  final double maxRV = 0.5;

  // to mock how long is takes for a move command to execute
  boolean isMoving;

  public MockNaoComponent() {
    super();

    possiblePostures = new ArrayList<>(Arrays.asList("Crouch", "LyingBack", "LyingBelly", "Sit", "SitOnChair", "SitRelax", "Stand", "StandInit", "StandZero"));
    currentPosture = "Stand";
    obstacle = false;
    floorSupport = true;

    // velocity component
    curTV = 0.0;
    curRV = 0.0;
    defTV = 0.1;
    defRV = 0.3;

    isMoving = false;
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("obstacle").hasArg().argName("true/false").desc("set status of obstacle detection").build());
    options.add(Option.builder("floorSupport").hasArg().argName("true/false").desc("set status of floorSupport detection").build());
    return options;
  }

  @Override
  public void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("obstacle")) {
      obstacle = Boolean.valueOf(cmdLine.getOptionValue("obstacle"));
    }
    if (cmdLine.hasOption("floorSupport")) {
      floorSupport = Boolean.valueOf(cmdLine.getOptionValue("floorSupport"));
    }
  }

  @Override
  public boolean isMoving() {
    log.info("isMoving: ({}) {}", getMyGroups(), isMoving);
    return isMoving;
  }

  @Override
  public boolean goToPosture(String targetPosture) {
    log.info("goToPosture: ({}) {}", getMyGroups(), targetPosture);
    if (possiblePostures.contains(targetPosture)) {
      Util.Sleep(500);
      currentPosture = targetPosture;
      return true;
    }
    return false;
  }

  @Override
  public boolean moveTo(float x, float y, float theta) {
    log.info("moveTo: ({})", getMyGroups());
    isMoving = true;
    //start thread to set isMoving to false after specified time
    Thread thread = new Thread(() -> {
      long startTime = System.currentTimeMillis();
      while ((System.currentTimeMillis() - startTime) < 5000) {
        Util.Sleep(100);
      }
      log.info("moveTo done moving");
      isMoving = false;
    });

    thread.start();

    return true;
  }

  @Override
  public boolean moveToBlocking(float x, float y, float theta) {
    log.info("moveToBlocking: ({})", getMyGroups());
    isMoving = true;
    //start thread to set isMoving to false after specified time
    Thread thread = new Thread(() -> {
      long startTime = System.currentTimeMillis();
      while ((System.currentTimeMillis() - startTime) < 5000) {
        Util.Sleep(100);
      }
      log.info("moveToBlocking done moving ({})", getMyGroups());
      isMoving = false;
    });

    thread.start();

    return true;
  }

  @Override
  public boolean pointTo(float x, float y, float z) {
    log.info("pointTo ({})", getMyGroups());
    return true;
  }

  @Override
  public boolean pointHeadTo(float x, float y, float z) {
    log.info("pointHeadTo ({}) ({},{},{})", getMyGroups(), x, y, z);
    return true;
  }

  @Override
  public boolean checkFloorSupport() {
    log.debug("checkFloorSupport");
    Util.Sleep(1000);
    return floorSupport;
  }

  @Override
  public boolean checkObstacle() {
    log.debug("checkObstacle");
    Util.Sleep(1000);
    return obstacle;
  }

  @Override
  public void setFloorSupport(boolean value) {
    log.info("setFloorSupport to: ({}) {}", getMyGroups(), value);
    floorSupport = value;
  }

  @Override
  public void setObstacle(boolean value) {
    log.info("setObstacle to: ({}) {}", getMyGroups(), value);
    obstacle = value;
  }

  @Override
  public String getPosture() {
    log.info("getPosture ({})", getMyGroups());
    return currentPosture;
  }

  @Override
  public List<String> getPostureList() {
    log.info("getPostureList ({})", getMyGroups());
    return possiblePostures;
  }

  @Override
  public void playFile(String fileName) {
    log.info("playFile: " + fileName);
  }

  @Override
  public void angleInterpolation(String name, float angle, float time, boolean isAbsolute) {
    log.info("angleInterpolation: ({}) {}", getMyGroups(), name);
    Util.Sleep(500);
  }

  @Override
  public void angleInterpolation(List<String> names, List<Float> angles, float time, boolean isAbsolute) {
    log.info("angleInterpolation: ({}) {}", getMyGroups(), names);
    Util.Sleep(500);
  }

  @Override
  public void angleInterpolation(List<String> names, List<Float> angles, List<Float> times, boolean isAbsolute) {
    log.info("angleInterpolation: ({}) {}", getMyGroups(), names);
    Util.Sleep(500);
  }

  @Override
  public float getAngle(String name) {
    log.info("getAngle: ({}) {}", getMyGroups(), name);
    return 0.0f;
  }

  // ********************************************************************
  // *** VelocityComponent interface
  // ********************************************************************
  @Override
  public boolean setVels(double tv, double rv) {
    log.info("setVels tv: ({}) {} rv: {}", getMyGroups(), tv, rv);
    if (tv > maxTV) {
      tv = maxTV;
    }
    if (rv > maxRV) {
      rv = maxRV;
    }

    curTV = tv;
    curRV = rv;

    if (abs(curTV) > 0 || abs(curRV) > 0) {
      isMoving = true;
    } else {
      isMoving = false;
    }
    return true;
  }

  @Override
  public boolean setRV(double rv) {
    log.info("setRV: ({}) {}", getMyGroups(), rv);
    return setVels(0.0, rv);
  }

  @Override
  public boolean setTV(double tv) {
    log.info("setTV: ({}) {}", getMyGroups(), tv);
    return setVels(tv, 0.0);
  }

  @Override
  public double[] getDefaultVels() {
    log.info("getDefaultVels");
    return new double[]{defTV, defRV};
  }

  @Override
  public double[] getVels() {
    log.info("getVels");
    return new double[]{curTV, curRV};
  }

  @Override
  public double getRV() {
    log.info("getRV");
    return curRV;
  }

  @Override
  public double getTV() {
    log.info("getTV");
    return curTV;
  }

  @Override
  public void stop() {
    log.info("stop ({})", getMyGroups());
    curTV = 0.0;
    curRV = 0.0;
    isMoving = false;
  }

  @Override
  public boolean restLeftArm() {
    log.info("restLeftArm ({})", getMyGroups());
    return true;
  }

  @Override
  public boolean restRightArm() {
    log.info("restRightArm ({})", getMyGroups());
    return true;
  }

  @Override
  public boolean setStiffness(String name, float stiffness) {
    log.info("setStiffness ({})", getMyGroups());
    return true;
  }

  @Override
  public boolean rest() {
    log.info("rest ({})", getMyGroups());
    return true;
  }

  @Override
  public boolean welcome() {
    log.info("welcome ({})", getMyGroups());
    return true;
  }

  @Override
  public boolean openHand(String name) {
    log.info("openHand ({})", getMyGroups());
    return true;
  }

  @Override
  public boolean closeHand(String name) {
    log.info("closeHand ({})", getMyGroups());
    return true;
  }

  @Override
  public Point3d getPixelPositionInRobotFrame(int pX, int pY, int width, int height, int camID) {
    log.info("getPixelPositionInRobotFrame");
    return new Point3d(0.0, 0.0, 0.0);
  }

  @Override
  public boolean sayAnimated(String annotatedText) {
    log.info("sayAnimated ({})", getMyGroups());
    return true;
  }

  @Override
  public boolean setBreathEnabled(String par, boolean enable) {
    log.info("setBreathEnabled ({})", getMyGroups());
    return true;
  }

  @Override
  public boolean ledOn(String name) {
    log.info("ledOn ({})", getMyGroups());
    return true;
  }

  @Override()
  public boolean ledOff(String name) {
    log.info("ledOff ({})", getMyGroups());
    return true;
  }

  @Override
  public boolean blink() {
    log.info("blink ({})", getMyGroups());
    return true;
  }

  // ********************************************************************
  // *** SpeechProductionComponent interface
  // ********************************************************************
  @Override
  public boolean sayText(String text) {
    log.info("sayText: ({}) {}", getMyGroups(), text);
    return true;
  }

  @Override
  public boolean sayText(String text, boolean wait) {
    log.info("sayText: ({}) {}", getMyGroups(), text);
    return true;
  }

  @Override
  public boolean sayToFile(String text, String filename) {
    log.info("sayToFile: " + text);
    return true;
  }

  @Override
  public boolean isSpeaking() {
    return false;
  }

  @Override
  public boolean stopUtterance() {
    log.info("stopUtterance ({})", getMyGroups());
    return true;
  }
}
