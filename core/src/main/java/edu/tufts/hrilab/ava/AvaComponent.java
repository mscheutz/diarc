/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava;

import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.OnInterrupt;
import edu.tufts.hrilab.asr.ConfigurableMicrophone;
import edu.tufts.hrilab.ava.message.AvaMoveGoal;
import edu.tufts.hrilab.ava.message.response.*;
import edu.tufts.hrilab.consultant.pose.PoseReference;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.map.PathAction;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;

import java.awt.image.BufferedImage;
import java.io.*;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.Duration;
import java.time.LocalDateTime;
import java.time.LocalTime;
import java.time.format.DateTimeFormatter;
import java.time.format.DateTimeParseException;
import java.time.temporal.ChronoUnit;
import java.util.*;

import java.net.URI;
import java.net.URISyntaxException;

import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.slug.common.UtteranceType;
import edu.tufts.hrilab.util.Http;
import edu.tufts.hrilab.util.RotationHelpers;
import edu.tufts.hrilab.util.Convert;
import edu.tufts.hrilab.util.Util;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.lang.Math;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.Semaphore;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import javax.imageio.ImageIO;
import javax.sound.sampled.*;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import javax.vecmath.Tuple3d;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

import io.github.cdimascio.dotenv.Dotenv;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import org.w3c.dom.Document;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import static java.lang.Double.parseDouble;
import static java.lang.System.getProperty;

/**
 * Represents the Ava robot navigation functionality.
 *
 * @author Matt McWilliams
 */
public class AvaComponent extends DiarcComponent implements AvaInterface {

  private boolean polling = false;

  private AvaHTTPAPI api;
  private double pointDistThresh = 2; //meters
  private double orientationThresh = 1; //radians
  private double zLiftThresh = .05; //meters
  private double cameraTiltThresh = 0.2;
  private double cameraHeight = 1.350; //meters
  private double cameraHeightMax = 0.0;

  private String uriString = "http://172.18.0.1";
  private List<BufferedImage> mapImages = new ArrayList<>();
  private List<RMSMapMetadataResponse> mapMetadatas = new ArrayList<>();
  private Thread sleepForDurationThread = null;
  private Symbol speaker = Factory.createSymbol("local_user");

  private Symbol dockLocation;

  private final int dockingTimeout = 90; //30 seconds

  private boolean stopped = false;

  /**
   * Flag to keep follow action loop alive.
   */
  private volatile boolean shouldFollow = false;

  private final AvaMoveGoal currentMoveGoal = new AvaMoveGoal();

  private final ScheduledExecutorService keepAliveExecutor = Executors.newScheduledThreadPool(1);
  private final ScheduledExecutorService stopGoingHomeExecutor = Executors.newScheduledThreadPool(1);

  private final Lock chargingLock = new ReentrantLock();
  private final Condition chargingTerminatedCondition = chargingLock.newCondition();

  private OutputStream ww_process_os = null;
  private boolean shouldDing = true;
  private OutputStream ready_process_os = null;
  private boolean shouldDingReady = true;
  private boolean listenForWW = true;
  private int mixerID = -1;
  private boolean haveDisplay = true;

  public AvaComponent() {
    super();
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("uri").hasArg().argName("uri").desc("Set the URI of the Ava Robotics RMS (default: " + uriString + " )").build());
    options.add(Option.builder("point_dist_thresh").hasArg().argName("point_distance_threshold").desc("Set the point distance threshold in meters (default: " + pointDistThresh + ")").build());
    options.add(Option.builder("orientation_thresh").hasArg().argName("orientation_threshold").desc("Set the rotation orientation threshold in radians (default: " + orientationThresh + ")").build());
    options.add(Option.builder("z_lift_thresh").hasArg().argName("z_lift_threshold").desc("Set the Z lift threshold in meters (default: " + zLiftThresh + ")").build());
    options.add(Option.builder("camera_tilt_thresh").hasArg().argName("camera_til_threshold").desc("Set the camera tilt threshold  (default: " + cameraTiltThresh + ")").build());
    options.add(Option.builder("camera_height").hasArg().argName("camera_height").desc("Set the height of the camera in meters (default:" + cameraHeight + ")").build());
    options.add(Option.builder("matchMixer").hasArgs().argName("name").desc("String to match for mixer in case of device re-ordering. Do not quote argument if spaces needed.").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    String envURI = null;
    try {
      Dotenv dotenv = Dotenv.load();

      envURI = dotenv.get("AVA_URI");
      log.debug("Loaded .env file with environment variables");
    } catch (Exception ex) {
      log.error("Exception caught during AvaComponent parseArgs.", ex);
    }

    if (cmdLine.hasOption("uri")) {
      uriString = cmdLine.getOptionValue("uri");
    } else if (envURI != null) {
      uriString = envURI;
    } else {
      log.debug("[parseArgs] Using default URI: " + uriString);
    }

    if (cmdLine.hasOption("point_dist_thresh")) {
      pointDistThresh = Double.valueOf(cmdLine.getOptionValue("point_dist_thresh"));
    }

    if (cmdLine.hasOption("orientation_thresh")) {
      orientationThresh = Double.valueOf(cmdLine.getOptionValue("orientation_thresh"));
    }

    if (cmdLine.hasOption("z_lift_thresh")) {
      zLiftThresh = Double.valueOf(cmdLine.getOptionValue("z_lift_thresh"));
    }

    if (cmdLine.hasOption("camera_tilt_thresh")) {
      cameraTiltThresh = Double.valueOf(cmdLine.getOptionValue("camera_tilt_thresh"));
    }

    if (cmdLine.hasOption("camera_height")) {
      cameraHeight = Double.valueOf(cmdLine.getOptionValue("camera_height"));
    }

    if (cmdLine.hasOption("matchMixer")) {
      String matchString = String.join(" ", cmdLine.getOptionValues("matchMixer"));
      mixerID = ConfigurableMicrophone.matchMixer(matchString);
      log.debug("Matching mixer string \"" + matchString + "\"");
    }

  }

  @Override
  protected void init() {
    URI avaUri;

    try {
      avaUri = new URI(uriString);
    } catch (URISyntaxException ex) {
      log.error("Error constructing URI from string ");
      log.error(ex.toString());
      return;
    }
    log.info("Connecting to local HTTP server " + uriString);

    api = new AvaHTTPAPI(avaUri);

    try {
      loadMapMetadata();
    } catch (Exception e) {
      log.error("[init] unable to connect to Ava robot API");
      System.exit(1);
    }

    List<String> dockNames = getDockTags();

    //reach out and add references to the map component
    //todo: (pete) this stinks. we shouldn't have to be reaching out like this. can we just share the api with a specialized map component instead?
    TRADEServiceInfo addReferenceToMapService;
    try {
      addReferenceToMapService = TRADE.getAvailableService(new TRADEServiceConstraints().name("addReferencesDirectly").argTypes(Map.class));
    } catch (TRADEException e) {
      log.error("Error getting addReferencesDirectly service", e);
      return;
    }

    Map<String, Pair<Point3d, Quat4d>> tags = getAllTags();
    adjustDockTags(tags, dockNames);
    try {
      addReferenceToMapService.call(void.class, tags);
    } catch (TRADEException e) {
      log.error("[init] error calling addReferencesDirectly");
    }

    //todo: (pete) should go in a consultant? assumes unique names? can only handle one dock?
    //todo: (pete) it really feels like the "dock-ness" of a location should be a property?
    if (dockNames.size() > 1) {
      log.error("[init] multiple tags labeled as dock in the map. only one dock currently supported");
    } else if (dockNames.isEmpty()) {
      log.error("[init] no dock marked in map! a dock is required.");
    } else {

      TRADEServiceInfo getReferencesService;
      try {
        getReferencesService = TRADE.getAvailableService(new TRADEServiceConstraints().name("getMapReferences").argTypes());
      } catch (TRADEException e) {
        log.error("[init] error getting getMapReferences service ", e);
        return;
      }

      Map<Symbol, PoseReference> poseReferences = new HashMap<>();
      try {
        poseReferences = getReferencesService.call(Map.class);
      } catch (TRADEException e) {
        log.error("[init] error calling getMapReferences");
      }

      String dockName = dockNames.get(0);
      for (Symbol refId : poseReferences.keySet()) {
        //abosultely shouldn't be blind indexing into the ref to get the name!!!!!!!
        if (poseReferences.get(refId).properties.get(0).getName().equals(dockName)) {
          dockLocation = refId;
        }
      }
    }


    keepAlive();
    stopGoingHomeOnTheHour();

    // get notified when display services become available
    try {
      TRADE.registerNotificationNowOrWhenJoined(this, new TRADEServiceConstraints().name("displayRobot").argTypes(), "displayRobot");
    } catch (TRADEException e) {
      log.error("Error registering for displayRobot service notification.", e);
    }

//    setZLift(0);
    getCameraHeightMax();

    //mixerID = ConfigurableMicrophone.matchMixer("Direct Audio Device: HDA Intel PCH, HDMI 0, CS-CODECPLU-3");
  }

  private void adjustDockTags(Map<String, Pair<Point3d, Quat4d>> allTags, List<String> dockTags) {
    if (allTags.isEmpty()) {
      return;
    }
    log.info(allTags.toString());
    log.info(dockTags.toString());
    for (String name : dockTags) {
      Pair<Point3d, Quat4d> pose = allTags.get(name);
      if (pose != null) {
        Vector3d orientationVec = RotationHelpers.quaternionToNormVect(pose.getRight());
        Point3d position = pose.getLeft();
        log.info("adjusting dock tag " + name + " with position " + pose.getLeft());
        position.add(orientationVec);
        Pair<Point3d, Quat4d> newPose = new MutablePair<>(position, pose.getRight());
        log.info("new position for " + name + " is " + newPose.getLeft());
        allTags.put(name, newPose);
      }
    }
  }

  @Override
  public Symbol getDockRefId() {
    return dockLocation;
  }

  /**
   * Calls the displayRobot TRADE service on the AvaDisplayComponent if available.
   */
  @Override
  public void displayRobot(TRADEServiceInfo displayService) {
    try {
      displayService.call(void.class);
    } catch (TRADEException e) {
      log.error("Error displaying robot", e);
    }
  }

  /**
   * Displays arbitrary text on the AvaDisplayComponent if available.
   *
   * @param text String to display on screen
   */
  private void displayText(String text) {
    TRADEServiceInfo handle;
    try {
      handle = TRADE.getAvailableService(new TRADEServiceConstraints().name("displayText").argTypes(String.class));
    } catch (TRADEException e) {
      log.error("Error getting displayText service", e);
      return;
    }

    try {
      handle.call(void.class, text);
    } catch (TRADEException e) {
      log.error("Error calling displayText service. text: {}", text, e);
    }
  }

  /**
   * Used to updated Firebase GUI
   * TODO: is this method needed?
   *
   * @return Point3d of robot position
   */
  @TRADEService
  public Point3d getPosition() {
    RMSPositionResponse res = api.getPosition();
    double positionX = Double.valueOf(res.response.x);
    double positionY = Double.valueOf(res.response.y);
    return new Point3d(positionX, positionY, 0);
  }


  //////////////////////////////////////////////////////////////////////
  // Position Interface
  //////////////////////////////////////////////////////////////////////

  /**
   * Deprecated. Cannot set pose of Ava robot without moving to location.
   *
   * @param location    robot location
   * @param orientation robot orientation
   */
  @Override
  public void setPose(Point3d location, Quat4d orientation) {
    log.warn("[setPose] functionality does not exist in Ava API.");
  }

  //////////////////////////////////////////////////////////////////////
  // AvaComponent Interface
  //////////////////////////////////////////////////////////////////////

  @Override
  public Pair<Point3d, Quat4d> getPose() {
    RMSPositionResponse res = api.getPosition();
    double positionX = Double.valueOf(res.response.x);
    double positionY = Double.valueOf(res.response.y);
    log.debug("[getPose] " + positionX + ", " + positionY + " theta " + res.response.theta);
    Quat4d quat4d = Convert.convertToQuat4d(Double.valueOf(res.response.theta), 0, 0);
    return new MutablePair<>(new Point3d(positionX, positionY, 0), quat4d);
  }

  /**
   * Retrieves the current robot position and rotation and returns a converted Matrix4d.
   *
   * @return robot pose as Matrix4d
   */
  public Matrix4d getPoseGlobalMatrix4d() {
    RMSPositionResponse res = api.getPosition();
    double positionX = Double.valueOf(res.response.x);
    double positionY = Double.valueOf(res.response.y);
    log.debug("[getPoseGlobalMatrix4d] x " + res.response.x + " y " + res.response.y + " theta " + res.response.theta);

    Point3d currentPosition = new Point3d(positionX, positionY, 0);
    double currentTheta = Double.valueOf(res.response.theta);
    Quat4d rotation = RotationHelpers.createQuaternion(new Vector3d(0, 0, 1), (float) currentTheta);

    return Convert.convertToMatrix4d(currentPosition, rotation);
  }

  /**
   * Go to a known location in the world.
   *
   * @param location a known location.
   * @param wait     should wait until movement is complete
   */
  @Override
  public Justification goToLocation(Symbol location, boolean wait) {
    log.debug("[goToLocation] Going to:{} Wait is {}", location.getName(), wait);
    currentMoveGoal.setGoToLocationGoal(location, wait);

    double padding = 1.0;
    boolean canOpenDoors = false;
    List<PathAction> path;

    Matrix4d currentPosition = getPoseGlobalMatrix4d();

    try {
      path = TRADE.getAvailableService(new TRADEServiceConstraints().name("getPath"))
              .call(List.class, currentPosition, location, padding, canOpenDoors);
    } catch (TRADEException e) {
      log.error("Error trying to get path for location: {}", location, e);
      return new ConditionJustification(false, Factory.createPredicate("path_to", location.getName(), "known"));
    }

    if (path.isEmpty()) {
      log.error("Empty path to location: {}", location);
      return new ConditionJustification(false, Factory.createPredicate("path_to", location.getName(), "known"));
    }
    Justification followPathRes = followPath(path);
    currentMoveGoal.setIdleGoal(AvaMoveGoal.CommandType.GO_TO_LOCATION);
    return followPathRes;
  }

//  @Override
  public Justification goToLocation(Symbol location) {
    return goToLocation(location, true);
  }

  @Override
  public Justification goToLocation(Symbol desiredLocation, Symbol initialLocation) {
    return goToLocation(desiredLocation, true);
  }

  @Override
  public Justification goToLocation(Point3d location, Quat4d orientation, boolean wait) {
    log.warn("[goToLocation] not implemented.");
    return new ConditionJustification(false);
  }

  @Override
  public Justification goToLocation(double xdest, double ydest, double quat_x, double quat_y, double quat_z, double quat_w, boolean wait) {
    log.warn("[goToLocation] not implemented.");
    return new ConditionJustification(false);
  }

  @Override
  public Justification approachLocation(Symbol desiredLocation) {
    goToLocation(desiredLocation);
    return new ConditionJustification(true);
  }

  @Override
  public Justification approachLocation(Symbol desiredLocation, Symbol initialLocation) {
    goToLocation(desiredLocation, initialLocation);
    return new ConditionJustification(true);
  }

  /**
   * Go to a position on the Ava robot map that is not named location in RMS.
   *
   * @param poseTransform target robot pose as Matrix4d
   * @return ConditionJustification of action
   */
  private Justification goToArbitrary(Matrix4d poseTransform) {
    int apiFailures = 0;
    int statusDelay = 0;

    // Setting up goal location and submitting to the api
    Pair<Point3d, Quat4d> goalPose = Convert.convertToPair(poseTransform);
    Point3d goalPosition = goalPose.getLeft();
    Quat4d orientation = goalPose.getRight();
    double[] angles = Convert.convertToEuler(orientation);
    double goalTheta = angles[Convert.YAW];

    log.debug("[goToArbitrary()] " + poseTransform);
    double positionX = goalPosition.x;
    double positionY = goalPosition.y;

    stopped = false;
    polling = true;
    log.info("[goToArbitrary] Going to x: " + positionX + ", y: " + positionY + ", theta: " + goalTheta);
    RMSResponse res = api.goToArbitrary(positionX, positionY, goalTheta);

    // Checking if there was an errorMessage
    if (res != null && res.errorMessage != null) {
      log.error("Error going to location: " + res.errorMessage);
      polling = false;
      if (res.errorMessage.contains("Inaccessible map location")) {
        return new ConditionJustification(false, Factory.createPredicate("accessible", "location"));
      } else {
        return new ConditionJustification(false, Factory.createPredicate("websocket_server", "connected"));
      }
    }

    // Keep checking until the robot is close enough to the goal (point and orientation threshold)
    DriveStatus ds = getDriveStatus();
    Pair<Point3d, Quat4d> currentPose = getPose();
    Point3d currentPosition = currentPose.getLeft();
    Quat4d currentOrientation = currentPose.getRight();
    double[] currentAngles = Convert.convertToEuler(currentOrientation);
    double currentTheta = currentAngles[Convert.YAW];
    while ((currentPosition.distance(goalPosition) > pointDistThresh
            || Math.abs(goalTheta - currentTheta) > orientationThresh)
            && ds != DriveStatus.COMPLETE_DRIVE_WAYPOINT) {

      // update current pose
      currentPose = getPose();
      currentPosition = currentPose.getLeft();
      currentOrientation = currentPose.getRight();
      currentAngles = Convert.convertToEuler(currentOrientation);
      currentTheta = currentAngles[Convert.YAW];

      // Every 3 seconds check the status
      if (statusDelay > 3) {
        try {
          RMSGoToStatusResponse status = api.getGoToStatus();
          // TODO: use the status
        } catch (Exception ex) {
          log.error("Error sending go to status request to RMS", ex);
          apiFailures++;
        }
      } else {
        statusDelay++;
      }

      ds = getDriveStatus();
      if (ds == DriveStatus.FAILED_DRIVE_WAYPOINT) {
        log.warn("[goToArbitrary] Failed to reach goal position: " + goalPosition + " theta: " + goalTheta);
        return new ConditionJustification(false, Factory.createPredicate("reachable", "location"));
      }

      // Stop the robot if the DIARC is set to stop the robot
      if (stopped) {
        break;
      }

      try {
        Thread.sleep(1000);
      } catch (Exception ex) {
        log.error("Error sleeping 1000ms", ex);
      }

      if (apiFailures >= 4) {
        return new ConditionJustification(false, Factory.createPredicate("http_server", "connected"));
      }
    }
    polling = false;
//    log.info("[goToArbitrary] Arrived at x: " + positionX + ", y: " + positionY + ", theta: " + goalTheta);
    return new ConditionJustification(true);
  }

  /**
   * Stop the robot by interrupting a goToLocation or goToArbitrary action.
   *
   * @return true
   */
  @Override
  public Justification stop() {
    log.info("[stop()] called");
    stopped = true;
    api.stop();
    return new ConditionJustification(true);
  }

  /**
   * Return whether or not robot is in motion.
   *
   * @return Justification true if moving
   */
  @Override
  public Justification isMoving() {
    boolean moving = api.isMoving();
    return new ConditionJustification(moving);
  }

  /**
   * Checks to see if the robot is within atOffset distance from goal pose
   *
   * @param locationTerm the predicate to observe at(?actor, ?location)
   * @return list of argument bindings empty list is failure to observe
   * list with empty hashmap means actor is within atOffset of goal pose
   */
  @Override
  public List<Map<Variable, Symbol>> checkAt(Term locationTerm) {
    ArrayList<Map<Variable, Symbol>> returnVal = new ArrayList<>();

    // get pose reference for target ?location
    Symbol refId = locationTerm.get(1);
    PoseReference poseReference = null;
    try {
      poseReference = TRADE.getAvailableService(new TRADEServiceConstraints().name("getMapReference").argTypes(Symbol.class))
                      .call(PoseReference.class, refId);
      poseReference.getPosition();
      poseReference.getOrientation();
    } catch (TRADEException e) {
      log.error("Error getting pose reference for {}", refId, e);
      return returnVal;
    }
    Point3d goalPosition = poseReference.getPose().getLeft();
    Quat4d goalOrientation = poseReference.getPose().getRight();
    double[] goalAngles = Convert.convertToEuler(goalOrientation);
    double goalTheta = goalAngles[Convert.YAW];

    // get current pose
    Pair<Point3d, Quat4d> currentPose = getPose();
    Point3d currentPosition = currentPose.getLeft();
    Quat4d currentOrientation = currentPose.getRight();
    double[] currentAngles = Convert.convertToEuler(currentOrientation);
    double currentTheta = currentAngles[Convert.YAW];

    log.info("[checkAt] Arrived at x: " + currentPosition.x+ ", y: " + currentPosition.y + ", theta: " + currentTheta);

    double pointDist = currentPosition.distance(goalPosition);
    double orientationDiff = Math.abs(goalTheta - currentTheta);
    if (pointDist <= pointDistThresh && orientationDiff <= orientationThresh) {
      returnVal.add(new HashMap<>());
    } else {
      if (pointDist > pointDistThresh) {
        log.warn("Point distance is above threshold: " + pointDist + " > " + pointDistThresh);
      }
      if (orientationDiff > orientationThresh) {
        log.warn("Orientation difference is above threshold: " + orientationDiff + " > " + orientationThresh);
      }
    }
    //
    return returnVal;
  }

  protected Justification followPath(List<PathAction> path) {
    log.debug("[followPath] Starting Follow Path");
    for (PathAction nextAction : path) {
      switch (nextAction.getType()) {
        case ACTION_GOTO: {
          Matrix4d pose = nextAction.getPose();
          Justification result = goToArbitrary(pose);
          if (!result.getValue()) {
            return result;
          }
        }
        break;
        case ACTION_CHECK_DOOR: {
          // TODO: check for open door
          log.warn("Cannot currently check for open doors. Attempting to navigate to next location in path.");
        }
        break;
        case ACTION_OPEN_DOOR: {
          // TODO: call door opening
          log.error("Cannot currently open doors. Failing execution.");
          return new ConditionJustification(false, Factory.createPredicate("property_of", nextAction.getReference(), Factory.createSymbol("blocked")));
        }
      }
    }
    return new ConditionJustification(true);
  }

  /**
   * Send the Ava robot to dock.
   */
  @Override
  public Justification goToDock() {
    boolean docked = false;
    int timeoutCounter = 0;
    api.dock();
    while (!docked) {
      if (timeoutCounter > dockingTimeout) {
        log.error("Docking process timed out");
        return new ConditionJustification(false);
      }
      try {
        Thread.sleep(1000);
      } catch (Exception ex) {
        log.error("Error sleeping 1000ms", ex);
      }
      RMSDockStatusResponse res = api.getDockStatus();
      docked = res.response.state.equals("Docked");
      timeoutCounter++;
    }
    setZLift(0);
    log.debug("[goToDock] Ava has docked");
    return new ConditionJustification(true);
  }

  /**
   * Get the scale and dimensions of the current map.
   * Store metadata in class properties.
   */
  private void loadMapMetadata() {
    mapMetadatas = api.getMapMetadatas();
    mapImages = api.getMaps();
  }

  @TRADEService
  public List<byte[]> getMapImages() {
    List<byte[]> toReturn = new ArrayList<>();
    for (BufferedImage image : mapImages) {
      ByteArrayOutputStream baos = new ByteArrayOutputStream();
      try {
        ImageIO.write(image, "png", baos); // Choose your desired format
      } catch (IOException e) {
        log.error("[getMapImages] exception writing map image to serializable format");
      }
      toReturn.add(baos.toByteArray());
    }
    return toReturn;
  }

  @TRADEService
  public List<RMSMapMetadataResponse> getMapMetadatas() {
    return mapMetadatas;
  }

  /**
   * Run background thread to poll navigation server for current position.
   * Used to keep websocket connection alive.
   */
  private void keepAlive() {
    keepAliveExecutor.scheduleAtFixedRate(() -> {
      try {
        if (!polling && api.hasSession()) {
          api.getHealth();
        }
      } catch (Exception e) {
        log.error("Exception while keeping Ava alive.", e);
      }
    }, 0, 10, TimeUnit.SECONDS);
  }

  /**
   * A (kind of hacky) script to override a lower-level system that automatically sends the robot back to the dock on the hour
   *
   * It continuously checks the robot's drive status and intervenes if the robot attempts to go home on the hour
   * without an explicit move goal. In that situation the script stops that action and reinstates the previous move goal.
   */
  private void stopGoingHomeOnTheHour() {
    stopGoingHomeExecutor.schedule(() -> {
      // Wait until next hour
      LocalDateTime now = LocalDateTime.now();
      LocalDateTime nextHour = now.truncatedTo(ChronoUnit.HOURS).plusHours(1);
      long sleepTime = Math.max(Duration.between(now, nextHour).toMillis() - (60 * 1000), 0); // A minute before the hour
      try {
        Thread.sleep(sleepTime);
      } catch (InterruptedException e) {
        log.error("[stopGoingHomeOnTheHour] Error sleeping until next hour", e);
      }

      long startTime = System.currentTimeMillis();
      long endTime = startTime + 5 * 60 * 1000; // 5 minutes
      boolean caughtGoingHomeOnTheHour = false;
      while (System.currentTimeMillis() < endTime || !caughtGoingHomeOnTheHour) {
        DriveStatus ds = getDriveStatus();
        if (ds == DriveStatus.GO_HOME && currentMoveGoal.getCommandType() != AvaMoveGoal.CommandType.GO_HOME) {
          caughtGoingHomeOnTheHour = true;
          log.info("[stopGoingHomeOnTheHour] Robot was going home without a move goal. Stopped and reinstated previous move goal.");
          stop();
          reinstateMoveGoal();
        }
        try {
          Thread.sleep(1000);
        } catch (InterruptedException ex) {
          log.error("[stopGoingHomeOnTheHour] Error sleeping 1000ms", ex);
        }
      }
    }, 1, TimeUnit.SECONDS);
  }

  /**
   * Reinstates previous move goal. Mainly in the case when a lower level system overrides current diarc functionality
   */
  private void reinstateMoveGoal() {
    log.debug("[reinstateMoveGoal]: {}", currentMoveGoal);
    if (currentMoveGoal.isIdle()) {
    } else if (currentMoveGoal.isGoHome()) {
      goHomePrimitive();
    } else if (currentMoveGoal.isGoToLocation()) {
      goToLocation((Symbol) currentMoveGoal.getParameters().get(0), (boolean) currentMoveGoal.getParameters().get(1));
    } else {
      log.warn("[reinstateMoveGoal] currentMoveGoal contains unknown input");
    }
  }

  @Override
  public Justification goHomePrimitive() {
    log.debug("Starting [goHome]");
    currentMoveGoal.setGoHomeGoal();
    Justification ret;
    if (!stopped) {
      ret = goToDock();
    } else {
      ret = new ConditionJustification(false);
    }
    currentMoveGoal.setIdleGoal(AvaMoveGoal.CommandType.GO_HOME);
    log.debug("Finishing [goHome]");
    return ret;
  }

  @Override
  public Justification endSession() {
    api.end();
    return new ConditionJustification(true);
  }

  @Override
  public double getBatteryLevel() {
    double level = api.getBatteryLevel();
    log.debug("Battery level: " + level + "%");
    return level;
  }

  @Override
  public double getZLift() {
    RMSZLiftCameraTiltResponse res = api.getZLiftCameraTilt();
    double lift = Double.valueOf(res.response.zLift.position) / (Double.valueOf(res.response.zLift.maxPosition) - Double.valueOf(res.response.zLift.minPosition));
    lift *= 100.0;
    log.info("Z lift at: " + lift + "%");
    return lift;
  }

  /**
   * Calculate the maximum height of the camera using the Z lift range.
   * Assumes camera height is at lowest position.
   */
  private void getCameraHeightMax() {
    RMSZLiftCameraTiltResponse res = api.getZLiftCameraTilt();
    cameraHeightMax = cameraHeight + (Double.valueOf(res.response.zLift.maxPosition) - Double.valueOf(res.response.zLift.minPosition));
    log.info("Camera height max: " + cameraHeightMax + " meters");
  }

  @Override
  public Justification setZLift(double lift) {
    RMSZLiftCameraTiltResponse res = api.getZLiftCameraTilt();
    double position = Double.valueOf(res.response.zLift.position);
    double minPosition = Double.valueOf(res.response.zLift.minPosition);
    double target = ((lift / 100.0) * (Double.valueOf(res.response.zLift.maxPosition) - minPosition)) + minPosition;
    api.setZLift(target);
    while (Math.abs(position - target) > zLiftThresh) {
      res = api.getZLiftCameraTilt();
      position = Double.valueOf(res.response.zLift.position);
      try {
        Thread.sleep(1000);
      } catch (Exception ex) {
        log.error("Error sleeping 1000ms", ex);
      }
    }
    log.info("Z lift set to: " + target + " [" + lift + "%]");
    return new ConditionJustification(true);
  }

  @Override
  public double getCameraTilt() {
    RMSZLiftCameraTiltResponse res = api.getZLiftCameraTilt();
    double tilt = Double.valueOf(res.response.cameraTilt.position) / (Double.valueOf(res.response.cameraTilt.maxPosition) - Double.valueOf(res.response.cameraTilt.minPosition));
    tilt *= 100.0;
    log.info("Z lift at: " + tilt + "%");
    return tilt;
  }

  @Override
  public Justification setCameraTilt(double tilt) {
    RMSZLiftCameraTiltResponse res = api.getZLiftCameraTilt();
    double position = Double.valueOf(res.response.cameraTilt.position);
    double minPosition = Double.valueOf(res.response.cameraTilt.minPosition);
    double target = ((tilt / 100.0) * (Double.valueOf(res.response.cameraTilt.maxPosition) - minPosition)) + minPosition;
    api.setCameraTilt(target);
    while (Math.abs(position - target) > cameraTiltThresh) {
      res = api.getZLiftCameraTilt();
      position = Double.valueOf(res.response.cameraTilt.position);
      try {
        Thread.sleep(1000);
      } catch (Exception ex) {
        log.error("Error sleeping 1000ms", ex);
      }
    }
    log.info("Camera tilt to: " + target + " [" + tilt + "%]");
    return new ConditionJustification(true);
  }

  /**
   * Set the rotation of the robot.
   *
   * @param orientation Quat4d representing goal robot rotation
   */
  public Justification setRotation(Quat4d orientation) {
    Tuple3d xyzOrientation = RotationHelpers.quatToXYZRotations(orientation);
    return setRotation(xyzOrientation.z);
  }

  /**
   * Set the rotation of the robot.
   *
   * @param theta goal Z rotation of the robot
   */
  public Justification setRotation(double theta) {
    RMSPositionResponse res = api.getPosition();
    double currentRotation = Double.valueOf(res.response.theta);
    api.setRotation(theta);
    while (Math.abs(currentRotation - theta) > orientationThresh) {
      res = api.getPosition();
      currentRotation = Double.valueOf(res.response.theta);
      try {
        Thread.sleep(1000);
      } catch (Exception ex) {
        log.error("Error sleeping 1000ms", ex);
      }
    }
    log.info("Robot rotation set to: " + theta + " [" + currentRotation + "]");
    return new ConditionJustification(true);
  }

  @Override
  public double getRuntime() {
    RMSRuntimeResponse res = api.getRuntime();
    double runtime = Double.valueOf(res.response.minutes) * 60.0;
    log.debug("Estimated remaining runtime = " + runtime + " seconds");
    return runtime;
  }

  /**
   * Get the ranges of the laser scan.
   *
   * @return float array of laser scans in meters
   */
  @Override
  public float[] getLaserScan() {
    RMSLidarResponse lidarData = api.getLidar();
    float[] laserScan = new float[lidarData.response.ranges.length];
    for (int i = 0; i < lidarData.response.ranges.length; i++) {
      laserScan[i] = (float) lidarData.response.ranges[i];
    }
    return laserScan;
  }

  /**
   * Get the angle increment between scans. Uses difference between the first two.
   *
   * @return angle increment as float
   */
  @Override
  public float getLaserAngleIncrement() {
    RMSLidarResponse lidarData = api.getLidar();
    return (float) Math.abs(lidarData.response.ranges[1] - lidarData.response.ranges[0]);
  }

  @Override
  public List<Obstacle> getDynamicObstacles() {
    RMSDynamicObstaclesResponse res = api.getDynamicObstacles();
    if (res.response != null && res.response.obstacles != null) {
      log.debug("Found " + res.response.obstacles.length + " dynamic obstacle" + (res.response.obstacles.length == 1 ? "" : "s"));
      return Arrays.asList(res.response.obstacles);
    }
    log.debug("Found no dynamic obstacles");
    return new ArrayList<>();
  }

  @Override
  public boolean isDocked() {
    RMSDockStatusResponse res = api.getDockStatus();
    boolean docked = res.response.state.equals("Docked");
    log.debug("Dock status: [" + res.response.status + "] " + res.response.detail);
    return docked;
  }

  @Override
  public Justification lookAtObstacle(Obstacle obs) {
    return lookAt(Double.valueOf(obs.position.x), Double.valueOf(obs.position.y), 30);
  }

  @Override
  public Justification lookAt(double x, double y, double duration) {
    RMSResponse res = api.lookAt(x, y, cameraHeight, duration);
    log.debug("Looking at " + x + ", " + y);
    return new ConditionJustification(true);
  }

  private Map<String, Pair<Point3d, Quat4d>> getAllTags() {
    List<RMSListTagsResponse> resList = api.getAllTags();
    Map<String, Pair<Point3d, Quat4d>> toReturn = new HashMap<>();
    for (RMSListTagsResponse res : resList) {
      for (Tag tag : res.response.tags.values()) {
        if ("true".equals(tag.attributes.get("tracs"))) {
          Point3d point = new Point3d(parseDouble(tag.position.x), parseDouble(tag.position.y), 0);
          Quat4d quat4d = Convert.convertToQuat4d(parseDouble(tag.position.theta), 0, 0);
          toReturn.put(tag.name, new MutablePair<>(point, quat4d));
        }
      }
    }
    return toReturn;
  }

  private List<String> getDockTags() {
    List<RMSListTagsResponse> resList = api.getAllTags();
    List<String> toReturn = new ArrayList<>();
    for (RMSListTagsResponse res : resList) {
      for (Tag tag : res.response.tags.values()) {
        if ("Dock".equals(tag.attributes.get("TagType"))) {
            toReturn.add(tag.name);
          }
        }
      }
    return toReturn;
  }

  @Override
  public Justification stopFollow() {
    if (shouldFollow) {
      shouldFollow = false;
      return new ConditionJustification(true);
    } else {
      return new ConditionJustification(false, Factory.createPredicate("following()"));
    }
  }

  @Override
  public Justification follow() {
    long lastTargetFound = System.currentTimeMillis();
    long maxWaitTime = 10000;
    shouldFollow = true;
    Follower follower = new Follower();
    while (shouldFollow) {
      Pair<Point3d, Quat4d> currPose = getPose();
      Obstacle target = follower.getObstacleToFollow(currPose, getDynamicObstacles());
      if (target == null) {
        if (System.currentTimeMillis() - lastTargetFound > maxWaitTime) {
          shouldFollow = false;
          log.debug("No person found in 10 seconds. Terminating follow action.");
          return new ConditionJustification(false, Factory.createPredicate("found(person)"));
        }
      } else {
        lastTargetFound = System.currentTimeMillis();
        float[] vels = follower.calculateFollowVelocity(currPose, target);
        log.debug("Moving towards target obstacle with velocity t: {} r: {} ...", vels[0], vels[1]);
        api.setVelocity(vels[0], vels[1], 0);
        log.debug("... returned from moving towards target obstacle.");
      }
    }

    log.debug("[follow] has been stopped.");
    return new ConditionJustification(true);
  }

  /**
   * Returns the real robot drive status from the API. Useful for verifying what the robot has done
   *
   * @return the real drive status associated with the API
   */
  private DriveStatus getDriveStatus() {
    RMSDriveStatusResponse dsr = api.getDriveStatus();

    // Check if the response or its status is null
    if (dsr == null || dsr.response == null || dsr.response.status == null) {
      return DriveStatus.OTHER;
    }

    // Check individual status fields for null and their values
    if ("IN_PROGRESS".equals(dsr.response.status.GoToDock) && "IN_PROGRESS".equals(dsr.response.status.PayloadPose)) {
      return DriveStatus.GO_HOME;
    } else if ("IN_PROGRESS".equals(dsr.response.status.DockWithWaypoint)) {
      return DriveStatus.DOCK_WAYPOINT;
    } else if ("IN_PROGRESS".equals(dsr.response.status.DriveWaypoint)) {
      return DriveStatus.DRIVE_WAYPOINT;
    } else if ("COMPLETE".equals(dsr.response.status.DriveWaypoint)) {
      return DriveStatus.COMPLETE_DRIVE_WAYPOINT;
    } else if ("FAILED".equals(dsr.response.status.DriveWaypoint)) {
      return DriveStatus.FAILED_DRIVE_WAYPOINT;
    } else if ("None".equals(dsr.response.commandMode)) {
      return DriveStatus.NONE;
    } else {
      return DriveStatus.OTHER;
    }
  }

  /**
   * Real drive statuses associated with the API
   */
  private enum DriveStatus {
    GO_HOME,
    DOCK_WAYPOINT,
    DRIVE_WAYPOINT,
    COMPLETE_DRIVE_WAYPOINT,
    FAILED_DRIVE_WAYPOINT,
    OTHER,
    NONE,
  }

  //todo: this shouldn't live here
  @Override
  public Symbol parseTime(Symbol input, Symbol amOrPm) {
    String inputString = input.toString();
    if (!inputString.contains(":")) {
      inputString = new StringBuilder(inputString).insert(inputString.length() - 2, ':').toString();
    }
    try {
      LocalTime localTime = LocalTime.parse(inputString, DateTimeFormatter.ofPattern("H:m"));
      if (amOrPm.getName().strip().equals("pm")) {
        if (!localTime.isAfter(LocalTime.parse("12:00", DateTimeFormatter.ofPattern("H:m")))) {
          localTime = localTime.plusHours(12);
        }
      }
      return Factory.createSymbol(localTime.format(DateTimeFormatter.ofPattern("HH-mm-ss")));
    } catch (DateTimeParseException e) {
      log.error("[parseTime] error parsing time input: %s %s".formatted(input, amOrPm));
    }
    return Factory.createSymbol("none");
  }

  @Override
  public Integer getWifiSignalStrength() {
    String tmpFilePath = getProperty("user.home") + "/.diarc/tmp";
    String tmpFilePathExtra = "\"" + tmpFilePath + "\"";

    ProcessBuilder pb = new ProcessBuilder("awk", "NR==3 {printf(\"%.0f\\n\",$3*10/7) > " + tmpFilePathExtra + "}", "/proc/net/wireless");
    ProcessBuilder readerPb = new ProcessBuilder("cat", tmpFilePath);
    pb.inheritIO();
    String homeDirPath = getProperty("user.home");
    pb.directory(new File(homeDirPath));
    readerPb.directory(new File(homeDirPath));
    try {
      Process process = pb.start();
      process.waitFor();
      process = readerPb.start();
      String result = new String(process.getInputStream().readAllBytes()).replace("\"", "").strip();
      return Integer.parseInt(result);
    } catch (IOException | NumberFormatException e) {
      log.error("[getWifiSignalStrength] failed to get wifi signal strength with exception: ", e);
    } catch (InterruptedException e) {
      throw new RuntimeException(e);
    }
    return -1;
  }

  @Override
  public void blockToCharge() {
    log.info("[blockToCharge] blocking on chargingTerminatedCondition");
    try {
      chargingLock.lock();
      chargingTerminatedCondition.await();
    } catch (InterruptedException e) {
      log.error("[blockToCharge] condition await interrupted", e);
    } finally {
      chargingLock.unlock();
    }
  }

  @Override
  public void interruptChargingBlock() {
    log.info("[interruptChargingBlock] signaling chargingTerminatedCondition");
    chargingLock.lock();
    chargingTerminatedCondition.signal();
    chargingLock.unlock();
  }

  @TRADEService
  @Action
  public void triggerWW() {
    if (ww_process_os == null) {
      log.debug("[triggerWW] python subprocess input stream is null");
      return;
    }
    BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(ww_process_os));
    //TODO: Should this still trigger the WW ding?
    //TODO: Also should this have a longer ASR timeout than if triggered by voice?
    log.info("[triggerWW] in method");
    //shouldDing = false;
    try {
      writer.write("hello\n");
      writer.flush();
    } catch (IOException e) {
      log.error("[triggerWW] error signalling WW trigger");
    }
  }

  //TODO: Move back strictly to this component? Initially moved method to MaryTTS component while debugging to see if locking
  // and line management was part of the issue, but don't believe it is actually necessary
  //@TRADEService
  public void playAudioClip(String fileName) {
    log.info("[playAudioClip] calling method on MaryTTSComponent");
    String homeDir = System.getProperty("user.home");
    String path = Paths.get(homeDir, ".diarc", "ava", fileName).toString();
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("playAudioClip").argTypes(String.class)).call(void.class, path);
    } catch (TRADEException e) {
      log.debug("[playAudioClip] exception calling playAudioClip in MaryTTSComponent. Attempting locally", e);
      try {
        log.info("[playAudioClip] playing audio clip at path: " + path);
        File audioFile = new File(path);
        AudioInputStream audioStream = AudioSystem.getAudioInputStream(audioFile);

        AudioFormat format = audioStream.getFormat();
        Clip audioClip;
        if (mixerID == -1) {
          DataLine.Info info = new DataLine.Info(Clip.class, format);
          audioClip = (Clip) AudioSystem.getLine(info);
        } else {
          audioClip = AudioSystem.getClip(AudioSystem.getMixerInfo()[mixerID]);
        }
        audioClip.open(audioStream);
        FloatControl control = (FloatControl) audioClip.getControl(FloatControl.Type.MASTER_GAIN);
        control.setValue(20f * (float) Math.log10(0.3));

        AtomicBoolean isDone = new AtomicBoolean(false);
        Clip finalPlaybackClip = audioClip;
        audioClip.addLineListener(event -> {
          if (event.getType() == LineEvent.Type.STOP) {
            finalPlaybackClip.close();
            isDone.set(true);
          }
        });
        audioClip.loop(0);
        audioClip.start();

        while (!isDone.get()) {
          Util.Sleep(100);
        }
        audioStream.close();
      } catch (Exception ex) {
        log.error("[playAudioClip] error playing audio clip", ex);
      }
    }
  }
  @Override
  public void interruptListenForWWandASR() {
    listenForWW = false;
  }
  @Override
  public void listenForWWandASR() {
    listenForWW = true;
    //TODO: What of the below do we want to expose through the config?
    //args:
    // --model_size: "tiny", "base", "small", "medium", "large"
    // --ww_model_path: ~/.diarc/python/ww/models/hey_eyva_model_float32.tflite
    // --ww_detection_thresh: 0.8
    // --use_faster_whisper
    // --gpu
    // --asr_mode: 1 == local, 2 == remote
    // --endpoint: http://<ip>:5000/
    String home = System.getProperty("user.home");
    //ProcessBuilder processBuilder = new ProcessBuilder(home+"/.diarc/python/ww/venv/bin/python3.10", "-u", home+"/.diarc/python/ww/run_ww_and_asr.py", "--use_faster_whisper");

    // moonshine base cpu
    ProcessBuilder processBuilder = new ProcessBuilder(home+"/.diarc/python/ww/venv/bin/python3.10", "-u", home+"/.diarc/python/ww/run_ww_and_asr.py");

    // fasterwhisper small gpu
    //ProcessBuilder processBuilder = new ProcessBuilder(home+"/.diarc/python/ww/venv/bin/python3.10", "-u", home+"/.diarc/python/ww/run_ww_and_asr.py", "--model_size","small","--use_faster_whisper","--asr_mode","1","--gpu");

    // remote fasterwhisper small gpu
    //ProcessBuilder processBuilder = new ProcessBuilder(home+"/.diarc/python/ww/venv/bin/python3.10", "-u", home+"/.diarc/python/ww/run_ww_and_asr.py","--use_faster_whisper","--asr_mode","2","--endpoint", "http://localhost:5000/");

    //********If haven't updated ww env and script, use the below*******
    //ProcessBuilder processBuilder = new ProcessBuilder(home+"/.diarc/python/ww/venv/bin/python3.9", home+"/.diarc/python/ww/run_ww_and_asr.py", "--whisper_model", "base.en");

    processBuilder.redirectError(ProcessBuilder.Redirect.INHERIT); //For debugging
    try {
      Process process = processBuilder.start();
      InputStream is = process.getInputStream();
      ww_process_os = process.getOutputStream();
      String result = "";
      while (listenForWW) {
        while (result.isEmpty()) {
          //// Wake Word ////
          //Wait for WW detection
          while (is.available() == 0 && listenForWW) {
            Thread.sleep(100);
          }
          //WW detected, check if due to model trigger or external interrupt (mute or shutdown)
          if (!listenForWW) {
            break;
          }
          //Read message indicating WW detection from stream buffer
          StringBuilder s = new StringBuilder();
          while (is.available() != 0) {
            s.append((char) is.read());
          }
          log.info("[listenForWWandASR] reading from inputstream: " + s.toString());
          //Display on screen that we are now listening using ASR
          displayText("Understanding speech", 10000);
          //Also play audio indicating listening status
          if (shouldDing) {
            playAudioClip("ding.wav");
            try {
              TRADE.getAvailableService(new TRADEServiceConstraints().name("stopUtterance")).call(void.class);
            } catch (TRADEException e) {
              log.debug("[listenForWWAndASR] exception calling stopUtterance", e);
            }
          } else {
            shouldDing = true;
          }

          //// ASR ////
          //Wait for ASR result:
          log.info("waiting for ASR result");
          while (is.available() == 0) {
            Thread.sleep(100);
          }
          //Get ASR result (using <unk> to signal ASR timed out before any results were detected)
          s.setLength(0);
          while (is.available() != 0) {
            s.append((char) is.read());
          }
          result = s.toString();
        }

        //Parse ASR result - form: <prefix> <speech>
        //  <unk> == base failure case - ASR timed out before detecting any voice activity or was unable to transcribe anything
        //  <cutoff> == false positive ww detection heuristics triggered on python side - attempt to parse, but fail silently if so
        //  <full> == base success case  - have full ASR results
        boolean cutoff_result = false;
        //If ASR timed out, loop back to WW detection
        if (!result.isEmpty()) {
          //  TODO: Do we want this distinction between vad triggering without any transcription vs no vad at all?
          if (result.equals("<unk>\n") || result.equals("<novad>\n")) {
            if (result.equals("<unk>\n")) {
              // ASR couldn't parse, play low toned beep and loop to top
              playAudioClip("negativeASRResult.wav");
            }
            try {
              TRADEServiceInfo service = TRADE.getAvailableService(new TRADEServiceConstraints().name("clearText").argTypes());
              service.call(void.class);
            } catch (TRADEException e) {
              log.warn("[transcribeAndReportSpeech] Exception displaying text", e);
            }
            if (result.equals("<novad>\n")) {
              displayText("<No voice input detected>", 5000);
            }
            result = "";
            continue;
          } else if (result.startsWith("<cutoff> ")) {
            // ASR detected potential false positive - attempt to parse before handling
            result = result.substring(9);
            cutoff_result = true;
          } else {
            // Positive ASR detection, play high toned beep, display understood text, and attempt to parse
            if (result.startsWith("<full> ")) {
              result = result.substring(7);
            } else {
              log.warn("[listenForWWandASR] unknown or no result prefix detected, is run_ww_and_asr.py up to date?");
            }
            playAudioClip("positiveASRResult.wav");
            displayText("Recognized Speech: " + result, 5000);
          }

          // Reformat ASR result to prevent parser issues, create utterance
          result = result.trim().toLowerCase();
          result = result.replaceAll("\\p{Punct}", "");
          Utterance utterance = new Utterance.Builder()
                  .setSpeaker(speaker)
                  .setAddressee(Factory.createSymbol("ava"))
                  .setWords(Arrays.asList(result.trim().split(" ")))
                  .setUtteranceType(UtteranceType.UNKNOWN)
                  .setIsInputUtterance(true)
                  .build();

          // Experimental <cutoff> ww false positive effort //
          //Attempt to parse first, if input can't be parsed, then assume false positive ww detection in this case and do not
          // make noticeable to the user that something happened
          if (cutoff_result) {
            try {
              utterance = TRADE.getAvailableService(new TRADEServiceConstraints().name("parseUtterance").argTypes(Utterance.class)).call(Utterance.class, utterance);
            } catch (TRADEException ex) {
              log.error("[listenForWWAndASR] error calling parseUtterance", ex);
            }
            if (utterance.getType() == UtteranceType.UNKNOWN) {
              //TODO: How should this be phrased?
              displayText("<Input could not be understood and detected as potential background noise>", 10000);
              result = "";
              continue;
            } else {
              // Present successful parse as normal
              //TODO: how does this practically look when interacting with the robot?
              playAudioClip("positiveASRResult.wav");
              displayText("Recognized Speech: " + result, 5000);
            }
          }

          //Pass ASR result off to dialogue pipeline and loop to top to continue listening for further input
          try {
            TRADE.getAvailableService(new TRADEServiceConstraints().name("reportRecognizedSpeech").argTypes(Utterance.class)).call(void.class, utterance);
            log.info("Reported utterance \"" + result + "\"");
          } catch (TRADEException ex) {
            log.warn("Failed to report speech \"" + result + "\"");
          }

          result = "";
        }
      }
      is.close();
      ww_process_os.close();
      process.destroy();
      //String result = new String(is.readAllBytes());
      //int exitCode = process.waitFor();
      //log.info(String.valueOf(exitCode));
      //log.info(result);
    } catch (IOException | InterruptedException e) {
      log.error("error running process",e);
    }
  }

  private void displayText(String text, int duration) {
    try {
      TRADEServiceInfo service = TRADE.getAvailableService(new TRADEServiceConstraints().name("displayText").argTypes(String.class));
      service.call(void.class,text);
    } catch (TRADEException e) {
      log.warn("[transcribeAndReportSpeech] Exception displaying text", e);
    }
    new Thread(() -> {
      try {
        Thread.sleep(duration);
      } catch (InterruptedException e) {
        log.error("[listenForWWandASR] exception sleeping in clearDisplay thread");
      }
      try {
        TRADEServiceInfo service = TRADE.getAvailableService(new TRADEServiceConstraints().name("clearText").argTypes());
        service.call(void.class);
      } catch (TRADEException e) {
        log.warn("[transcribeAndReportSpeech] Exception displaying text", e);
      }
    }).start();
  }

  @Override
  public void waitForReadyAck() {
    String home = System.getProperty("user.home");
    ProcessBuilder processBuilder = new ProcessBuilder(home+"/.diarc/python/ww/venv/bin/python3.9", "-u", home+"/.diarc/python/ww/run_ready_ww.py");
    processBuilder.redirectError(ProcessBuilder.Redirect.INHERIT); //For debugging
    try {
      Process process = processBuilder.start();
      InputStream is = process.getInputStream();
      ready_process_os = process.getOutputStream();
      try {
        TRADEServiceInfo service = TRADE.getAvailableService(new TRADEServiceConstraints().name("displayText").argTypes(String.class));
        service.call(void.class, "Please say \"ready\" when you are ready to go");
      } catch (TRADEException e) {
        log.warn("[transcribeAndReportSpeech] Exception displaying text", e);
      }
      //Wait for WW detection
      while (is.available() == 0) {
        Thread.sleep(100);
      }
      //Clear stream buffer of message indicating WW detection
      StringBuilder s = new StringBuilder();
      while (is.available() != 0) {
        s.append((char) is.read());
      }
      if (shouldDingReady) {
        playAudioClip("positiveASRResult.wav");
      }
      shouldDingReady = true;

      try {
        TRADEServiceInfo service = TRADE.getAvailableService(new TRADEServiceConstraints().name("clearText").argTypes());
        service.call(void.class);
      } catch (TRADEException e) {
        log.warn("[transcribeAndReportSpeech] Exception displaying text", e);
      }
      is.close();
      ready_process_os.close();
      process.destroy();
    } catch (IOException | InterruptedException e) {
      log.error("error running process", e);
    }
    ready_process_os = null;
  }

  @Override
  public void interruptWaitForReadyAck() {
    shouldDingReady = false;
    if (ready_process_os == null) {
      log.debug("[interruptWaitForReadyAck] python subprocess input stream is null");
      return;
    }
    BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(ready_process_os));
    try {
      writer.write("hello\n");
      writer.flush();
    } catch (IOException e) {
      log.error("[interruptWaitForReadyAck] error signalling WW trigger");
    }
  }

  @Action
  @TRADEService
  public void interruptWaitForLearningGoalSubmissionOrQuestionAnswer(Symbol agent, Predicate learnedActionPredicate) {
    //Should only have to cancel one inner action of the two threads the action is waiting one - the action itself should cancel the other one
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("interruptWaitForLearningStateUpdate").argTypes(Symbol.class,Predicate.class)).call(void.class,agent,learnedActionPredicate);
    } catch (TRADEException e) {
      log.error("[interruptWaitForLearningGoalSubmissionOrQuestionAnswer] Exception canceling waiting for learning state change to unblock primitive", e);
    }
  }

  //TODO: where should this live? Kind of an abomination of an action which shouldn't exist in the first place and
  //      touches both action and dialogue. Placing here because it is ava demo specific for now?
  @OnInterrupt(
          onCancelServiceCall = "interruptWaitForLearningGoalSubmissionOrQuestionAnswer(?agent,?learnedActionPredicate)",
          onSuspendServiceCall = "interruptWaitForLearningGoalSubmissionOrQuestionAnswer(?agent,?learnedActionPredicate)"
  )
  @Action
  @TRADEService
  public Map<Variable, Symbol> waitForLearningGoalSubmissionOrQuestionAnswer(Symbol agent, String questionString, List<Term> responseForms, Predicate learnedActionPredicate) {
    Semaphore threadJoinSem = new Semaphore(0);
    ExecutorService questionAskingExecutor = Executors.newSingleThreadExecutor();
    Future<HashMap<Variable,Symbol>> future = questionAskingExecutor.submit(() -> {
      HashMap<Variable,Symbol> resp = null;
      try {
        resp = TRADE.getAvailableService(new TRADEServiceConstraints().name("waitForResponse").argTypes(String.class, List.class, Symbol.class)).call(HashMap.class,questionString, responseForms, agent);
      } catch (TRADEException e) {
        log.error("[waitForLearningGoalSubmissionOrQuestionAnswer] Exception waiting for question response", e);
      }
      threadJoinSem.release();
      return resp;
    });

    final Boolean[] updateReceived = new Boolean[1];
    Thread learningStateThread = new Thread(() -> {
      try {
        updateReceived[0] = TRADE.getAvailableService(new TRADEServiceConstraints().name("waitForLearningStateUpdate").argTypes(Symbol.class, Predicate.class)).call(Boolean.class,agent,learnedActionPredicate);
      } catch (TRADEException e) {
        log.error("[waitForLearningGoalSubmissionOrQuestionAnswer] Exception waiting for learning state change", e);
      }
      threadJoinSem.release();
    });
    learningStateThread.start();

    try {
      threadJoinSem.acquire();
    } catch (InterruptedException e) {
      log.error("[waitForLearningGoalSubmissionOrQuestionAnswer] exception waiting for one of the two conditions to occur", e);
      return null;
    }

    if (future.isDone()) {
      HashMap<Variable, Symbol> questionResponse;
      try {
        questionResponse = future.get();
      } catch (InterruptedException | ExecutionException e) {
        log.error("[waitForLearningGoalSubmissionOrQuestionAnswer] exception getting question asking response bindings",e);
        return null;
      }
      try {
        TRADE.getAvailableService(new TRADEServiceConstraints().name("interruptWaitForLearningStateUpdate").argTypes(Symbol.class,Predicate.class)).call(void.class,agent,learnedActionPredicate);
      } catch (TRADEException e) {
        log.error("[waitForLearningGoalSubmissionOrQuestionAnswer] Exception canceling waiting for learning state change", e);
      }
      learningStateThread.interrupt();
      return questionResponse;
    } else if (!learningStateThread.isAlive()) {
      try {
        TRADE.getAvailableService(new TRADEServiceConstraints().name("cancelWaitForResponse").argTypes(String.class, List.class, Symbol.class)).call(Map.class,questionString, responseForms, agent);
      } catch (TRADEException e) {
        log.error("[waitForLearningGoalSubmissionOrQuestionAnswer] Exception canceling waiting for question response", e);
      }
      questionAskingExecutor.shutdownNow();
      if (!updateReceived[0]) {
        log.warn("[waitForLearningGoalSubmissionOrQuestionAnswer] learning state listener interrupted but no question answer was detected");
        return null;
      }
      return new HashMap<>();
    } else {
      log.error("[waitForLearningGoalSubmissionOrQuestionAnswer] semaphore signalled but neither thread terminated?");
      return null;
    }
  }

  @Override
  public void waitForDuration(long seconds) {
    log.info("[waitForDuration] sleeping for {} seconds", seconds);
    try {
      sleepForDurationThread = Thread.currentThread();
      Thread.sleep(seconds*1000);
    } catch (InterruptedException ignored) {}
    sleepForDurationThread = null;
    log.info("[waitForDuration] done sleeping");
  }

  @Override
  public void interruptWaitForDuration() {
    if (sleepForDurationThread != null) {
      sleepForDurationThread.interrupt();
    }
  }

  @Override
  public void enableAdminMode() {
    speaker = Factory.createSymbol("local_user");
  }

  @Override
  public void disableAdminMode() {
    speaker = Factory.createSymbol("local_visitor");
  }

  @TRADEService
  public void toggleDisplay() {
    String homeDir = System.getProperty("user.home");
    String scriptDir = Paths.get(homeDir, ".diarc").toString();
    String statusCommand = "/bin/bash " + scriptDir + "/getDisplay.sh";
    log.info("[toggleDisplay] status command: " + statusCommand);
    String statusResult = callLocal(statusCommand);
    log.info("[toggleDisplay] status result: " + statusResult);


    DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
    DocumentBuilder builder = null;
    try {
      builder = factory.newDocumentBuilder();
    } catch (ParserConfigurationException e) {
      throw new RuntimeException(e);
    }
    Document doc = null;
    try {
      doc = builder.parse(
              new ByteArrayInputStream(statusResult.getBytes("UTF-8"))
      );
    } catch (SAXException e) {
      throw new RuntimeException(e);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    String value;
    NodeList nodes = doc.getElementsByTagName("MainVideoSource");
    value = nodes.item(0).getTextContent();

    int id = Integer.parseInt(value);
    log.info("[toggleDisplay] id of current display input: " + id);
    String command;
    if (id == 1) {
      command = "/bin/bash " + scriptDir + "/setDisplay2.sh";
    } else {
      command = "/bin/bash " + scriptDir + "/setDisplay1.sh";
    }
    String result = callLocal(command);
    log.info("[toggleDisplay] result: " + result);
    //TODO: what is proper error handling/result when switch failed?
    haveDisplay = !haveDisplay;
  }
  @TRADEService
  public boolean getHaveDisplay() {
    return haveDisplay;
  }

  private String callLocal(String command) {
    StringBuilder result = new StringBuilder();
    String s;
    try {
      Process p = Runtime.getRuntime().exec(command);
      BufferedReader br = new BufferedReader(
              new InputStreamReader(p.getInputStream()));
      while ((s = br.readLine()) != null)
        result.append(s);
      p.waitFor();
      log.info("[callLocal] p: " + p);
      p.destroy();
    } catch (Exception e) {
      log.error("[showTRACS] error changing display input");
    }
    return result.toString();
  }

}
