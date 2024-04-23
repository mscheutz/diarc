/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.movebase;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.consultant.pose.PoseConsultant;
import edu.tufts.hrilab.consultant.pose.PoseReference;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.interfaces.MoveBaseInterface;
import edu.tufts.hrilab.map.PathAction;
import edu.tufts.hrilab.diarcros.common.Amcl;
import edu.tufts.hrilab.diarcros.common.RosConfiguration;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Pose;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Point;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Quaternion;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.HashMap;

import edu.tufts.hrilab.diarcros.util.Convert;
import edu.tufts.hrilab.util.RotationHelpers;
import edu.tufts.hrilab.util.SimpleGeometry;
import edu.tufts.hrilab.util.Util;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Tuple3d;

/**
 * Provides simplified access to the ROS move base node.
 * <p>
 * This interface is convenient when you want to be able to send
 * navigation goals without tracking their progress.
 *
 * @author Jeremiah Via <jeremiah.via@gmail.com>
 */
public class MoveBaseComponent extends DiarcComponent implements MoveBaseInterface {
  private RosConfiguration rc = new RosConfiguration();
  private MoveBase base;
  private Amcl amcl;

  private String mapFrame = "map";
  private String baseFrame = "base_link";

  private double pointDistThresh = .25;
  private double orientationThresh = .40;

  private MoveTowardsEnabler moveTowardsEnabler;

  // TODO: remove this
  private String locationsFile;

  private boolean hasInitPose = false;
  private double[] initPose;

  public MoveBaseComponent() {
    super();
  }

  @Override
  protected void init() {

    // Convert known locations to refs json file
    Map<String, Pose> knownLocations = new HashMap<>();
    if (locationsFile != null && !locationsFile.isEmpty()) {
      try {
        FileReader fileReader = new FileReader(locationsFile);
        BufferedReader bufferedReader = new BufferedReader(fileReader);
        String line;
        while ((line = bufferedReader.readLine()) != null) {
          String parts1[] = line.split(":");
          String parts2[] = parts1[1].split(" ");
          knownLocations.put(parts1[0].replace(" ", ""), new Pose(new Point(Float.parseFloat(parts2[0]), Float.parseFloat(parts2[1]), Float.parseFloat(parts2[2])),
                  new Quaternion(Float.parseFloat(parts2[3]), Float.parseFloat(parts2[4]), Float.parseFloat(parts2[5]), Float.parseFloat(parts2[6]))));
        }

        bufferedReader.close();
      } catch (IOException e) {
        log.error("Failed to read locations file: " + locationsFile, e);
      }

      String kbName = "location";
      PoseConsultant consultant = new PoseConsultant(PoseReference.class, kbName, new ArrayList<>());
      knownLocations.forEach((k, v) -> {
        PoseReference ref = consultant.createReference(Factory.createVariable("VAR0", kbName), List.of(Factory.createPredicate(k, Factory.createVariable("VAR0", kbName))));
        ref.setPose(Convert.convertToPoint3d(v.getPosition()), Convert.convertToQuat4d(v.getOrientation()));
      });

      consultant.writeReferencesToFile("knownLocations.json");
      log.warn("Known locations written to a new file: knownLocations.json. Use this file in a MapComponent. The locations will not be used here!");
    }

    base = new MoveBase(rc, mapFrame, baseFrame);
    amcl = new Amcl();
    amcl.waitForNode();
    moveTowardsEnabler = new MoveTowardsEnabler();

    if (hasInitPose) {
      updateAmclPose(initPose[0], initPose[1], initPose[2]);
    }

    log.debug("All set up! Connected at namespace: " + rc.namespace);
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("locations").hasArg().argName("file").desc("DEPRECATED: use -refs instead.").build());
    options.add(Option.builder("map").longOpt("map_frame").hasArg().argName("map_frame_id").desc("ROS tf robot base frame (default: base_link").build());
    options.add(Option.builder("base").longOpt("base_frame").hasArg().argName("base_frame_id").desc("ROS tf robot map frame (default: map)").build());
    options.add(Option.builder("ns").longOpt("namespace").hasArg().argName("namespace").desc("Set a ROS namespace pointing to this ROS component. Default: empty.").build());
    options.add(Option.builder("tf_prefix").hasArg().argName("namespace").desc("Set a tf prefix. Note that this appears similar to a namespace, but ROS namespaces do not impact tf tree links (making this parameter necessary). Read the TF docs for more detail. Default: empty.").build());
    options.add(Option.builder("rosmasteruri").hasArg().argName("rosmasteruri").desc("Override ROS_MASTER_URI environment variable").build());
    options.add(Option.builder("point_dist_thresh").hasArg().argName("point_distance_threshold").desc("Set the point distance threshold (default: 0.25)").build());
    options.add(Option.builder("pose").longOpt("initialPose").numberOfArgs(3).argName("x y theta").desc("Initial seed pose for localizer").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("locations")) {
      log.error("Using locations is no longer supported. Use -refs in the MapComponent instead.");
      locationsFile = cmdLine.getOptionValue("locations");
    }
    if (cmdLine.hasOption("base")) {
      baseFrame = cmdLine.getOptionValue("base");
    }
    if (cmdLine.hasOption("map")) {
      mapFrame = cmdLine.getOptionValue("map");
    }
    // TODO @cst-- i'm guessing these same CLI parameters will come up in other ROS-based components. move to RosConfig class?
    if (cmdLine.hasOption("namespace")) {
      rc.namespace = cmdLine.getOptionValue("namespace");
    }
    if (cmdLine.hasOption("tf_prefix")) {
      rc.tfPrefix = cmdLine.getOptionValue("tf_prefix");
      baseFrame = rc.getPrefixedFrame(baseFrame);
    }
    if (cmdLine.hasOption("rosmasteruri")) {
      String uri = cmdLine.getOptionValue("rosmasteruri");
      rc.setUriFromString(uri);
    }
    if (cmdLine.hasOption("point_dist_thresh")) {
      pointDistThresh = Float.parseFloat(cmdLine.getOptionValue("point_dist_thresh"));
    }
    if (cmdLine.hasOption("initialPose")) {
      String[] values = cmdLine.getOptionValues("initialPose");
      initPose = new double[3];
      initPose[0] = Double.parseDouble(values[0]);
      initPose[1] = Double.parseDouble(values[1]);
      initPose[2] = Double.parseDouble(values[2]);
      hasInitPose = true;
    }
  }

  private void updateAmclPose(double x, double y, double theta) {
    edu.tufts.hrilab.diarcros.msg.geometry_msgs.Point xyz = new edu.tufts.hrilab.diarcros.msg.geometry_msgs.Point(x, y, 0);
    edu.tufts.hrilab.diarcros.msg.geometry_msgs.Quaternion orientation = Convert.eulerToQuaternion(theta, 0, 0);
    edu.tufts.hrilab.diarcros.msg.geometry_msgs.Pose pose = new edu.tufts.hrilab.diarcros.msg.geometry_msgs.Pose(xyz, orientation);

    double[] covariance = new double[36]; // covariance is a 6x6 matrix indicating our confidence in this pose
    // because we want AMCL to be able to update our pose, we don't want to pass 0's (since this would imply 0 error, perfect knowledge)
    // so instead, we need to set the values on the diagonal to something non-zero, we'll use 0.1
    // diagonals would be (0,0) -> 0, (1,1) -> 7, (2,2) -> 14, (3,3) -> 21...
    for (int i = 0; i < 36; i++) {
      if (i % 7 == 0) {
        covariance[i] = 0.1;
      }
    }
    edu.tufts.hrilab.diarcros.msg.geometry_msgs.PoseWithCovariance pwc = new edu.tufts.hrilab.diarcros.msg.geometry_msgs.PoseWithCovariance(pose, covariance);
    edu.tufts.hrilab.diarcros.msg.geometry_msgs.PoseWithCovarianceStamped pwcs = new edu.tufts.hrilab.diarcros.msg.geometry_msgs.PoseWithCovarianceStamped(pwc);
    amcl.sendInitialpose(pwcs);
  }

  protected Pose getPose() {
    return base.getPose();
  }

  /**
   * Used to updated Firebase GUI
   *
   * @return
   */
  @TRADEService
  public Point3d getPosition() {
    Pose p = getPose();
    Point point = p.getPosition();

    log.debug("[getPosition] executing getting position for firebase agent location" + " " + p);
    log.debug("[getPosition] executing getting position for firebase agent location" + " " + point.getX() + " " + point.getY() + " " + point.getZ());

    return new Point3d(point.getX(), point.getY(), point.getZ());
  }


  //////////////////////////////////////////////////////////////////////
  // Position Interface
  //////////////////////////////////////////////////////////////////////

  @Override
  public void setPoseGlobal(double x, double y, double theta) {
    updateAmclPose(x, y, theta);
  }

  //////////////////////////////////////////////////////////////////////
  // MoveBaseComponent Interface
  //////////////////////////////////////////////////////////////////////

  @Override
  public double[] getPoseGlobalQuat() {
    Pose pose = getPose();
    return new double[]{pose.getPosition().getX(), pose.getPosition().getY(),
            pose.getOrientation().getX(), pose.getOrientation().getY(),
            pose.getOrientation().getZ(), pose.getOrientation().getW()};
  }

  /**
   * Go to a pose in the world.
   *
   * @param pose a goal pose
   */
  protected Justification goToLocation(final Pose pose, boolean wait) {
    return base.goToLocation(pose, wait);
  }

  /**
   * Go to a known location in the world.
   *
   * @param location a known location.
   * @param wait     should wait until movement is complete
   */
  @Override
  public Justification goToLocation(Symbol location, boolean wait) {
    // TODO: make these command line args?
    double padding = 1.0;
    boolean canOpenDoors = false;

    List<PathAction> path;
    try {
      path = TRADE.getAvailableService(new TRADEServiceConstraints().name("getPath")).call(List.class, Convert.convertToMatrix4d(getPose()), location, padding, canOpenDoors);
    } catch (TRADEException e) {
      log.error("Error trying to get path for location: " + location);
      log.error("ERROR", e);
      return new ConditionJustification(false, Factory.createPredicate("path_to", location.getName(), "known"));
    }

    if (path.isEmpty()) {
      log.error("Empty path to location: " + location);
      return new ConditionJustification(false, Factory.createPredicate("path_to", location.getName(), "known"));
    }
    return followPath(path);
  }

  @Override
  public Justification goToLocation(Symbol location) {
    return goToLocation(location, true);
  }

  @Override
  public Justification goToLocation(Symbol desiredLocation, Symbol initialLocation) {
    return goToLocation(desiredLocation, true);
  }

  @Override
  public Justification goToLocation(double xdest, double ydest, double quat_x, double quat_y, double quat_z, double quat_w, boolean wait) {
    if (getPose() == null) {
      log.warn("Robot pose has not been initialized. Navigation will likely fail");
    }
    Pose pose = new Pose(new Point(xdest, ydest, 0), new Quaternion(quat_x, quat_y, quat_z, quat_w));
    return goToLocation(pose, wait);
  }

  @Override
  public Justification approachLocation(Symbol desiredLocation) {
    goToLocation(desiredLocation);
    return moveTowardsEnabler.moveTowards();
  }

  @Override
  public Justification approachLocation(Symbol desiredLocation, Symbol initialLocation) {
    goToLocation(desiredLocation, initialLocation);
    return moveTowardsEnabler.moveTowards();
  }

  @Override
  public Justification stop() {
    return base.stop();
  }

  @Override
  public Justification isMoving() {
    return base.isMoving();
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
    List<Symbol> args = locationTerm.getArgs();
    Pose locationPose = getLocationPose(args.get(1));
    ArrayList<Map<Variable, Symbol>> returnVal = new ArrayList<>();
    if (locationPose != null) {
      Point currentPoint = getPose().getPosition();
      Point goalPoint = locationPose.getPosition();
      log.debug("currentPoint " + currentPoint.getX() + " " + currentPoint.getY() + " " + currentPoint.getZ());
      log.debug("goalPoint " + goalPoint.getX() + " " + goalPoint.getY() + " " + goalPoint.getZ());
      double dist = Util.getDistanceFrom(currentPoint.getX(), currentPoint.getY(), goalPoint.getX(), goalPoint.getY());
      log.debug("offset " + pointDistThresh);
      log.debug("distance " + dist);
      Quaternion qt = locationPose.getOrientation();
      Quat4d qt4d = new Quat4d(qt.getX(), qt.getY(), qt.getZ(), qt.getW());
      Tuple3d angles = RotationHelpers.quatToXYZRotations(qt4d);
      log.debug("goalAngle " + angles.getZ());

      Quaternion qtCurrent = getPose().getOrientation();
      Quat4d qtCurrent4d = new Quat4d(qtCurrent.getX(), qtCurrent.getY(), qtCurrent.getZ(), qtCurrent.getW());
      Tuple3d anglesCurrent = RotationHelpers.quatToXYZRotations(qtCurrent4d);
      double angleDiff = SimpleGeometry.diffAngle(angles.getZ(), anglesCurrent.getZ());
      log.debug("currentAngle " + anglesCurrent.getZ());
      log.debug("angle thresh " + orientationThresh);
      log.debug("angle distance " + angleDiff);

      if (dist < pointDistThresh && Math.abs(angleDiff) < orientationThresh) {
        returnVal.add(new HashMap<>());
      } else {
        log.debug("[checkAt] not at location");
      }
    }
    return returnVal;
  }

//  FOR REFERENCE:
//  uint8 PENDING         = 0   # The goal has yet to be processed by the action server
//  uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
//  uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
//                            #   and has since completed its execution (Terminal State)
//  uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
//  uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
//                            #    to some failure (Terminal State)
//  uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
//          #    because the goal was unattainable or invalid (Terminal State)
//  uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
//                            #    and has not yet completed execution
//  uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
//                            #    but the action server has not yet confirmed that the goal is canceled
//  uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
//                            #    and was successfully cancelled (Terminal State)
//  uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
//                            #    sent over the wire by an action server

  protected Pose getLocationPose(Symbol refID) {
    Matrix4d transform = null;
    try {
      transform = TRADE.getAvailableService(new TRADEServiceConstraints().name("getEntityForReference").argTypes(Symbol.class,Class.class)).call(Matrix4d.class, refID, Matrix4d.class);
    } catch (TRADEException e) {
      log.error("[getLocationPose] exception getting Matrix4d from reference, returning null", e);
    }

    // location is a reference
    Pose pose = null;
    if (transform != null) {
      pose = Convert.convertToPose(transform);
    } else {
      log.error("[getLocationPose] null pose returned for reference: " + refID);
    }
    return pose;
  }

  protected Justification followPath(List<PathAction> path) {
    for (PathAction nextAction : path) {
      switch (nextAction.getType()) {
        case ACTION_GOTO: {
          Pose p = Convert.convertToPose(nextAction.getPose());
          Justification result = goToLocation(p, true);
          if (!result.getValue()) {
            return result;
          }
        }
        break;
        case ACTION_CHECK_DOOR: {
          // TODO: check for open door
          log.warn("Cannot currently check for open doors. Attempting to navigate to next location in path.");
//            boolean doorOpen = (Boolean) TRADE.callThe("isFacingOpenDoor");
//            if (doorOpen) {
//              log.debug("no obstacle, continuing on");
//              TRADE.callThe("markDoor", nextAction.getReference(), false);
//              // TRADE.callThe("forwardMove", 0.3, 1.0, this);
//            } else {
//              TRADE.callThe("markDoor", nextAction.getReference(), true);
//              log.debug("door closed :(");
//              return new ConditionJustification(false, Factory.createPredicate("open", nextAction.getReference()));
//            }
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
}
