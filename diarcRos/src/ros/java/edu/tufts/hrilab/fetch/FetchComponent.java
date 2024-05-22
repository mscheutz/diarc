/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.fetch;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;

import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.moveit.MoveItComponent;
import edu.tufts.hrilab.diarcros.fetch.HeadTrajControllerPointHeadAction;
import edu.tufts.hrilab.diarcros.fetch.TorsoControllerFollowJointTrajectoryActionNode;
import edu.tufts.hrilab.diarcros.fetch.RobotDriver;
import edu.tufts.hrilab.diarcros.msg.control_msgs.FollowJointTrajectoryGoal;
import edu.tufts.hrilab.diarcros.msg.control_msgs.PointHeadGoal;
import edu.tufts.hrilab.diarcros.msg.Duration;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Vector3;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Point;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.PointStamped;
import edu.tufts.hrilab.diarcros.msg.power_msgs.BatteryState;
import edu.tufts.hrilab.diarcros.msg.power_msgs.BreakerState;
import edu.tufts.hrilab.diarcros.msg.power_msgs.BreakerCommandRequest;
import edu.tufts.hrilab.diarcros.msg.power_msgs.BreakerCommandResponse;
import edu.tufts.hrilab.diarcros.msg.fetch_driver_msgs.RobotState;
import edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyRequest;
import edu.tufts.hrilab.diarcros.msg.std_srvs.EmptyResponse;
import edu.tufts.hrilab.diarcros.msg.trajectory_msgs.JointTrajectory;
import edu.tufts.hrilab.diarcros.msg.trajectory_msgs.JointTrajectoryPoint;
import edu.tufts.hrilab.vision.stm.MemoryObject;
import edu.tufts.hrilab.util.Util;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.ros.actionlib.state.SimpleClientGoalState;
import org.ros.exception.RosException;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.vecmath.Quat4d;
import javax.vecmath.Point3d;

public class FetchComponent extends MoveItComponent implements FetchInterface {

  private final HeadTrajControllerPointHeadAction head;
  private final TorsoControllerFollowJointTrajectoryActionNode torso;
  private final RobotDriver robotDriver;
  private boolean initHeadPoseSet = false;
  private Point3d initHeadPose;
  private boolean initTorsoHeightSet = false;
  private float initTorsoHeight;

  public FetchComponent() {
    super();

    head = new HeadTrajControllerPointHeadAction();
    torso = new TorsoControllerFollowJointTrajectoryActionNode();
    robotDriver = new RobotDriver();

    head.waitForNode();
    torso.waitForNode();
    robotDriver.waitForNode();

    configName = "Fetch.json";
  }

  @Override
  protected void init() {
    super.init();

    if (initHeadPoseSet) {
      pointHeadTo(initHeadPose);
    }
    if (initTorsoHeightSet) {
      setTorsoPosition(initTorsoHeight);
    }
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("head").longOpt("startHeadPose").numberOfArgs(3).argName("x y z").desc("Set initial head pose (x,y,z in base frame).").build());
    options.add(Option.builder("torso").longOpt("startTorsoHeight").hasArg().argName("height").desc("Set initial torso height (meters).").build());
    options.addAll(super.additionalUsageInfo());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    super.parseArgs(cmdLine);

    if (cmdLine.hasOption("startHeadPose")) {
      initHeadPoseSet = true;
      String[] values = cmdLine.getOptionValues("startHeadPose");
      initHeadPose = new Point3d(Double.parseDouble(values[0]), Double.parseDouble(values[1]), Double.parseDouble(values[2]));
    }
    if (cmdLine.hasOption("startTorsoHeight")) {
      initTorsoHeightSet = true;
      initTorsoHeight = Float.parseFloat(cmdLine.getOptionValue("startTorsoHeight"));
    }
  }

  @Override
  public boolean pointHeadTo(MemoryObject target_object) {
    //log.debug("[pointHeadTo] object method entered.");

    // transform to base_link coordinate frame
    target_object.transformToBase();

    // point head to center of object
    return pointHeadTo(target_object.getLocation());
  }

  @Override
  public boolean pointHeadTo(Symbol objectRef) {
    // get Point3d from object ref
    Point3d location = null;
    try {
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getEntityForReference").argTypes(Symbol.class, Class.class));
      location = tsi.call(Point3d.class, objectRef, Point3d.class);
    } catch (TRADEException e) {
      log.error("[pointTo] exception getting Point3d from reference, returning null", e);
    }

    if (location != null) {
      return pointHeadTo(location);
    }
    return false;
  }

  @Override
  public boolean pointHeadTo(Point3d targetPosition) {
    return pointHeadTo(targetPosition, 0.25);
  }

  private boolean pointHeadTo(Point3d targetPosition, double velocity) {
    log.debug("[pointHeadTo] method entered.");

    PointStamped pos = new PointStamped();
    pos.getHeader().setFrameId(moveItConfig.baseLinkString);
    pos.setPoint(new Point(targetPosition.x, targetPosition.y, targetPosition.z));
    PointHeadGoal headGoal = new PointHeadGoal();
    headGoal.setTarget(pos);
    headGoal.setPointingFrame("head_camera_link");
    Vector3 pointingAxis = new Vector3(1.0, 0.0, 0.0);
    headGoal.setPointingAxis(pointingAxis);
    headGoal.setMaxVelocity(velocity); // rad/s

    SimpleClientGoalState goalState = null;
    boolean result = false;
    for (int i = 0; i < 5; i++) {
      try {
        head.sendGoal(headGoal);
        //head.waitForResult(10, TimeUnit.SECONDS);
        long t0 = System.currentTimeMillis();
        long threshold = 10000l;
        while (threshold > (System.currentTimeMillis() - t0)) {
          goalState = head.getState();
          if (goalState != null && goalState.getState() != SimpleClientGoalState.StateEnum.ACTIVE && goalState.getState() != SimpleClientGoalState.StateEnum.PENDING) {
            break;
          }
          Thread.sleep(10);
        }
      } catch (InterruptedException | RosException e) {
        log.error("pointHeadTo", e);
        return false;
      }

      if (goalState != null && goalState.getState() != SimpleClientGoalState.StateEnum.PREEMPTED) {
        result = (goalState.getState() == SimpleClientGoalState.StateEnum.SUCCEEDED);
        break;
      }
      //boolean result = (goalState.getState() == SimpleClientGoalState.StateEnum.SUCCEEDED);
      //log.debug("[pointHeadTo] has not terminated yet");
    }
    if (goalState != null) {
      //log.debug(String.format("[pointHeadTo] finished with goal state: %s.", goalState.getState().toString()));
    }
    return result;
  }

  @Override
  public boolean setTorsoPosition(double position) {
    List<JointTrajectoryPoint> trajPoints = new ArrayList<>();
    trajPoints.add(new JointTrajectoryPoint(new double[]{position}, new double[]{0}, new double[]{0}, new double[]{1}, new Duration(5)));
    JointTrajectory traj = new JointTrajectory();
    traj.setJointNames(Arrays.asList("torso_lift_joint"));
    traj.setPoints(trajPoints);
    FollowJointTrajectoryGoal trajGoal = new FollowJointTrajectoryGoal();
    trajGoal.setTrajectory(traj);
    try {
      torso.sendTorsoControlMsgsFollowJointTrajectoryGoal(trajGoal);
      torso.waitForTorsoControlMsgsFollowJointTrajectoryResult();
    } catch (RosException | InterruptedException e) {
      log.error("Error setting torso position.", e);
    }
    SimpleClientGoalState state = torso.getTorsoControlMsgsFollowJointTrajectoryState();
    return (state.getState() == SimpleClientGoalState.StateEnum.SUCCEEDED);
  }

  @Override
  public Justification lookAround() {
    try {
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("pauseCapture"));
      tsi.call(void.class);

      moveGroup.callClearOctomap(new EmptyRequest(), new EmptyResponse());
      for (int i = 0; i < 1; i++) {
        log.debug("moving head " + i);
        pointHeadTo(new Point3d(0, -.5, 0), 90);
        pointHeadTo(new Point3d(0, .5, 0), 90);
        pointHeadTo(new Point3d(0.5, 0, 0), 90); // center down
        pointHeadTo(new Point3d(0.5, 0, 2), 90); // center up
        pointHeadTo(new Point3d(0.5, 0, .6), 90);
        log.debug("done moving head " + i);
      }
      Util.Sleep(500); // wait for head to stop moving
      tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("resumeCapture"));
      tsi.call(void.class);

    } catch (TRADEException e) {
      return new ConditionJustification(false);
    }

    return new ConditionJustification(true);
  }

  @Override
  public Justification moveTo(String group_name, Symbol objectRef) {
    lookAround();
    return super.moveTo(group_name, objectRef);
  }

  @Override
  public List<Map<Variable, Symbol>> checkGrasping(Term graspingTerm) {
    boolean closed = closeGripper("arm").getValue();
    List<Map<Variable, Symbol>> returnVal = new ArrayList<>();
    if (!closed) {
      returnVal.add(new HashMap<>());
    }
    return returnVal;
  }

  @Override
  public Justification moveObject(Symbol objectRef, String arm, String direction) {
    log.debug("Starting moveObject " + direction);
    Point3d relPoint = new Point3d(0, 0, 0);
    switch (direction) {
      case "up":
        relPoint = new Point3d(0.0, 0.0, 0.15);
        break;
      case "down":
        relPoint = new Point3d(0.0, 0.0, -0.10);
        break;
      case "forward":
        relPoint = new Point3d(0.30, 0.0, 0.0);
        break;
      case "right":
        relPoint = new Point3d(0.0, -0.30, 0.0);
        break;
    }
    return moveToRelative(arm, relPoint, new Quat4d(0, 0, 0, 1));
  }

  @Override
  public Justification look(String direction) {
    Point3d point;
    switch (direction) {
      case "up":
        point = new Point3d(0.35, 0, 1.5);
        break;
      case "down":
        point = new Point3d(0.5, 0, 0.55);
        break;
      case "left":
        point = new Point3d(1.0, 0.7, 1.2);
        break;
      case "right":
        point = new Point3d(1.0, -0.7, 1.2);
        break;
      case "forward":
        point = new Point3d(1.0, 0, 1.2);
        break;
      default:
        point = new Point3d(0.5, 0, 0.5);
    }
    return new ConditionJustification(pointHeadTo(point));
  }

  public float getChargeLevel() {
    BatteryState currBatteryState = robotDriver.getBatteryState();
    if (currBatteryState == null) {
      Util.Sleep(1100); // Battery state might not have been published yet (1 Hz)
      currBatteryState = robotDriver.getBatteryState();
      if (currBatteryState == null) { // Give up on trying to get battery state
        log.warn("[getChargeLevel] Battery state is null. Unable to get battery status");
        return -1;
      }
    }
    return currBatteryState.getChargeLevel();
  }

  public FetchBreakerStates getBreakerState(String breakerName) {
    // Get the robot state
    RobotState currRobotState = robotDriver.getRobotState();
    if (currRobotState == null) {
      Util.Sleep(50); // Robot state might not have been published yet (100 Hz)
      currRobotState = robotDriver.getRobotState();
      if (currRobotState == null) { // Give up on trying to get robot state
        log.warn("[getBreakerState] Robot state is null. Unable to get breaker status");
        return null;
      }
    }

    // Find the state of the given breaker
    for (BreakerState breaker : currRobotState.getBreakers()) {
      if (breaker.getName().equals(breakerName)) {
        return FetchBreakerStates.valToFetchBreakerStates(breaker.getState());
      }
    }

    log.warn("[getBreakerState] Breaker with name '" + breakerName + "' not found");
    return null;
  }

  public FetchBreakerStates setBreakerState(String breakerName, boolean state) {
    BreakerCommandRequest req = new BreakerCommandRequest();
    BreakerCommandResponse res = new BreakerCommandResponse();
    req.setEnable(state);

    switch (breakerName) {
      case "arm_breaker":
        robotDriver.callArmBreaker(req, res);
        break;
      case "base_breaker":
        robotDriver.callBaseBreaker(req, res);
        break;
      case "gripper_breaker":
        robotDriver.callGripperBreaker(req, res);
        break;
      default:
        log.warn("[setBreakerState] Breaker with name '" + breakerName + "' not found");
        return null;
    }

    return FetchBreakerStates.valToFetchBreakerStates(res.getStatus().getState());
  }
}
