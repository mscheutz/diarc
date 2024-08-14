/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.tower;

import ai.thinkingrobots.trade.*;

import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Pose;
import edu.tufts.hrilab.diarcros.util.Convert;
import edu.tufts.hrilab.fol.*;

import java.util.*;

import edu.tufts.hrilab.movebase.MoveBase;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import javax.vecmath.Matrix4d;

import edu.tufts.hrilab.diarcros.common.RosConfiguration;

import edu.tufts.hrilab.diarcros.tower.TowerWaypointFollower;

public class ElevatorBoardingComponent extends DiarcComponent {

  protected RosConfiguration rc = new RosConfiguration();
  protected MoveBase base;
  protected String mapFrame = "map";
  protected String baseFrame = "base_link";
  protected String ns = "";
  protected TowerWaypointFollower waypointFollower;

  /**
   * How far away to stand from panel, in meters.
   */
  private double padding = 0.5;
  /**
   * If goal pose should be converted to relative pose before being sent to ROS movebase.
   */
  private boolean useRL = false;

  public ElevatorBoardingComponent() {
    super();
  }

  @Override
  public void init() {
    if (useRL) {
      base = new MoveBase(rc, mapFrame, baseFrame, this.getMyGroups());
    } else {
      waypointFollower = new TowerWaypointFollower();
      // tempTest = false;
      // if (tempTest) {
      //   // TEMP
      //   List<Matrix4d> bogusMatrices = new ArrayList<Matrix4d>();
      //   Matrix4d m1 = new Matrix4d(1.0, 0.0, 0.0, 0.0,
      //                             0.0, 1.0, 0.0, 0.0,
      //                             0.0, 0.0, 1.0, 0.0,
      //                             102.0, 0.0, 0.0, 1.0);
      //   Matrix4d m2 = new Matrix4d(1.0, 0.0, 0.0, 0.0,
      //                             0.0, 1.0, 0.0, 0.0,
      //                             0.0, 0.0, 1.0, 0.0,
      //                             44.0, 0.0, 0.0, 1.0);
      //   bogusMatrices.add(m1);
      //   bogusMatrices.add(m2);
      //   // bogusMatrices.add(Matrix4d(1.0, 0.0, 0.0, 0.0,
      //   //                            0.0, 1.0, 0.0, 0.0,
      //   //                            0.0, 0.0, 1.0, 0.0,
      //   //                            102.0, 0.0, 0.0, 1.0));
      //   // bogusMatrices.add(Matrix4d(1.0, 0.0, 0.0, 0.0,
      //   //                            0.0, 1.0, 0.0, 0.0,
      //   //                            0.0, 0.0, 1.0, 0.0,
      //   //                            44.0, 0.0, 0.0, 1.0));
      //   waypointFollower.waitForNode();
      //   try {
      //     this.followPath(bogusMatrices);
      //   } catch (TRADEException e) {
      //   }
      // }
    }
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("rl").longOpt("use_rl").hasArg().argName("true/false").desc("If true, use RL model to enter/exit elevator, otherwise use waypoints. (default: false").build());
    options.add(Option.builder("map").longOpt("map_frame").hasArg().argName("map_frame_id").desc("ROS tf robot base frame (default: base_link").build());
    options.add(Option.builder("base").longOpt("base_frame").hasArg().argName("base_frame_id").desc("ROS tf robot map frame (default: map)").build());
    options.add(Option.builder("ns").longOpt("namespace").hasArg().argName("namespace").desc("Set a ROS namespace pointing to this ROS component. Default: empty.").build());
    options.add(Option.builder("tf_prefix").hasArg().argName("namespace").desc("Set a tf prefix. Note that this appears similar to a namespace, but ROS namespaces do not impact tf tree links (making this parameter necessary). Read the TF docs for more detail. Default: empty.").build());
    options.add(Option.builder("rosmasteruri").hasArg().argName("rosmasteruri").desc("Override ROS_MASTER_URI environment variable").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("rl")) {
      useRL = Boolean.parseBoolean(cmdLine.getOptionValue("rl"));
    }
    if (cmdLine.hasOption("base")) {
      baseFrame = cmdLine.getOptionValue("base");
    }
    if (cmdLine.hasOption("map")) {
      mapFrame = cmdLine.getOptionValue("map");
    }
    if (cmdLine.hasOption("namespace")) {
      rc.namespace = cmdLine.getOptionValue("namespace");
    } else {
      rc.setNamespace(ns);
    }
    if (cmdLine.hasOption("tf_prefix")) {
      rc.tfPrefix = cmdLine.getOptionValue("tf_prefix");
      baseFrame = rc.getPrefixedFrame(baseFrame);
    }
    if (cmdLine.hasOption("rosmasteruri")) {
      String uri = cmdLine.getOptionValue("rosmasteruri");
      rc.setUriFromString(uri);
    }
  }

  @Action
  @TRADEService
  public Justification boardElevator(Symbol elevatorDoor) {
    if (useRL) {
      return boardElevatorRL(elevatorDoor);
    } else {
      return boardElevatorWaypoints(elevatorDoor);
    }
  }

  @Action
  @TRADEService
  public Justification exitElevator(Symbol elevatorDoor) {
    if (useRL) {
      return exitElevatorRL(elevatorDoor);
    } else {
      return exitElevatorWaypoints(elevatorDoor);
    }
  }

  /**
   * Board the elevator using waypoints, by giving a ref id to an elevator door
   *
   * @param elevatorDoor ref id of the elevator door
   * @return Justification as to why the action failed or succeeded
   */
  private Justification boardElevatorWaypoints(Symbol elevatorDoor) {
    List<Matrix4d> path = null;
    try {
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getElevatorEnterPosePath"));
      path = tsi.call(List.class, elevatorDoor, padding);
    } catch (TRADEException e) {
      log.error("Error calling getElevatorEnterPosePath for door: " + elevatorDoor.getName(), e);
    }

    // execute path
    try {
      return followPath(path);
    } catch (TRADEException e) {
      log.error("Error calling goToLocation during elevator boarding.", e);
    }
    return new ConditionJustification(true);
  }

  /**
   * Exit the elevator using waypoints, by giving a ref id to an elevator door
   *
   * @param elevatorDoor ref id of the elevator door
   * @return Justification as to why the action failed or succeeded
   */
  private Justification exitElevatorWaypoints(Symbol elevatorDoor) {
    List<Matrix4d> path = null;
    try {
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getElevatorExitPosePath"));
      path = tsi.call(List.class, elevatorDoor, padding);
    } catch (TRADEException e) {
      log.error("Error calling getElevatorExitPosePath for door: " + elevatorDoor.getName(), e);
    }

    // execute path
    try {
      return followPath(path);
    } catch (TRADEException e) {
      log.error("Error calling goToLocation during elevator exit.", e);
      return new ConditionJustification(false);
    }
  }

  /**
   * Helper method to execute path.
   *
   * @param path
   * @return
   * @throws TRADEException
   */
  private Justification followPath(List<Matrix4d> path) throws TRADEException {
    List<Pose> waypoints = new ArrayList<Pose>();

    for (Matrix4d transform : path) {
      Pose pose = Convert.convertToPose(transform);
      waypoints.add(pose);
    }

    edu.tufts.hrilab.diarcros.msg.tower_waypoint_follower.getPoseArrayRequest request = new edu.tufts.hrilab.diarcros.msg.tower_waypoint_follower.getPoseArrayRequest();
    request.setPoses(waypoints);

    edu.tufts.hrilab.diarcros.msg.tower_waypoint_follower.getPoseArrayResponse response = new edu.tufts.hrilab.diarcros.msg.tower_waypoint_follower.getPoseArrayResponse();
    waypointFollower.callTowerWaypointFollower(request, response);

    return new ConditionJustification(true);
  }

  /**
   * Board the elevator using RL, by giving a ref id to an elevator door
   *
   * @param elevatorDoor ref id of the elevator door
   * @return Justification as to why the action failed or succeeded
   */
  private Justification boardElevatorRL(Symbol elevatorDoor) {
    Matrix4d transform = null;
    try {
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getElevatorPanelPose"));
      transform = tsi.call(Matrix4d.class, elevatorDoor, padding);
    } catch (TRADEException e) {
      log.error("unable to get pose from door " + elevatorDoor.getName(), e);
    }
    Pose goal = getRelativePoseFromGlobal(mapFrame, baseFrame, transform);
    return base.goToLocation(goal, true);
  }

  /**
   * Exit the elevator by giving a ref id to an elevator door
   *
   * @param elevatorDoor ref id of the elevator door
   * @return Justification as to why the action failed or succeeded
   */
  private Justification exitElevatorRL(Symbol elevatorDoor) {
    Matrix4d transform = null;
    try {
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getElevatorExitPose"));
      transform = tsi.call(Matrix4d.class, elevatorDoor);
    } catch (TRADEException e) {
      log.error("unable to get pose from door " + elevatorDoor.getName(), e);
    }
    Pose goal = getRelativePoseFromGlobal(mapFrame, baseFrame, transform);
    return base.goToLocation(goal, true);
  }

  private Pose getRelativePoseFromGlobal(String globalFrame, String relativeFrame, Matrix4d relativePose) {
    Matrix4d matTf = null;
    try {
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getTransform"));
      matTf = tsi.call(Matrix4d.class, globalFrame, relativeFrame);
    } catch (TRADEException e) {
      log.error("unable to get transform for frame " + globalFrame + " to " + relativeFrame, e);
    }
    if (matTf == null) {
      return null;
    }
    relativePose.mul(matTf);
    return Convert.convertToPose(matTf);
  }
}
