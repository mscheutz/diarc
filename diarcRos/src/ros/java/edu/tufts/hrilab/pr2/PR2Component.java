/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.pr2;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import edu.tufts.hrilab.moveit.MoveItComponent;
import edu.tufts.hrilab.moveit.util.MoveItHelper;
import edu.tufts.hrilab.diarcros.msg.Duration;
import edu.tufts.hrilab.diarcros.msg.Time;
import edu.tufts.hrilab.diarcros.msg.control_msgs.FollowJointTrajectoryGoal;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Point;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.PointStamped;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Vector3;
import edu.tufts.hrilab.diarcros.msg.pr2_controllers_msgs.JointTrajectoryControllerState;
import edu.tufts.hrilab.diarcros.msg.pr2_controllers_msgs.PointHeadGoal;
import edu.tufts.hrilab.diarcros.msg.pr2_controllers_msgs.SingleJointPositionGoal;
import edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState;
import edu.tufts.hrilab.diarcros.msg.std_msgs.Header;
import edu.tufts.hrilab.diarcros.msg.trajectory_msgs.JointTrajectory;
import edu.tufts.hrilab.diarcros.msg.trajectory_msgs.JointTrajectoryPoint;
import edu.tufts.hrilab.diarcros.pr2.ArmTrajController;
import edu.tufts.hrilab.diarcros.pr2.HeadTrajControllerPointHeadAction;
import edu.tufts.hrilab.diarcros.pr2.TorsoControllerPositionJointActionNode;
import edu.tufts.hrilab.util.Util;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.apache.commons.lang3.ArrayUtils;
import org.ros.actionlib.state.SimpleClientGoalState;
import org.ros.exception.RosException;

import javax.vecmath.Point3d;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public final class PR2Component extends MoveItComponent implements PR2Interface {

  private final Gson gson = new GsonBuilder().setPrettyPrinting().serializeNulls().serializeSpecialFloatingPointValues().create();

  // DIARCROS nodes
  private final HeadTrajControllerPointHeadAction head;
  private final TorsoControllerPositionJointActionNode torso;

  // for recording and playing back arm trajectories
  private ArmTrajController arm_traj_controller;

  private boolean initStartPose = false;

  public PR2Component() {
    super();
    configName = "PR2.json";

    torso = new TorsoControllerPositionJointActionNode();
    torso.waitForNode(10);
    if (!torso.isNodeReady()) {
      log.error("Head failed to initialize.");
    }

    head = new HeadTrajControllerPointHeadAction();
    head.waitForNode(10);
    if (!head.isNodeReady()) {
      log.error("Head failed to initialize.");
    }

    arm_traj_controller = new ArmTrajController();
    arm_traj_controller.waitForNode(10);
    if (!arm_traj_controller.isNodeReady()) {
      log.error("Head failed to initialize.");
    }
  }

  @Override
  protected void init() {
    super.init();

    if (initStartPose) {
      setTorsoPosition(0.3);
      pointHeadTo(new Point3d(0.35, -0.1, 1.0));
      goToPose(Factory.createSymbol("start"));
    }
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("startPose").desc("call goToPose(start) on startup").build());
    options.addAll(super.additionalUsageInfo());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    super.parseArgs(cmdLine);

    if (cmdLine.hasOption("startPose")) {
      initStartPose = true;
    }
  }

  @Override
  public boolean setTorsoPosition(double position) {
    SingleJointPositionGoal positionGoal = new SingleJointPositionGoal();
    positionGoal.setPosition(position);
    try {
      torso.sendPr2ControllersMsgsSingleJointPositionGoal(positionGoal);
      torso.waitForPr2ControllersMsgsSingleJointPositionResult();
    } catch (RosException | InterruptedException e) {
      log.error("Error setting torso position.", e);
    }
    SimpleClientGoalState state = torso.getPr2ControllersMsgsSingleJointPositionState();
    return (state.getState() == SimpleClientGoalState.StateEnum.SUCCEEDED);
  }

  @Override
  public boolean pointHeadTo(Symbol objectRef) {
    log.debug("[pointHeadTo] ref method entered.");

    // get Point3d from object ref
    Point3d location = null;
    try {
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getEntityForReference").argTypes(Symbol.class, Class.class));
      location = tsi.call(Point3d.class, objectRef, Point3d.class);

    } catch (TRADEException e) {
      log.error("[pointTo] exception getting Point3d from reference, returning null", e);
    }

    if (location == null) {
      return false;
    }

    // point head to location
    return pointHeadTo(location);
  }

  @Override
  public boolean pointHeadTo(Point3d targetPosition) {
    log.debug("[pointHeadTo] method entered.");

    PointStamped pos = new PointStamped();
    pos.getHeader().setFrameId(moveItConfig.baseLinkString);
    pos.setPoint(new Point(targetPosition.x, targetPosition.y, targetPosition.z));

    Vector3 pointingAxis = new Vector3(1.0, 0.0, 0.0);  //(1,0,0) = x-axis

    PointHeadGoal headGoal = new PointHeadGoal();
    headGoal.setTarget(pos);
    headGoal.setMaxVelocity(0.35); // rad/s
    headGoal.setPointingFrame("high_def_frame");    //TODO: probably change this to kinect frame
    headGoal.setPointingAxis(pointingAxis);

    SimpleClientGoalState goalState;
    try {
      head.sendGoal(headGoal);
      head.waitForResult(10, TimeUnit.SECONDS);
      goalState = head.getState();
    } catch (InterruptedException | RosException e) {
      log.error("pointHeadTo", e);
      return false;
    }

    boolean result = (goalState.getState() == SimpleClientGoalState.StateEnum.SUCCEEDED);
    log.debug(String.format("[pointHeadTo] finished with goal state: %s.", goalState.getState().toString()));
    return result;
  }

  @Override
  public boolean goToPoseNoPlanning(String pose_name) {
    boolean result = false;

    if (poses.containsKey(pose_name)) {
      JointState joint_state = poses.get(pose_name).getJointState();
      List<String> lArmJointNames = new ArrayList<>();
      List<String> rArmJointNames = new ArrayList<>();
      List<Double> lPositions = new ArrayList<>();
      List<Double> rPositions = new ArrayList<>();
      List<Double> lVelocities = new ArrayList<>();
      List<Double> rVelocities = new ArrayList<>();
      List<Double> lAccels = new ArrayList<>();
      List<Double> rAccels = new ArrayList<>();
      double vel = 0.01;
      for (int i = 0; i < joint_state.getName().size(); ++i) {
        String jointName = joint_state.getName().get(i);
        if (jointName.startsWith("l_") && !jointName.contains("gripper")) {
          lArmJointNames.add(jointName);
          lPositions.add(joint_state.getPosition()[i]);
          lVelocities.add(vel);
          lAccels.add(0.0);
        } else if (jointName.startsWith("r_") && !jointName.contains("gripper")) {
          rArmJointNames.add(jointName);
          rPositions.add(joint_state.getPosition()[i]);
          rVelocities.add(vel);
          rAccels.add(0.0);
        }
      }

      JointTrajectoryPoint lJTPoint = new JointTrajectoryPoint();
      lJTPoint.setTimeFromStart(new Duration(0, 0));
      lJTPoint.setPositions(MoveItHelper.convertToDoubleArray(lPositions));
      lJTPoint.setVelocities(MoveItHelper.convertToDoubleArray(lVelocities));
      lJTPoint.setAccelerations(MoveItHelper.convertToDoubleArray(lAccels));
      lJTPoint.setEffort(new double[]{});
      JointTrajectoryPoint rJTPoint = new JointTrajectoryPoint();
      rJTPoint.setTimeFromStart(new Duration(0, 0));
      rJTPoint.setPositions(MoveItHelper.convertToDoubleArray(rPositions));
      rJTPoint.setVelocities(MoveItHelper.convertToDoubleArray(rVelocities));
      rJTPoint.setAccelerations(MoveItHelper.convertToDoubleArray(rAccels));
      rJTPoint.setEffort(new double[]{});

      List<JointTrajectoryPoint> lArmJTPoints = new ArrayList<>();
      lArmJTPoints.add(lJTPoint);
      List<JointTrajectoryPoint> rArmJTPoints = new ArrayList<>();
      rArmJTPoints.add(rJTPoint);

      JointTrajectory lArmJointTrajectory = new JointTrajectory();
      lArmJointTrajectory.setHeader(new Header(0, new Time(), "/base_link")); //is this right?
      lArmJointTrajectory.setJointNames(lArmJointNames);
      lArmJointTrajectory.setPoints(lArmJTPoints);
      JointTrajectory rArmJointTrajectory = new JointTrajectory();
      rArmJointTrajectory.setHeader(new Header(0, new Time(), "/base_link")); //is this right?
      rArmJointTrajectory.setJointNames(rArmJointNames);
      rArmJointTrajectory.setPoints(rArmJTPoints);

      FollowJointTrajectoryGoal lArmFollowJointTrajectoryGoal = new FollowJointTrajectoryGoal();
      lArmFollowJointTrajectoryGoal.setTrajectory(lArmJointTrajectory);

      FollowJointTrajectoryGoal rArmFollowJointTrajectoryGoal = new FollowJointTrajectoryGoal();
      rArmFollowJointTrajectoryGoal.setTrajectory(rArmJointTrajectory);

      log.debug("[goToPoseNoPlanning] left trajectory goal: " + gson.toJson(lArmFollowJointTrajectoryGoal));
      log.debug("[goToPoseNoPlanning] right trajectory goal: " + gson.toJson(rArmFollowJointTrajectoryGoal));

      try {
        arm_traj_controller.sendLArmControllerFollowJointTrajectoryClientGoal(lArmFollowJointTrajectoryGoal);
        arm_traj_controller.sendRArmControllerFollowJointTrajectoryClientGoal(rArmFollowJointTrajectoryGoal);
//        arm_traj_controller.sendLArmControllerJointTrajectoryActionClientGoal(null);
//        arm_traj_controller.sendRArmControllerJointTrajectoryActionClientGoal(null);

        // NOTE: this is an attempt to wait for the goals to complete but
        // none of the waitFor___ methods seem to work and isDone doesn't seem to
        // be reliable either. As a result, this method usually returns right before
        // the goals actually terminate.
        while (!arm_traj_controller.getLArmControllerFollowJointTrajectoryClientState().isDone()) {
          Util.Sleep(10);
          log.info("[goToPoseNoPlanning] LArmControllerFollowJointTrajectoryClient status: "
                  + arm_traj_controller.getLArmControllerFollowJointTrajectoryClientState().getState());
        }
        log.info("[goToPoseNoPlanning] LArmControllerFollowJointTrajectoryClient status: "
                + arm_traj_controller.getLArmControllerFollowJointTrajectoryClientState().getState());

        while (!arm_traj_controller.getRArmControllerFollowJointTrajectoryClientState().isDone()) {
          Util.Sleep(10);
          log.info("[goToPoseNoPlanning] RArmControllerFollowJointTrajectoryClient status: "
                  + arm_traj_controller.getRArmControllerFollowJointTrajectoryClientState().getState());
        }
        log.info("[goToPoseNoPlanning] RArmControllerFollowJointTrajectoryClient status: "
                + arm_traj_controller.getRArmControllerFollowJointTrajectoryClientState().getState());

        if (arm_traj_controller.getLArmControllerFollowJointTrajectoryClientState().getState() == SimpleClientGoalState.StateEnum.SUCCEEDED
                && arm_traj_controller.getRArmControllerFollowJointTrajectoryClientState().getState() == SimpleClientGoalState.StateEnum.SUCCEEDED) {
          result = true;
        }
      } catch (RosException e) {
        log.error("Error trying to send follow joint trajectory goal.", e);
      }

    } else {
      log.error("[goToPose] planning scene null.");
    }

    if (!result) {
      log.error("[goToPose] Have you modified your MoveIt! groups to include a whole_body_no_base group containing (arms,torso,head) sub-groups?");
    }
    return result;
  }

  /**
   * There are two arms here, so startRecordingTrajectory intentionally overrides MoveItComponent method.
   *
   * @param trajectoryName
   */
  @Override
  public void startRecordingTrajectory(final String trajectoryName) {
    Thread recording_thread = new Thread(() -> {
      Time start_time = arm_traj_controller.getCurrentTime();
      List<JointTrajectoryPoint> trajectory_points = new ArrayList<>();
      recordingTrajectory = true;
      JointTrajectoryControllerState r_arm_state = null;
      JointTrajectoryControllerState l_arm_state = null;
      while (recordingTrajectory) {
        Duration time_from_start = arm_traj_controller.getCurrentTime().subtract(start_time);
        r_arm_state = arm_traj_controller.getRArmControllerState();
        l_arm_state = arm_traj_controller.getLArmControllerState();
        if (r_arm_state != null && l_arm_state != null) {
          // get right and left arms trajectory point
          JointTrajectoryPoint r_traj_point = r_arm_state.getActual();
          JointTrajectoryPoint l_traj_point = l_arm_state.getActual();

          // combine right and left arms into single JointTrajectoryPoint
          JointTrajectoryPoint combined_traj_point = new JointTrajectoryPoint();
          combined_traj_point.setTimeFromStart(time_from_start);
          combined_traj_point.setPositions(ArrayUtils.addAll(r_traj_point.getPositions(), l_traj_point.getPositions()));
          combined_traj_point.setVelocities(ArrayUtils.addAll(r_traj_point.getVelocities(), l_traj_point.getVelocities()));
          combined_traj_point.setAccelerations(ArrayUtils.addAll(r_traj_point.getAccelerations(), l_traj_point.getAccelerations()));
          combined_traj_point.setEffort(ArrayUtils.addAll(r_traj_point.getEffort(), l_traj_point.getEffort()));
          trajectory_points.add(combined_traj_point);

          log.debug("[startRecordingTrajectory] trajectory goal: " + gson.toJson(combined_traj_point));
        } else {
          log.warn("[startRecordingTrajectory] trajectory null.");
        }
        Util.Sleep(400);
      }

      log.debug("[startRecordingTrajectory] done recording.");

      if (r_arm_state != null && l_arm_state != null && !trajectory_points.isEmpty()) {
        JointTrajectory new_traj = new JointTrajectory();
        List<String> combined_joint_names = new ArrayList<>(r_arm_state.getJointNames());
        combined_joint_names.addAll(l_arm_state.getJointNames());
        new_traj.setJointNames(combined_joint_names);
        new_traj.setPoints(trajectory_points);
        recordedTrajectories.put(trajectoryName, new_traj);

        log.debug("[startRecordingTrajectory] added new trajectory: " + trajectoryName);
      } else {
        log.warn("[startRecordingTrajectory] not adding new trajectory: " + trajectoryName);
      }
    });
    recording_thread.start();
  }

  //EAK: this method was used for two hand pickup/release
//  @Override
//  public boolean moveToRelative(String group_name, Point3d point, Quat4d orientation, double squeeze_amount) {
//    log.info("Move to relative entered");
//
//    // Translation/rotation offset by 0.
//    Pose rel_pose_translation = new Pose(new Point(point.x, point.y, point.z),
//            new Quaternion(0, 0, 0, 1));
//    Pose rel_pose_rotation = new Pose(new Point(0, 0, 0),
//            new Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
//
//    // get current pose(s)
//    ArrayList<String> link_names = new ArrayList<>();
//    List<Pose> link_poses = new ArrayList<>();
//
//    String effectorName = endEffectorNames.get(group_name);
//    if(effectorName == null) {
//      log.error("That group name wasn't valid (doesn't appear to have any sort of end effector mentioned in the json config): "+group_name);
//      return false;
//    } else if (notOneArm(group_name)) { // This is a special case where we need to deal with both arms
//      link_names.add(endEffectorNames.get("left_arm"));
//      link_names.add(endEffectorNames.get("right_arm"));
//    } else { // This is the standard case
//      link_names.add(effectorName);
//    }
//
//    link_poses = getPoses(link_names, baseLink);
//    // add new relative pose info to current pose(s) to calculate final pose in base frame
//    JointTrajectory final_trajectory = null;
//    Pose new_pose_rotation;
//    Pose new_pose_translation;
//    Pose curr_link_pose;
//    if (link_poses.size() == 2) {
//      MoveItHelper.squeezePoses(link_poses.get(0), link_poses.get(1), squeeze_amount);
//    }
//
//    // CR: Maybe start breaking this down into smaller functions, its getting long
//    double fraction_of_first = 1.0;
//    for (int i = 0; i < link_poses.size(); ++i) {
//      // CR: given that this method is already very long, I'd consider making the contents of this loop its own method
//      curr_link_pose = link_poses.get(i);
//
//      //rotate the final pose of each gripper by the same rotation
//      new_pose_rotation = Transformations.appendPoses(rel_pose_rotation, curr_link_pose);
//
//      //then translate the position by the point value
//      new_pose_translation = new Pose(new Point(rel_pose_translation.getPosition().getX() + curr_link_pose.getPosition().getX(),
//              rel_pose_translation.getPosition().getY() + curr_link_pose.getPosition().getY(),
//              rel_pose_translation.getPosition().getZ() + curr_link_pose.getPosition().getZ()),
//              new Quaternion(0, 0, 0, 1));
//
//      //the link pose is the translated and rotation link position in base link frame
//      Pose goal_link_pose = new Pose(new_pose_translation.getPosition(), new_pose_rotation.getOrientation());
//
//      // TODO: introduce new classes to build and manage path constraints
//      // 2 classes: Cartesian and ...   the other one - whatever ROS calls it
//      //compute a straight line path for each link
//      GetCartesianPathResponse res = new GetCartesianPathResponse();
//      GetCartesianPathRequest req = MoveItHelper.makeCartesianPath(curr_link_pose, goal_link_pose, link_names.get(i), baseLink, group_name);
//
//      log.debug("making cartpaths");
//      move_group.callComputeCartesianPath(req, res);
//      log.debug("The planning error code: " + res.getErrorCode().getVal());
//
//      if (res.getFraction() < fraction_of_first) //in the case that only a fraction of a trajectory is able to be completed by one of the arms
//      {
//        fraction_of_first = res.getFraction();
//      }
//
//      // CR: Maybe check against i instead of checking final_trajectory
//      if (final_trajectory == null) {
//        final_trajectory = res.getSolution().getJointTrajectory();
//        MoveItHelper.smoothCartesianRotation(final_trajectory);
//      } else {
//        JointTrajectory new_trajectory = res.getSolution().getJointTrajectory();
//        MoveItHelper.smoothCartesianRotation(new_trajectory);
//        final_trajectory = MoveItHelper.combineTrajectories(final_trajectory, new_trajectory, fraction_of_first);
//      }
//    }
//
//    RobotTrajectory rt = new RobotTrajectory(final_trajectory, new MultiDOFJointTrajectory());
//    try {
//      return move_group.executeKinematicPath(rt);
//    } catch (InterruptedException | RosException e) {
//      log.error("Exception while trying to execute trajectory.", e);
//      return false;
//    }
//  }
}
