/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.moveit.util;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import edu.tufts.hrilab.diarcros.msg.Time;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Point;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Pose;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Quaternion;
import edu.tufts.hrilab.diarcros.msg.geometry_msgs.Vector3;
import edu.tufts.hrilab.diarcros.msg.moveit_msgs.BoundingVolume;
import edu.tufts.hrilab.diarcros.msg.moveit_msgs.Constraints;
import edu.tufts.hrilab.diarcros.msg.moveit_msgs.GetCartesianPathRequest;
import edu.tufts.hrilab.diarcros.msg.moveit_msgs.JointConstraint;
import edu.tufts.hrilab.diarcros.msg.moveit_msgs.TrajectoryConstraints;
import edu.tufts.hrilab.diarcros.msg.moveit_msgs.MotionPlanRequest;
import edu.tufts.hrilab.diarcros.msg.moveit_msgs.OrientationConstraint;
import edu.tufts.hrilab.diarcros.msg.moveit_msgs.PositionConstraint;
import edu.tufts.hrilab.diarcros.msg.moveit_msgs.RobotState;
import edu.tufts.hrilab.diarcros.msg.moveit_msgs.WorkspaceParameters;
import edu.tufts.hrilab.diarcros.msg.sensor_msgs.JointState;
import edu.tufts.hrilab.diarcros.msg.shape_msgs.SolidPrimitive;
import edu.tufts.hrilab.diarcros.msg.std_msgs.Header;
import edu.tufts.hrilab.diarcros.msg.trajectory_msgs.JointTrajectory;
import edu.tufts.hrilab.diarcros.msg.trajectory_msgs.JointTrajectoryPoint;

import java.util.Arrays;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import javax.vecmath.Matrix4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.apache.commons.lang3.ArrayUtils;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * @author Evan Krause <evan.krause@tufts.edu>
 */
public class MoveItHelper {

  private static final Gson gson = new GsonBuilder().setPrettyPrinting().serializeNulls().serializeSpecialFloatingPointValues().create();
  private static Logger log = LoggerFactory.getLogger(MoveItHelper.class);

  public static double[] convertToDoubleArray(List<Double> list) {
    double[] returnArr = new double[list.size()];
    for (int i = 0; i < list.size(); ++i) {
      returnArr[i] = list.get(i);
    }
    return returnArr;
  }

  /**
   * Move point by gripper_offset amount in direction of orientation's negative
   * x-axis (based on gripper's coordinate frame).
   *
   * @param grasp_point       grasp point on object
   * @return
   */
  public static Point3d calcTargetGraspPoint(final Point3d grasp_point, final Quat4d grasp_orientientation, float grasp_offset) {
    Matrix4d pose = new Matrix4d(grasp_orientientation, new Vector3d(grasp_point), 1.0);
    Vector3d neg_x_axis = new Vector3d(-1.0, 0.0, 0.0);
    neg_x_axis.scale(grasp_offset);
    Matrix4d neg_mat = new Matrix4d(new Quat4d(0.0, 0.0, 0.0, 1.0), neg_x_axis, 1.0);
    Vector3d target_point = new Vector3d();
    pose.mul(neg_mat);
    pose.get(target_point);
    //log.debug("[calcTargetGraspPoint] linc's ttarget_point: " + target_point);

    //
    //    log.debug("[calcTargetGraspPoint] grasp_point: " + grasp_point);
    //    log.debug("[calcTargetGraspPoint] neg_x_axis: " + neg_x_axis);
    //    //pose.invert();
    //    pose.transform(neg_x_axis);
    //    log.debug("[calcTargetGraspPoint] transformed neg_x_axis: " + neg_x_axis);
    //    target_point = new Vector3d(grasp_point);
    //    target_point.add(neg_x_axis);
    //    log.debug("[calcTargetGraspPoint] transformed grasp_point: " + target_point);
    //
    return new Point3d(target_point);
  }

  /**
   * Move point by offset amount in direction of specified offset_axis.
   *
   * @param point       3D point
   * @param orientation point's orientation
   * @param offset_axis direction to apply offset
   * @param offset      (amount to translate along offset_axis)
   * @return
   */
  public static Point3d calcTargetOffset(final Point3d point, final Quat4d orientation, Vector3d offset_axis, float offset) {
    Matrix4d pose = new Matrix4d(orientation, new Vector3d(point), 1.0);
    Vector3d axis = new Vector3d(offset_axis);
    axis.normalize();
    axis.scale(offset);
    Matrix4d mat = new Matrix4d(new Quat4d(0.0, 0.0, 0.0, 1.0), axis, 1.0);
    Vector3d target_point = new Vector3d();
    pose.mul(mat);
    pose.get(target_point);
    return new Point3d(target_point);
  }

//  public static void squeezePoses(Pose init_pose1, Pose init_pose2, double squeeze_amount) {
//    log.debug("Squeeze amount: " + squeeze_amount);
//
//    //these are the vectors describing the difference to move the initial pose to a squeeze position
//    //2-1
//    Point pose1topose2 = Transformations.difference(init_pose1.getPosition(), init_pose2.getPosition());
//    Point pose2topose1 = Transformations.difference(init_pose2.getPosition(), init_pose1.getPosition());
//
//    double distance = Transformations.distance(init_pose1.getPosition(), init_pose2.getPosition());
//
//    Point scaledpose2topose1 = Transformations.scale(pose2topose1, distance * squeeze_amount);
//    Point scaledpose1topose2 = Transformations.scale(pose1topose2, distance * squeeze_amount);
//
//    Point scaled_vector1 = Transformations.add(scaledpose1topose2, init_pose1.getPosition());
//    Point scaled_vector2 = Transformations.add(scaledpose2topose1, init_pose2.getPosition());
//
//    init_pose1.setPosition(scaled_vector1);
//    init_pose2.setPosition(scaled_vector2);
//  }

  public static void smoothCartesianRotation(JointTrajectory jt) {
    List<String> names = jt.getJointNames();
    List<JointTrajectoryPoint> points = jt.getPoints();
    int[] jointsToChange = new int[]{names.indexOf("r_wrist_roll_joint"),
                                     names.indexOf("l_wrist_roll_joint"),
                                     names.indexOf("r_forearm_roll_joint"),
                                     names.indexOf("l_forearm_roll_joint")};

    // CR: We should describe what we did here and/or make it less gross
    for (int i = 0; i < jointsToChange.length; i++) {
      int jointChange = jointsToChange[i];
      if (jointChange == -1) {
        continue;
      }
      double amount_to_add = 0;
      for (int j = 1; j < points.size(); j++) {
        double now_position = points.get(j).getPositions()[jointChange];
        double prev_position = points.get(j - 1).getPositions()[jointChange];

        double diff = now_position - prev_position;
        if (diff < -1 * Math.PI) {
          amount_to_add = 2 * Math.PI;
        } else if (diff > Math.PI) {
          amount_to_add = -2 * Math.PI;
        }

        points.get(j).getPositions()[jointChange] += amount_to_add;
      }
    }
    jt.setPoints(points);
  }

  public static JointTrajectory sortTrajectory(JointTrajectory jt) {
    Collections.sort(jt.getPoints(), new Comparator<JointTrajectoryPoint>() {
      @Override
      public int compare(JointTrajectoryPoint o1, JointTrajectoryPoint o2) {
        return o1.getTimeFromStart().compareTo(o2.getTimeFromStart());
      }
    });
    return jt;
  }

  // takes the first and second and combines them assuming the time increment is consistent and the trajectories are ordered by time
  // ignores velocities and accelerations for now
  public static JointTrajectory combineTrajectories(JointTrajectory jt1, JointTrajectory jt2, double fraction_of_first) {
    jt1 = sortTrajectory(jt1);
    jt2 = sortTrajectory(jt2);

    JointTrajectory jt = new JointTrajectory();

    //take the header info from the first joint trajectory only
    jt.setHeader(jt1.getHeader());

    //add all the joint names together and put it in the first joint trajectory, then give it to jt
    jt.getJointNames().addAll(jt1.getJointNames());
    jt.getJointNames().addAll(jt2.getJointNames());

    int npoints_1 = jt1.getPoints().size();
    int npoints_2 = jt2.getPoints().size();
    int npoints;

    if (npoints_1 < npoints_2) {
      npoints = npoints_2;
    } else {
      npoints = npoints_1;
    }
    ArrayList<JointTrajectoryPoint> jtps = new ArrayList<JointTrajectoryPoint>();

    int adjusted_n_waypoints = (int) (npoints * fraction_of_first);
    log.trace("adjusted_n_waypoints: " + adjusted_n_waypoints);
    for (int i = 0; i < adjusted_n_waypoints; i++) {
      double[] jt1_positions;
      double[] jt2_positions;

      JointTrajectoryPoint jtp = new JointTrajectoryPoint();
      jtp.setEffort(new double[0]);
      jtp.setAccelerations(new double[0]);
      jtp.setVelocities(new double[0]);

      if (i >= npoints_1) {
        jt1_positions = jt1.getPoints().get(npoints_1 - 1).getPositions();
      } else {
        jt1_positions = jt1.getPoints().get(i).getPositions();
      }

      if (i >= npoints_2) {
        jt2_positions = jt2.getPoints().get(npoints_2 - 1).getPositions();
      } else {
        jt2_positions = jt2.getPoints().get(i).getPositions();
      }
      // CR: Kinda gross to follow
      if (npoints_1 < npoints_2) {
        jtp.setTimeFromStart(jt2.getPoints().get(i).getTimeFromStart());
      } else {
        jtp.setTimeFromStart(jt1.getPoints().get(i).getTimeFromStart());
      }

      double[] jt_positions = ArrayUtils.addAll(jt1_positions, jt2_positions);
      jtp.setPositions(jt_positions);
      jtps.add(jtp);
    }

    jt.setPoints(jtps);
    return jt;
  }

  public static MotionPlanRequest makeRequest(String group_name,
                                              List<Constraints> goalConstraints,
                                              Constraints pathConstraints,
                                              TrajectoryConstraints trajectoryConstraints,
                                              List<String> jointNames,
                                              JointState jointState,
                                              RobotState robotState,
                                              double planningTime,
                                              int maxPlanningAttempts) {

    WorkspaceParameters ws_params = new WorkspaceParameters(new Vector3(-1.0, -1.0, 0), new Vector3(1.0, 1.0, 1.0));

    // prune jointState to only include info for joints included in jointNames
    List<String> prunedNames = new ArrayList<>(jointNames.size());
    double[] prunedPosition = new double[jointNames.size()];
    double[] prunedVelocity = new double[jointNames.size()];
    double[] prunedEffort = new double[jointNames.size()];
    int prunedIndex = 0;
    for (int i = 0; i < jointState.getName().size(); ++i) {
      String jointName = jointState.getName().get(i);
      if (jointNames.contains(jointName)) {
        prunedNames.add(jointName);
        prunedPosition[prunedIndex] = jointState.getPosition()[i];
        prunedVelocity[prunedIndex] = jointState.getVelocity()[i];
        prunedEffort[prunedIndex] = jointState.getEffort()[i];
        ++prunedIndex;
      }
    }
    JointState prunedJointState = new JointState(prunedNames, prunedPosition, prunedVelocity, prunedEffort);
    if (prunedNames.size() != jointNames.size()) {
      log.error("[makeRequest] all the requested Joint Names were not contained in the JointState." +
              "This will likely result in MoveIt! planning problems.");
      log.debug("Pruned names [" + prunedNames.size() + "]: " + Arrays.toString(prunedNames.toArray()));
      log.debug("Joint names [" + jointNames.size() + "]: " + Arrays.toString(jointNames.toArray()));
    }

    robotState.setJointState(prunedJointState);
    robotState.setIsDiff(true);
    goalConstraints.forEach(g -> g.setName(group_name));

    MotionPlanRequest req = new MotionPlanRequest();

    req.setStartState(robotState);
    req.setGoalConstraints(goalConstraints);
    req.setWorkspaceParameters(ws_params);
    req.setGroupName(group_name);
    // pathConstraints is either null or user-specified.
    if (pathConstraints != null) {
      req.setPathConstraints(pathConstraints);
    }
    // trajectoryConstraints is either null or user-specified.
    if (trajectoryConstraints != null) {
      req.setTrajectoryConstraints(trajectoryConstraints);
    }

    req.setNumPlanningAttempts(maxPlanningAttempts); // CR: We should figure out what exactly this does instead of guessing values
    req.setAllowedPlanningTime(planningTime);//5 secs

    return req;
  }

  // combine constraints into single Constraints object
  public static void appendGoalConstrains(Constraints constraints, Constraints newConstraints) {
    // NOTE: doesn't work to add each end effector constraint to goalConstraints as separate entries
    constraints.getJointConstraints().addAll(newConstraints.getJointConstraints());
    constraints.getOrientationConstraints().addAll(newConstraints.getOrientationConstraints());
    constraints.getPositionConstraints().addAll(newConstraints.getPositionConstraints());
    constraints.getVisibilityConstraints().addAll(newConstraints.getVisibilityConstraints());
  }

  // check out http://docs.ros.org/api/moveit_core/html/utils_8cpp_source.html#l00125. It's the function which I got this from
  public static Constraints makeGoalConstraints(Pose pose, double pos_tolerance, double angle_tolerance, String target_link, String base_link) {
    Header header = new Header(0, new Time(), base_link);

    edu.tufts.hrilab.diarcros.msg.moveit_msgs.Constraints goal_constraints = new edu.tufts.hrilab.diarcros.msg.moveit_msgs.Constraints();
    goal_constraints.setName(target_link);

    // Position constraint
    List<PositionConstraint> position_constraints = new ArrayList<>();

    PositionConstraint pcm = new edu.tufts.hrilab.diarcros.msg.moveit_msgs.PositionConstraint();
    pcm.setHeader(header);
    pcm.setLinkName(target_link);
    pcm.setTargetPointOffset(new Vector3(0.0, 0.0, 0.0));
    pcm.setWeight(1.0);

    List<Pose> sphere_poses = new ArrayList<>();
    Pose sphere_pose = new Pose();
    sphere_pose.setOrientation(new Quaternion(0.0, 0.0, 0.0, 1.0));
    sphere_pose.setPosition(pose.getPosition());
    sphere_poses.add(sphere_pose);

    BoundingVolume region = new BoundingVolume();
    SolidPrimitive sphere = new SolidPrimitive(SolidPrimitive.SPHERE, new double[]{pos_tolerance});
    List<SolidPrimitive> prims = new ArrayList<>();
    prims.add(sphere);
    region.setPrimitives(prims);
    region.setPrimitivePoses(sphere_poses);
    pcm.setConstraintRegion(region);

    position_constraints.add(pcm);
    goal_constraints.setPositionConstraints(position_constraints);

    // Orientation constraint
    List<OrientationConstraint> orientation_constraints = new ArrayList<>();
    OrientationConstraint ocm = new OrientationConstraint();
    ocm.setHeader(header);
    ocm.setAbsoluteXAxisTolerance(angle_tolerance);
    ocm.setAbsoluteYAxisTolerance(angle_tolerance);
    ocm.setAbsoluteZAxisTolerance(angle_tolerance);
    ocm.setLinkName(target_link);
    ocm.setOrientation(pose.getOrientation());
    ocm.setWeight(1);

    orientation_constraints.add(ocm);
    goal_constraints.setOrientationConstraints(orientation_constraints);

    return goal_constraints;
  }

  public static Constraints makeCarryingPathConstraints(Pose start_pose, double angle_tolerance, String target_link, String base_link) {
    Header header = new Header(0, new Time(), base_link);

    edu.tufts.hrilab.diarcros.msg.moveit_msgs.Constraints pathConstraints = new edu.tufts.hrilab.diarcros.msg.moveit_msgs.Constraints();
    pathConstraints.setName(target_link);

    // Orientation constraint
    List<OrientationConstraint> orientation_constraints = new ArrayList<>();
    OrientationConstraint ocm = new OrientationConstraint();
    ocm.setHeader(header);
    //TODO: Will: find a way to pass in this information (I.E. which axis to maintain) instead of assuming it to follow
    //the fetch default assumption
    ocm.setAbsoluteXAxisTolerance(2*Math.PI); // roll which doesn't need to be constrained
    ocm.setAbsoluteYAxisTolerance(angle_tolerance); // pitch needs to be constrained less than 90 degrees
    //ocm.setAbsoluteZAxisTolerance(2 * Math.PI);
    ocm.setAbsoluteZAxisTolerance(angle_tolerance); // yaw needs to be constrained less than 90 degrees
    ocm.setLinkName(target_link);
    ocm.setOrientation(start_pose.getOrientation());
    ocm.setWeight(1);

    orientation_constraints.add(ocm);
    pathConstraints.setOrientationConstraints(orientation_constraints);
    return pathConstraints;
  }

  public static GetCartesianPathRequest makeCartesianPath(Pose pose, Pose newPose, String target_link, String base_link, String group) {
    GetCartesianPathRequest req = new GetCartesianPathRequest();
    Header header = new Header(0, new Time(), base_link);

    RobotState robot_state = new RobotState();

    JointState joint_state = new JointState();
    joint_state.setEffort(new double[0]);
    joint_state.setPosition(new double[0]);
    joint_state.setVelocity(new double[0]);

    robot_state.setIsDiff(true);
    robot_state.setJointState(joint_state);

    ArrayList<Pose> waypoints = makeStraightPath(pose, newPose);

    req.setLinkName(target_link);
    req.setGroupName(group);
//    if (target_link.equals("r_wrist_roll_link")) {
//      req.setGroupName("right_arm");
//    } else {
//      req.setGroupName("left_arm");
//    }

    req.setAvoidCollisions(false);
    req.setHeader(header);
    req.setStartState(robot_state);
    req.setMaxStep(.02); // CR: I remember 0.02 is also set somewhere else for max step. Global var?
    req.setJumpThreshold(-1);
    req.setWaypoints(waypoints);
    req.setPathConstraints(new Constraints());

    return req;
  }

  public static ArrayList<Pose> makeStraightPath(Pose start_pose, Pose end_pose) {
    ArrayList<Pose> paths = new ArrayList<>();
    Point pos_diff = new Point();
    pos_diff.setX(end_pose.getPosition().getX() - start_pose.getPosition().getX());
    pos_diff.setY(end_pose.getPosition().getY() - start_pose.getPosition().getY());
    pos_diff.setZ(end_pose.getPosition().getZ() - start_pose.getPosition().getZ());
    double mag_diff = Math.sqrt(pos_diff.getX() * pos_diff.getX() + pos_diff.getY() * pos_diff.getY() + pos_diff.getZ() * pos_diff.getZ());
    int n_waypoints = (int) (mag_diff / .02); // make 2 cm at a time waypoints // 1cm might be better // CR: Previous CR refers to this
    for (int i = 0; i < n_waypoints; i++) {
      double fraction_of_way = ((double) i) / n_waypoints;
      Point waypoint_pos = new Point();

      //Quaternion q = Transformations.quat4dToQuaternion(slerp(fraction_of_way, Transformations.quaternionToQuat4d(start_pose.getOrientation()), Transformations.quaternionToQuat4d(end_pose.getOrientation())));
      Quaternion q = start_pose.getOrientation(); //if you wanted no twisting
      waypoint_pos.setX(start_pose.getPosition().getX() + fraction_of_way * pos_diff.getX());
      waypoint_pos.setY(start_pose.getPosition().getY() + fraction_of_way * pos_diff.getY());
      waypoint_pos.setZ(start_pose.getPosition().getZ() + fraction_of_way * pos_diff.getZ());
      paths.add(new Pose(waypoint_pos, q));
    }
    return paths;
  }

  //this is taken from fttp://worldwind31.arc.nasa.gov/svn/trunk/WorldWind/src/gov/nasa/worldwind/geom/Quaternion.java
  // I hope nasa is ok with me being too lazy to import an interpolation functions for quaternions properly.
  // for future reference, a useful package to import might be remixlab.proscene (found https://code.google.com/p/proscene/)
  //this function rotates a quaternion a fraction (0 to 1) between the first and second quaternion
  public static Quat4d slerp(double fraction, Quat4d value1, Quat4d value2) {
    if (fraction < 0.0) {
      return value1;
    } else if (fraction > 1.0) {
      return value2;
    }

    double dot = value1.x * value2.x + value1.y * value2.y + value1.z * value2.z + value1.w * value2.w;
    double x2, y2, z2, w2;
    if (dot < 0.0) {
      dot = 0.0 - dot;
      x2 = 0.0 - value2.x;
      y2 = 0.0 - value2.y;
      z2 = 0.0 - value2.z;
      w2 = 0.0 - value2.w;
    } else {
      x2 = value2.x;
      y2 = value2.y;
      z2 = value2.z;
      w2 = value2.w;
    }

    double t1, t2;

    final double EPSILON = 0.0001;
    if ((1.0 - dot) > EPSILON) // standard case (slerp)
    {
      double angle = Math.acos(dot);
      double sinAngle = Math.sin(angle);
      t1 = Math.sin((1.0 - fraction) * angle) / sinAngle;
      t2 = Math.sin(fraction * angle) / sinAngle;
    } else // just lerp
    {
      t1 = 1.0 - fraction;
      t2 = fraction;
    }

    return new Quat4d(
            (value1.x * t1) + (x2 * t2),
            (value1.y * t1) + (y2 * t2),
            (value1.z * t1) + (z2 * t2),
            (value1.w * t1) + (w2 * t2));
  }

  /**
   * Make joint constraints from the robot state, using
   * only the specified joints.
   *
   * @param robot_state
   * @param joints_to_use joints to use -- uses all if null
   * @return
   */
  public static Constraints makeJointConstraints(RobotState robot_state,
                                                 List<String> joints_to_use) {
    // construct joint constraints
    List<JointConstraint> joint_constraints = new ArrayList<>();
    JointConstraint joint_constraint;
    JointState joint_state = robot_state.getJointState();
    double tolerance_above = 0.01;
    double tolerance_below = 0.01;
    double weight = 1.0;
    for (int i = 0; i < joint_state.getName().size(); ++i) {

      if (joints_to_use != null && !joints_to_use.contains(joint_state.getName().get(i))) {
        continue;
      }

      joint_constraint = new JointConstraint(joint_state.getName().get(i),
              joint_state.getPosition()[i],
              tolerance_above, tolerance_below, weight);
      joint_constraints.add(joint_constraint);
    }

    Constraints goal_constraints = new Constraints();
    goal_constraints.setJointConstraints(joint_constraints);
    return goal_constraints;
  }

  /**
   * Make joint constraints from a list of joint names
   * and joint trajectory points. The joint names and traj point's
   * position fields must be of the same length. Use to joints_to_use arg
   * to filter for desired joints.
   *
   * @param joint_names
   * @param joint_traj_point
   * @param joints_to_use    joints to use -- uses all if null
   * @return
   */
  public static Constraints makeJointConstraints(List<String> joint_names,
                                                 JointTrajectoryPoint joint_traj_point,
                                                 List<String> joints_to_use) {
    // construct joint constraints
    List<JointConstraint> joint_constraints = new ArrayList<>();
    JointConstraint joint_constraint;
    double tolerance_above = 0.01;
    double tolerance_below = 0.01;
    double weight = 1.0;
    for (int i = 0; i < joint_names.size(); ++i) {

      if (joints_to_use != null && !joints_to_use.contains(joint_names.get(i))) {
        continue;
      }
      joint_constraint = new JointConstraint(joint_names.get(i),
              joint_traj_point.getPositions()[i],
              tolerance_above, tolerance_below, weight);
      joint_constraints.add(joint_constraint);
    }

    Constraints goal_constraints = new Constraints();
    goal_constraints.setJointConstraints(joint_constraints);
    return goal_constraints;
  }

  /**
   * Construct a basic MotionPlanRequest. User must still call
   * setGroupName and setGoalConstraints as well as override any
   * other optional parameters.
   * @param planningTime amount of time allowed for planning a trajectory
   * @return
   */
  public static MotionPlanRequest makeDefaultMotionPlanRequest(double planningTime) {

    WorkspaceParameters ws_params = new WorkspaceParameters(new Vector3(0.0, 0.0, 0.0), new Vector3(0.0, 0.0, 0.0));

    //TODO: should we just get current robot state
    RobotState robot_state = new RobotState();
    JointState js = new JointState();
    js.setPosition(new double[0]);
    js.setVelocity(new double[0]);
    js.setEffort(new double[0]);
    robot_state.setJointState(js);
    robot_state.setIsDiff(true);

    MotionPlanRequest req = new MotionPlanRequest();
    req.setStartState(robot_state);
    req.setWorkspaceParameters(ws_params);
    req.setNumPlanningAttempts(1);
    req.setAllowedPlanningTime(planningTime);

    return req;
  }

  /**
   * The above function, but now with input from your current state
   *
   * @param robot_state
   * @param joint_state
   * @param planningTime amount of time allowed for planning a trajectory
   * @return
   */
  public static MotionPlanRequest makeMotionPlanRequest(RobotState robot_state, JointState joint_state, double planningTime) {
    WorkspaceParameters ws_params = new WorkspaceParameters(new Vector3(0.0, 0.0, 0.0), new Vector3(0.0, 0.0, 0.0));

    List<String> names = new ArrayList<>();
    joint_state.setName(names);
    robot_state.setJointState(joint_state);
    robot_state.setIsDiff(true);

    MotionPlanRequest request = new MotionPlanRequest();
    request.setStartState(robot_state);
    request.setWorkspaceParameters(ws_params);
    request.setNumPlanningAttempts(1);
    request.setAllowedPlanningTime(planningTime);

    return request;
  }

}
