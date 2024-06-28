/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.interfaces;

import ai.thinkingrobots.trade.TRADEService;

import edu.tufts.hrilab.action.EffectType;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.Effect;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import org.apache.commons.lang3.tuple.Pair;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.util.List;

//TODO: brad: Anything that is an action should only operate on Symbols
//  it is unclear if these other non symbol methods should be here at the highest level
public interface ArmInterface {

  /**
   * Move the groupName to a pose.
   * Uses the default coordinate frame as set by JSON (defaulting to base_link).
   *
   * @return true on success, false on failure. This function blocks.
   */
  @TRADEService
  @Action
  Justification moveTo(String groupName, Point3d point, Quat4d orientation);

  /**
   * Move the groupName to specified poses(s). If group name only
   * contains a single arm, use a different moveTo().
   * <p>
   * Note that there's a baked-in assumption that the only two-arned nmachines will use a group name
   * that is checked against in the 'isOneArm()' method (currently just "arms"). Update this function if
   * this does not describe your use case. It also assumes that groupName "arms" can be described as independent
   * movement of groupName "left_arm" and groupName "right_arm". At this point in time this is a safe assumption
   * because of the state of the PR2's JSON config.
   * 
   * point_l, orientation_l refer to the left arm, point_r and orientation_r refer to the right.
   *
   * @return true on success, false on failure. This function blocks.
   */
  @TRADEService
  @Action
  Justification moveTo(String groupName, Point3d point_l, Quat4d orientation_l, Point3d point_r, Quat4d orientation_r);

  /**
   * Move arms(s) to location of chosen grasp specified in MemoryObject. Uses
   * the default "grasp_point(X)" constraint to select grasp points. Assumes
   * default coordinate frame as set by JSON, defaulting to base_link.
   * Note: gripper should already be open/closed before calling this method.
   *
   * This is an alternative method to moveTo that takes in POWER ref instead of MemoryObject.
   *
   * @return true on success, false on failure. This function blocks.
   */
  @TRADEService
  @Action
  Justification moveTo(String groupName, Symbol refId);

  /**[
   * Move arms(s) to pose defined by the refId and constrained by
   * a set of constraints. Assumes base_link coordinate frame. Note: gripper
   * should already be open/closed before calling this method.
   *
   * @param constraints a list of predicate constraints
   * @return true on success, false on failure. This function blocks.
   */
  @TRADEService
  @Action
  Justification moveTo(String groupName, Symbol refId, List<? extends Term> constraints);

  /**
   * Move a certain amount from the current location, without any squeezing or
   * releasing.
   *
   * @param groupName  the name of the arm or arms to move
   * @param point       the amount of change in base frame the object is moved
   * @param orientation the amount to rotate the object in goal state from its
   *                    current rotation in base frame
   * @return success or failure to move. This function blocks.
   */
  @TRADEService
  @Action
  Justification moveToRelative(String groupName, Point3d point, Quat4d orientation);

  /**
   * Close gripper(s) to specified amount and attach MemoryObject as a collision
   * object. Alternative method to graspObject that takes in POWER ref instead of MemoryObject.
   *
   * @return true on success, false on failure. This function blocks.
   */
  @TRADEService
  @Action
  Justification graspObject(String groupName, Symbol refId, float position);

  /**
   * Move arm(s) and open gripper(s) to specified amount and detach MemoryObject
   * as collision object. Alternative method to releaseObject that takes in POWER ref instead of MemoryObject.
   *
   * @return true on success, false on failure. This function blocks.
   */
  @TRADEService
  @Action
  Justification releaseObject(String groupName, Symbol refId, float position);

  /**
   * Point a groupName to an object.
   *
   * @return true on success, false on failure. This function blocks
   */
  @TRADEService
  @Action
  Justification pointTo(String groupName, Symbol objectRef);

  /**
   * Convenience method of getPose that finds the end-effector link based
   * on the passed in groupName.
   * @param groupName
   * @return
   */
  @TRADEService
  @Action
  Pair<Point3d, Quat4d> getEEPose(String groupName);

  /**
   * Get the point and orientation of a link on the robot.
   */
  @TRADEService
  @Action
  Pair<Point3d, Quat4d> getPose(String linkName);

  /**
   * Record current robot state (i.e., joint positions), and name it.
   *
   * @param poseName The name of the pose to be saved.
   * @return true on success, false on failure. This function blocks.
   */
  @TRADEService
  @Action
  @Effect(
            effect={"object(?poseName, pose)"},
            type= EffectType.SUCCESS)
    Justification recordPose(Symbol poseName);

  /**
   * Write the poses recorded via recordPose to file.
   *
   * @return true on success
   */
  @TRADEService
  @Action
  Justification saveEEPosesToFile(String filename);

  /**
   * Load poses previously saved to file and make them accessible via goToPose.
   */
  @TRADEService
  @Action
  void loadEEPosesFromFile(String filename);

  /**
   * Write the poses recorded via recordPose to file.
   *
   * @return true on success
   */
  @TRADEService
  @Action
  Justification savePosesToFile(String filename);

  /**
   * Load poses previously saved to file and make them accessible via goToPose.
   */
  @TRADEService
  @Action
  void loadPosesFromFile(String filename);

  /**
   * Move robot to previously recorded robot state/pose, using only the specified group name (e.g., right_arm).
   * Currently set to ignore gripper joints so it will leave grippers in current state of open/closed.
   *
   * @param groupName which arm(s) should move (rightArm, leftArm, arms, etc)
   * @param poseName  name of pose to move into
   * @return true on success, false on failure. This function blocks.
   */
  @TRADEService
  @Action
  Justification goToPose(String groupName, Symbol poseName);

  /**
   * Move robot to previously recorded robot state/pose.
   * Currently set to ignore gripper joints so it will leave grippers in current state of open/closed.
   *
   * @param poseName  name of pose to move into
   * @return true on success, false on failure. This function blocks.
   */
  @TRADEService
  @Action
  Justification goToPose(Symbol poseName);

  /**
   * Start recording a trajectory. Trajectory recording can be started here, the robot can be
   * physically manipulated, and then stopRecordingTrajectory can be called to stop the current
   * recording.
   *
   * @param trajectoryName The name of the trajectory to be saved.
   */
  @TRADEService
  @Action
  void startRecordingTrajectory(String trajectoryName);

  /**
   * Stop recording the currently running trajectory recording.
   */
  @TRADEService
  @Action
  void stopRecordingTrajectory();

  /**
   * Execute a previously recorded trajectory.
   *
   * @return true on success, false on failure. This function blocks.
   */
  @TRADEService
  @Action
  Justification executeTrajectory(String trajectoryName);

  /**
   * Take all trajectories saved via startRecordingTrajectory and save them to a file for later.
   *
   * @param filename Path to file
   * @return true on success, false on failure.
   */
  @TRADEService
  Justification saveTrajectoriesToFile(String filename);

  /**
   * Load trajectories from a file that was generated via saveTrajectoriesToFile(). The loaded
   * trajectories will then be executable via executeTrajectory().
   */
  @TRADEService
  void loadTrajectoriesFromFile(String filename);

  @TRADEService
  @Action
  Justification closeGripper(String groupName);

  @TRADEService
  @Action
  Justification openGripper(String groupName);

  @TRADEService
  @Action
  Justification moveGripper(String groupName, float meters);

  @TRADEService
  float getGripperPosition(String groupName);

  /**
   * Presses an object based off a given pose
   *
   * @param group_name         The arm group to do the pressing (i.e. "arm")
   * @param object_location    The location of the object relative to the base link
   * @param object_orientation The orientation of the object relative to the base link with the normal vector coming out
   *                           of the object
   * @return Condition Justification stating if the action is successful or not
   */
  @TRADEService
  @Action
  Justification pressObject(String group_name, Point3d object_location, Quat4d object_orientation);

  /**
   * Presses an object based off a given symbol
   *
   * @param group_name The arm group to do the pressing (i.e. "arm")
   * @param refID      Symbol representing the object to be pressed
   * @return Condition Justification stating if the action is successful or not
   */
  @TRADEService
  @Action
  Justification pressObject(String group_name, Symbol refID);
}
