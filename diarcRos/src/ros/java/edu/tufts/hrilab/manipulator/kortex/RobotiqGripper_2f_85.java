/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.manipulator.kortex;

import edu.tufts.hrilab.manipulator.generic.GenericManipulator;
import edu.tufts.hrilab.diarcros.kortex.GripperControllerGripperActionNode;
import edu.tufts.hrilab.diarcros.msg.control_msgs.GripperCommand;
import edu.tufts.hrilab.diarcros.msg.control_msgs.GripperCommandGoal;
import edu.tufts.hrilab.util.Util;
import control_msgs.GripperCommandResult;
import org.ros.actionlib.state.SimpleClientGoalState;
import org.ros.exception.RosException;

import java.util.HashMap;
import java.util.concurrent.TimeUnit;

public class RobotiqGripper_2f_85 extends GenericManipulator {
  GripperControllerGripperActionNode gripper;

  public RobotiqGripper_2f_85() {
    super();
    try {
      gripper = GripperControllerGripperActionNode.getInstance("/my_gen3", new HashMap<>());
      assert gripper != null;
      gripper.waitForNode();
      maxGraspWidthMeters = 0.08f;

      gripper.cancelAllGoals();
      moveGripper(maxGraspWidthMeters);
      moveGripper(0);
      moveGripper(maxGraspWidthMeters);

    } catch (Exception e) {
      log.error("Error setting up gripper: " + e.getMessage());
    }
  }

  @Override
  public boolean moveGripper(float position) {
    // let's ensure that our grasp is somewhere between 0 and max
    if (position < 0) {
      position = 0;
    } else if (position > maxGraspWidthMeters) {
      position = maxGraspWidthMeters;
    }
    // EAK: the gripper controller position value is wrong on the ros_kortex side. we want [0, 0.08] = [closed, open]
    // but it's currently doing this: [0.8, 0.0] = [closed, open]
    position = (-position + 0.08f)*10;

    try {
      GripperCommand command = new GripperCommand(position, -1); // -1 means do not effort limit
      GripperCommandGoal command_goal = new GripperCommandGoal(command);

      // the wait for parts of this method are here because the gripper action
      // doesn't behave correctly
      double pos_diff_thresh = 0.001;
      double pos_change_thresh = 0.0001;
      SimpleClientGoalState state;
      GripperCommandResult result;

      boolean reached_goal;
      double last_pos;
      double pos_diff;
      double pos_change;

      gripper.sendGoal(command_goal);
      log.debug("Gripper moving...");
      gripper.waitForResult(3000l, TimeUnit.MILLISECONDS);
      log.debug("Gripper done.");
      result = gripper.getResult();
      if (result == null) {
        gripper.cancelAllGoals();
        log.warn("Gripper timeout");
        return false;
      }
      reached_goal = result.getReachedGoal();
      last_pos = result.getPosition();
      pos_diff = Math.abs(position - last_pos);
      pos_change = 1.0;
      while (pos_diff > pos_diff_thresh && pos_change > pos_change_thresh) {
        gripper.sendGoal(command_goal);
        gripper.waitForResult(3000l, TimeUnit.MILLISECONDS);
        result = gripper.getResult();
        if (result == null) {
          gripper.cancelAllGoals();
          log.warn("Gripper timeout");
          return false;
        }
        reached_goal = result.getReachedGoal();
        pos_change = Math.abs(last_pos - result.getPosition());
        last_pos = result.getPosition();
        pos_diff = Math.abs(position - last_pos);
        log.trace(String.format("move gripper info pos: %f. pos_diff: %f. reached goal: %b. stalled: %b.", last_pos, pos_diff, reached_goal, result.getStalled()));
      }
      state = gripper.getState();
      log.debug(String.format("gripper goal state: %s.", state.getState().toString()));

      // TODO: FIXME: this doesn't work properly: return (state.getState() == SimpleClientGoalState.StateEnum.SUCCEEDED);
      if (result != null && Math.abs(position - result.getPosition()) < pos_diff_thresh) {
        return true;
      } else {
        return false;
      }
    } catch (RosException | InterruptedException e) {
      log.error("Error trying to move gripper.", e);
      return false;
    }
  }

  @Override
  public float getCurrentGripperPosition() {
    try {
      return (float) gripper.getResult().getPosition();
    } catch (RosException re) {
      log.warn(String.valueOf(re));
      return -1;
    }
  }

  @Override
  public float getGoalGripperPosition() {
    try {
      return (float) gripper.getResult().getPosition();
    } catch (RosException e) {
      log.warn(String.valueOf(e));
      return -1;
    }
  }

  @Override
  public float getCurrentGripperEffort() {
    try {
      return (float) gripper.getResult().getEffort();
    } catch (RosException e) {
      log.warn(String.valueOf(e));
      return -1;
    }
  }

  @Override
  public void shutdown() {
    // TODO
  }
}
