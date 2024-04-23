/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.manipulator.pr2;

import edu.tufts.hrilab.manipulator.generic.GenericManipulator;
import edu.tufts.hrilab.diarcros.msg.pr2_controllers_msgs.Pr2GripperCommand;
import edu.tufts.hrilab.diarcros.msg.pr2_controllers_msgs.Pr2GripperCommandGoal;
import edu.tufts.hrilab.diarcros.pr2.RGripperControllerGripperActionNode;
import org.ros.actionlib.state.SimpleClientGoalState;
import org.ros.exception.RosException;

import java.util.concurrent.TimeUnit;

public class PR2RightGripper extends GenericManipulator {
  RGripperControllerGripperActionNode r_gripper;

  public PR2RightGripper() {
    super();
    r_gripper = RGripperControllerGripperActionNode.getInstance();

    r_gripper.waitForNode();
    maxGraspWidthMeters = 0.09f;
  }

  @Override
  public boolean moveGripper(float position) {
    try {
      Pr2GripperCommand command = new Pr2GripperCommand(position, -1);
      Pr2GripperCommandGoal command_goal = new Pr2GripperCommandGoal(command);

      // the wait for parts of this method are here because the gripper action
      // doesn't behave correctly
      double pos_diff_thresh = 0.001;
      double pos_change_thresh = 0.0001;
      SimpleClientGoalState state;
      pr2_controllers_msgs.Pr2GripperCommandResult result;

      boolean reached_goal;
      double last_pos;
      double pos_diff;
      double pos_change;

      r_gripper.sendGoal(command_goal);
      log.debug("Right gripper moving...");
      r_gripper.waitForResult(3000l, TimeUnit.MILLISECONDS);
      log.debug("Right gripper done.");
      result = r_gripper.getResult();
      if (result == null) {
        r_gripper.cancelAllGoals();
        log.warn("Gripper timeout");
        return false;
      }
      reached_goal = result.getReachedGoal();
      last_pos = result.getPosition();
      pos_diff = Math.abs(position - last_pos);
      pos_change = 1.0;
      while (pos_diff > pos_diff_thresh && pos_change > pos_change_thresh) {
        r_gripper.sendGoal(command_goal);
        r_gripper.waitForResult(3000l, TimeUnit.MILLISECONDS);
        result = r_gripper.getResult();
        if (result == null) {
          r_gripper.cancelAllGoals();
          log.warn("Gripper timeout");
          return false;
        }
        reached_goal = result.getReachedGoal();
        pos_change = Math.abs(last_pos - result.getPosition());
        last_pos = result.getPosition();
        pos_diff = Math.abs(position - last_pos);
        log.trace(String.format("move gripper info pos: %f. pos_diff: %f. reached goal: %b. stalled: %b.", last_pos, pos_diff, reached_goal, result.getStalled()));
      }
      state = r_gripper.getState();
      log.debug(String.format("r_gripper goal state: %s.", state.getState().toString()));

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
      return (float) r_gripper.getResult().getPosition();
    } catch (RosException re) {
      log.warn("Error getting gripper position.", re);
      return -1;
    }
  }

  @Override
  public float getGoalGripperPosition() {
    try {
      return (float) r_gripper.getResult().getPosition();
    } catch (RosException e) {
      log.warn("Error getting goal gripper position.", e);
      return -1;
    }
  }

  @Override
  public float getCurrentGripperEffort() {
    try {
      return (float) r_gripper.getResult().getEffort();
    } catch (RosException e) {
      log.warn("Error getting gripper effort.", e);
      return -1;
    }
  }

  @Override
  public void shutdown() {
    // No ROS nodes (?), so nothing to do here
  }

}
