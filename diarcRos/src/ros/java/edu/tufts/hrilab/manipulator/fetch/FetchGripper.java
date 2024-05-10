/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.manipulator.fetch;

import edu.tufts.hrilab.manipulator.generic.GenericManipulator;
import edu.tufts.hrilab.diarcros.fetch.GripperControllerGripperActionNode;
import edu.tufts.hrilab.diarcros.msg.control_msgs.GripperCommand;
import edu.tufts.hrilab.diarcros.msg.control_msgs.GripperCommandGoal;
import org.ros.actionlib.state.SimpleClientGoalState;
import org.ros.exception.RosException;

import java.util.concurrent.TimeUnit;

public class FetchGripper extends GenericManipulator {

  GripperControllerGripperActionNode gripper;

  public FetchGripper() {
    super();
    gripper = GripperControllerGripperActionNode.getInstance();

    gripper.waitForNode();
    maxGraspWidthMeters = 0.09f;
  }

  @Override
  public boolean moveGripper(float position) {

    GripperCommand command = new GripperCommand(position, 60);
    GripperCommandGoal command_goal = new GripperCommandGoal(command);

    // the wait for parts of this method are here because the gripper action
    // doesn't behave correctly
    double pos_diff_thresh = 0.0001;
    double pos_change_thresh = 0.000001; //0.00001;
    SimpleClientGoalState state;

    try {
      gripper.sendGoal(command_goal);
      log.debug("Gripper moving...");
      gripper.waitForResult(20, TimeUnit.SECONDS);
      log.debug("Gripper done.");

      control_msgs.GripperCommandResult result = gripper.getResult();
      log.debug("gripper reached goal: " + result.getReachedGoal());
      if (!result.getReachedGoal()) {
        try {
          Thread.sleep(100);
          result = gripper.getResult();
          log.debug("gripper reached goal? " + result.getReachedGoal());
        } catch (InterruptedException e) {
        }
      }
      if (result == null) {
        log.warn("Gripper timeout");
        return false;
      }
      return result.getReachedGoal();
      //boolean reached_goal = result.getReachedGoal();
      //double last_pos = result.getPosition();
      //double pos_diff = Math.abs(position - last_pos);
      //double pos_change = 1.0;
      //while (pos_diff > pos_diff_thresh && pos_change > pos_change_thresh) {
      //  //            while (!reached_goal && pos_change > 0.0001) {
      //  gripper.sendGoal(command_goal);
      //  gripper.waitForResult(20, TimeUnit.SECONDS);
      //  result = gripper.getResult();
      //  if (result == null) {
      //    log.warn("Gripper timeout");
      //    return false;
      //  }
      //  reached_goal = result.getReachedGoal();
      //  pos_change = Math.abs(last_pos - result.getPosition());
      //  last_pos = result.getPosition();
      //  pos_diff = Math.abs(position - last_pos);
      //  log.debug(String.format("move gripper pos: %f. pos_diff: %f. pos_change: %f. reached goal: %b. stalled: %b.", last_pos, pos_diff, pos_change, reached_goal, result.getStalled()));
      //}
      //state = gripper.getState();
      //log.debug(String.format("gripper goal state: %s.", state.getState().toString()));
    } catch (RosException | InterruptedException e) {
      log.error("Error trying to move gripper.", e);
      return false;
    }

    // TODO: FIXME: this doesn't work properly: return (state.getState() == SimpleClientGoalState.StateEnum.SUCCEEDED);
    //return true;
  }

  @Override
  public float getCurrentGripperPosition() {
    try {
      return (float) gripper.getResult().getPosition();
    } catch (RosException re) {
      log.warn("getCurrentGripperPosition", re);
      return -1;
    }
  }

  @Override
  public float getGoalGripperPosition() {
    try {
      return (float) gripper.getResult().getPosition();
    } catch (RosException e) {
      log.warn("getGoalGripperPosition", e);
      return -1;
    }
  }

  @Override
  public float getCurrentGripperEffort() {
    try {
      return (float) gripper.getResult().getEffort();
    } catch (RosException e) {
      log.warn("getCurrentGripperEffort", e);
      return -1;
    }
  }

  @Override
  public void shutdown() {
    // No ROS nodes (?), so nothing to do here
  }
}
