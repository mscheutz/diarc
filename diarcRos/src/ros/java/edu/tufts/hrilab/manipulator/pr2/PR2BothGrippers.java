/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.manipulator.pr2;

import edu.tufts.hrilab.manipulator.generic.GenericManipulator;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class PR2BothGrippers extends GenericManipulator {
  private PR2LeftGripper l_gripper;
  private PR2RightGripper r_gripper;

  /**
   * Executor to execute both grippers asynchronously.
   */
  private ExecutorService executorService = Executors.newCachedThreadPool();

  public PR2BothGrippers() {
    super();
    l_gripper = new PR2LeftGripper();
    r_gripper = new PR2RightGripper();
  }

  @Override
  public boolean moveGripper(float position) {
    List<Callable<Boolean>> tasks = new ArrayList<>();
    tasks.add(() -> l_gripper.moveGripper(position));
    tasks.add(() -> r_gripper.moveGripper(position));

    try {
      List<Future<Boolean>> futures = executorService.invokeAll(tasks);

      // check that all tasks finished successfully
      for (Future<Boolean> f : futures) {
        if (!f.get()) {
          return false;
        }
      }
    } catch (InterruptedException | ExecutionException e) {
      log.error("[moveGripper] exception caught.", e);
      return false;
    }

    return true;
  }

  @Override
  public float getCurrentGripperPosition() {
    log.warn("Returning value of l_gripper.");
    return l_gripper.getCurrentGripperPosition();
  }

  @Override
  public float getGoalGripperPosition() {
    return l_gripper.getGoalGripperPosition();
  }

  @Override
  public float getCurrentGripperEffort() {
    return l_gripper.getCurrentGripperEffort();
  }

  @Override
  public void shutdown() {
    // No ROS nodes (?), so nothing to do here
  }
}
