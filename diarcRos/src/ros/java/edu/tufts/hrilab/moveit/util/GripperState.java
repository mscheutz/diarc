/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.moveit.util;

public class GripperState {
  // Different grippers have different ways of representing basically all the same things.
  // Let's standardize that within DIARC here.
  double effort = -1;
  double current_position = -1;
  double goal_position = -1;

  public GripperState() {}
  public GripperState(double effort, double current_position, double goal_position) {
    this.effort = effort;
    this.current_position = current_position;
    this.goal_position = goal_position;
  }
}
