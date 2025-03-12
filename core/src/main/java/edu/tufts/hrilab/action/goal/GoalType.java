/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.goal;

public enum GoalType {
  STANDARD(false,false),
  SCHEDULED(true,false),
  PERSISTENT(false,true),
  PERSISTENTSCHEDULED(true,true),
  SCHEDULEDPERSISTENTSCHEDULED(true,true);

  /**
   * if true, this goal has been scheduled to occur at some future time after submission
   */
  private final boolean scheduled;
  /**
   * If true, this goal is wrapped as a persistent goal (i.e., while loop) and executed until failure or cancellation/suspension.
   */
  private final boolean persistent;

  GoalType(boolean isScheduled, boolean isPersistent) {
    scheduled = isScheduled;
    persistent = isPersistent;
  }

  /**
   * Getter method for whether this enum instance represents a scheduled goal.
   */
  public boolean isScheduled() {
    return scheduled;
  }

  /**
   * Getter method for whether this enum instance represents a persistent goal.
   */
  public boolean isPersistent() {
    return persistent;
  }
}