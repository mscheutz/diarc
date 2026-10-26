package edu.tufts.hrilab.action.goal;

import java.io.Serializable;

/** Result of waiting for a goal transition newer than a previously observed version. */
public final class GoalWaitResult implements Serializable {
  private static final long serialVersionUID = 1L;

  public enum Outcome {
    UPDATED, TERMINATED, TIMED_OUT, INTERRUPTED, UNKNOWN_GOAL
  }

  public final Outcome outcome;
  private final GoalInfo goalInfo;

  public GoalWaitResult(Outcome outcome, GoalInfo goalInfo) {
    this.outcome = java.util.Objects.requireNonNull(outcome);
    this.goalInfo = goalInfo == null ? null : new GoalInfo(goalInfo);
  }

  public GoalInfo getGoalInfo() {
    return goalInfo == null ? null : new GoalInfo(goalInfo);
  }
}
