package edu.tufts.hrilab.action.goal;

import java.io.Serializable;
import java.util.List;
import java.util.stream.Collectors;

/** A timeout is not an acknowledgment that a robot or primitive has stopped. */
public final class GoalManagerShutdownReport implements Serializable {
  private static final long serialVersionUID = 1L;

  public enum Outcome { COMPLETE, TIMED_OUT, INTERRUPTED, FAILED }

  public final Outcome outcome;
  public final boolean interruptionRequested;
  public final String failure;
  public final String phase;
  private final List<GoalInfo> unfinishedGoals;

  public GoalManagerShutdownReport(Outcome outcome, boolean interruptionRequested,
                                   String failure, String phase, List<GoalInfo> unfinishedGoals) {
    this.outcome = outcome;
    this.interruptionRequested = interruptionRequested;
    this.failure = failure;
    this.phase = phase;
    this.unfinishedGoals = unfinishedGoals.stream().map(GoalInfo::new).collect(Collectors.toList());
  }

  public List<GoalInfo> getUnfinishedGoals() {
    return unfinishedGoals.stream().map(GoalInfo::new).collect(Collectors.toList());
  }
}
