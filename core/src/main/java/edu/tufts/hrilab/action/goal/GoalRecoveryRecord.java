package edu.tufts.hrilab.action.goal;

import java.io.Serializable;
import java.util.List;
import java.util.Objects;

/** Previous-session work requiring operator reconciliation, never an executable goal. */
public final class GoalRecoveryRecord implements Serializable {
  private static final long serialVersionUID = 1L;

  public final String sessionId;
  public final long goalId;
  public final String predicate;
  public final GoalStatus status;
  public final List<GoalSchedulingExplanation.ResourceClaim> claims;

  public GoalRecoveryRecord(String sessionId, long goalId, String predicate, GoalStatus status,
                            List<GoalSchedulingExplanation.ResourceClaim> claims) {
    this.sessionId = Objects.requireNonNull(sessionId);
    this.goalId = goalId;
    this.predicate = Objects.requireNonNull(predicate);
    this.status = Objects.requireNonNull(status);
    this.claims = List.copyOf(claims);
  }
}
