package edu.tufts.hrilab.action.goal;

import java.io.Serializable;
import java.util.List;

/** Current scheduling state, not a history of past scheduling decisions. */
public final class GoalSchedulingExplanation implements Serializable {
  private static final long serialVersionUID = 1L;

  public enum Location {
    UNKNOWN, SUBMITTING, LEARNING, PENDING, ACTIVE, SCHEDULED, PAST
  }

  public enum Reason {
    UNKNOWN_GOAL, SUBMITTING, LEARNING, SCHEDULED, TERMINATING, TERMINATED, STARTING, EXECUTING, SUSPENDED,
    ACTIVE_RESOURCE_CONFLICT, HIGHER_PRIORITY_PENDING_CONFLICT, READY
  }

  public static final class ResourceClaim implements Serializable {
    private static final long serialVersionUID = 1L;
    public final String provider;
    public final String resource;
    public final boolean required;
    public final boolean held;

    public ResourceClaim(String provider, String resource, boolean required, boolean held) {
      this.provider = provider;
      this.resource = resource;
      this.required = required;
      this.held = held;
    }
  }

  public static final class ResourceConflict implements Serializable {
    private static final long serialVersionUID = 1L;
    public final String provider;
    public final String resource;
    public final boolean goalHolds;
    public final boolean blockerHolds;

    public ResourceConflict(String provider, String resource, boolean goalHolds, boolean blockerHolds) {
      this.provider = provider;
      this.resource = resource;
      this.goalHolds = goalHolds;
      this.blockerHolds = blockerHolds;
    }
  }

  public static final class Blocker implements Serializable {
    private static final long serialVersionUID = 1L;
    private final GoalInfo goalInfo;
    public final Location location;
    public final List<ResourceConflict> resources;

    public Blocker(GoalInfo goalInfo, Location location, List<ResourceConflict> resources) {
      this.goalInfo = new GoalInfo(goalInfo);
      this.location = location;
      this.resources = List.copyOf(resources);
    }

    public GoalInfo getGoalInfo() {
      return new GoalInfo(goalInfo);
    }
  }

  public final long goalId;
  public final String policy;
  private final GoalInfo goalInfo;
  public final Location location;
  /** Zero-based priority order, or -1 when not pending. */
  public final int queuePosition;
  public final List<Reason> reasons;
  public final List<ResourceClaim> claims;
  public final List<Blocker> blockers;

  public GoalSchedulingExplanation(long goalId, String policy, GoalInfo goalInfo, Location location,
                                   int queuePosition, List<Reason> reasons,
                                   List<ResourceClaim> claims, List<Blocker> blockers) {
    this.goalId = goalId;
    this.policy = policy;
    this.goalInfo = goalInfo == null ? null : new GoalInfo(goalInfo);
    this.location = location;
    this.queuePosition = queuePosition;
    this.reasons = List.copyOf(reasons);
    this.claims = List.copyOf(claims);
    this.blockers = List.copyOf(blockers);
  }

  public GoalInfo getGoalInfo() {
    return goalInfo == null ? null : new GoalInfo(goalInfo);
  }
}
