/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.recovery;

import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.fol.Predicate;

import java.util.HashSet;
import java.util.Objects;
import java.util.Set;

/**
 * Constraints that define when a RecoveryPolicy is applicable.
 */
public class PolicyConstraints {
  /**
   * The DBEntry method name containing the recovery policy specification.
   */
  public String methodName;
  /**
   * Type of recovery policy - one of Action or Goal.
   * The corresponding recovery policy can be caught in and applied under contexts of this type.
   * Single exception is root goals which are direct action instructions.
   */
  public RecoveryType recoveryType = RecoveryType.GOAL;
  /**
   * The corresponding recovery policy is applicable to these goals.
   * Can be empty (i.e., applies to all goals).
   * Cannot be used at the same time as excludedGoals.
   */
  public Set<Predicate> goals = new HashSet<>();
  /**
   * The corresponding recovery policy is NOT applicable to these goals.
   * Can be empty (i.e., applies to all goals).
   * Cannot be used at the same time as goals.
   */
  public Set<Predicate> excludedGoals = new HashSet<>();
  /**
   * The corresponding recovery policy is applicable to these failed actions.
   * Can be empty (i.e., applies to all failed actions).
   * Cannot be used at the same time as excludedFailedActions.
   */
  public Set<Predicate> failedActions = new HashSet<>();
  /**
   * The corresponding recovery policy is NOT applicable to these failed actions.
   * Can be empty (i.e., applies to all failed actions).
   * Cannot be used at the same time as failedActions.
   */
  public Set<Predicate> excludedFailedActions = new HashSet<>();
  /**
   * The corresponding recovery policy is applicable to these failure reasons.
   * Can be empty (i.e., applies to all failure reasons).
   * Cannot be used at the same time as excludedFailureReasons. If an action failure has
   * more than one failure reason, a single policy must handle all the failure reasons.
   */
  public Set<Predicate> failureReasons = new HashSet<>();
  /**
   * The corresponding recovery policy is NOT applicable to these failure reasons.
   * Can be empty (i.e., applies to all failure reasons).
   * Cannot be used at the same time as failureReasons.
   */
  public Set<Predicate> excludedFailureReasons = new HashSet<>();
  /**
   * The corresponding recovery policy is applicable to these action statuses.
   * Can be empty (i.e., applies to all action statuses).
   */
  public Set<ActionStatus> actionStatuses = new HashSet<>();

  /**
   * (Action Recovery Only)
   * Indication of how to continue with execution after a recovery policy
   * has successfully completed. See PostRecoveryExecutionType for details.
   */
  public PostRecoveryExecutionType postRecoveryExecutionType = null;

  /**
   * (Action Recovery Only)
   * The corresponding recovery policy will defer matching to failure reasons
   * until/unless the failure propagates to an action context whose signature
   * matches one of the entries in this field.
   * Can be empty (i.e. applies to the direct parent of the failed context regardless of signature)
   */
  public Set<Predicate> catchingActions = new HashSet<>();

  /**
   * Maximum number of times this recovery policy can be executed for the same failure reasons under the same context
   * ActionRecovery RETRY: Attempt count resets when the reattempted action succeeds
   * GaolRecovery: Attempt count always increments and never resets. TODO if also want to check for progress (see notes in Recovery.java)
   */
  public int maximumAttempts = 1;

  /**
   * Returns true if the corresponding recovery policy is always applicable (i.e., empty recovery ASL block).
   * @return
   */
  public boolean alwaysApplicable() {
    return goals.isEmpty() && excludedGoals.isEmpty() &&
            failedActions.isEmpty() && excludedFailedActions.isEmpty() &&
            failureReasons.isEmpty() && excludedFailureReasons.isEmpty() &&
            actionStatuses.isEmpty() &&
            catchingActions.isEmpty();
  }

  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (o == null || getClass() != o.getClass()) return false;
    PolicyConstraints that = (PolicyConstraints) o;
    return maximumAttempts == that.maximumAttempts &&
            Objects.equals(methodName, that.methodName) &&
            recoveryType == that.recoveryType &&
            Objects.equals(goals, that.goals) &&
            Objects.equals(excludedGoals, that.excludedGoals) &&
            Objects.equals(failedActions, that.failedActions) &&
            Objects.equals(excludedFailedActions, that.excludedFailedActions) &&
            Objects.equals(failureReasons, that.failureReasons) &&
            Objects.equals(excludedFailureReasons, that.excludedFailureReasons) &&
            Objects.equals(actionStatuses, that.actionStatuses) &&
            postRecoveryExecutionType == that.postRecoveryExecutionType &&
            Objects.equals(catchingActions, that.catchingActions);
  }

  @Override
  public int hashCode() {
    return Objects.hash(methodName, recoveryType, goals, excludedGoals, failedActions,
            excludedFailedActions, failureReasons, excludedFailureReasons, actionStatuses,
            postRecoveryExecutionType, catchingActions, maximumAttempts);
  }
}
