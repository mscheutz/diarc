/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.recovery;

import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.fol.Predicate;

import java.util.HashSet;
import java.util.Set;

/**
 * Constraints that define when a RecoveryPolicy is applicable.
 */
public class PolicyConstraints {
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
}
