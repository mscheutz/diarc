/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.recovery;

import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.List;

public class FailureInformation {
  private static Logger log = LoggerFactory.getLogger(FailureInformation.class);

  public Symbol actor;
  public Predicate goal;
  public Predicate failedAction;
  public List<Predicate> failureReasons;
  public ActionStatus actionStatus;

  /**
   * Checks if the recovery policy constraints are applicable to this failure information.
   *
   * @param constraints
   * @return
   */
  public boolean isApplicable(PolicyConstraints constraints) {
    return (getNumberOfConstraintMatches(constraints) > 0);
  }

  public int getNumberOfConstraintMatches(PolicyConstraints constraints) {
    int matches = 0;

    if (goal == null  || failureReasons == null || actionStatus == null) {
      log.warn("Incomplete failure information. Can not match against any policy constraints.");
      return 0;
    }

    // if there are failedActions or excludedFailedActions constraints, and the failedAction is unknown, the policy can't be used
    if (failedAction == null && ((!constraints.failedActions.isEmpty()) || !constraints.excludedFailedActions.isEmpty())) {
      return 0;
    }

    if (!constraints.actionStatuses.isEmpty()) {
      if (!constraints.actionStatuses.contains(actionStatus)) {
        return 0;
      } else {
        ++matches;
      }
    }

    if (!constraints.goals.isEmpty()) {
      if (constraints.goals.stream().noneMatch(g -> goal.instanceOf(g))) {
        return 0;
      } else {
        ++matches;
      }
    }

    if (!constraints.excludedGoals.isEmpty()) {
      if (constraints.excludedGoals.stream().anyMatch(g -> goal.instanceOf(g))) {
        return 0;
      } else {
        ++matches;
      }
    }

    if (!constraints.failedActions.isEmpty()) {
      if (constraints.failedActions.stream().noneMatch(action -> failedAction.instanceOf(action))) {
        return 0;
      } else {
        ++matches;
      }
    }

    if (!constraints.excludedFailedActions.isEmpty()) {
      if (constraints.excludedFailedActions.stream().anyMatch(action -> failedAction.instanceOf(action))) {
        return 0;
      } else {
        ++matches;
      }
    }

    if (!constraints.failureReasons.isEmpty()) {
      if (!failureReasons.stream().allMatch(reason -> constraints.failureReasons.stream().anyMatch(validReason -> reason.instanceOf(validReason)))) {
        return 0;
      } else {
        ++matches;
      }
    }

    if (!constraints.excludedFailureReasons.isEmpty()) {
      if (failureReasons.stream().anyMatch(reason -> constraints.excludedFailureReasons.stream().anyMatch(invalidReason -> reason.instanceOf(invalidReason)))) {
        return 0;
      } else {
        ++matches;
      }
    }

    return matches;
  }
}
