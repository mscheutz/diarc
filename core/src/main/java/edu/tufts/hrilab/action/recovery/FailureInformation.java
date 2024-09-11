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
import java.util.Objects;

public class FailureInformation {
  private static Logger log = LoggerFactory.getLogger(FailureInformation.class);

  public Symbol actor;
  public RecoveryType contextType;
  public Predicate catchingContext;
  public boolean isCatchingContextParent = false;
  public Predicate goal;
  public List<Predicate> failedActions;
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

    //Recovery policy type and catching context type need to match (action vs goal)
    if (constraints.recoveryType != contextType) {
      return 0;
    }

    // if there are failedActions or excludedFailedActions constraints, and the failedAction is unknown, the policy can't be used
    if (failedActions.isEmpty() && (!constraints.failedActions.isEmpty() || !constraints.excludedFailedActions.isEmpty())) {
      return 0;
    }



    //Additional action recovery only checks
    if (constraints.recoveryType == RecoveryType.ACTION) {
      //If an explicit catching action signature is defined, this context must match it
      if (constraints.catchingAction != null) {
        if (catchingContext != null && catchingContext.instanceOf(constraints.catchingAction)) {
          ++matches;
        } else {
          return 0;
        }
      }
      //Otherwise, this context must be a direct parent of the failed context in order to match
      else if (!isCatchingContextParent) {
        return 0;
      }
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
      if (constraints.failedActions.stream().noneMatch(action -> failedActions.stream().anyMatch(action2 -> action2.instanceOf(action)))) {
        return 0;
      } else {
        ++matches;
      }
    }

    if (!constraints.excludedFailedActions.isEmpty()) {
      if (constraints.excludedFailedActions.stream().anyMatch(action -> failedActions.stream().anyMatch(action2 -> action2.instanceOf(action)))) {
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

  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (o == null || getClass() != o.getClass()) return false;
    FailureInformation failureInformation = (FailureInformation) o;
    return Objects.equals(actor, failureInformation.actor) &&
           Objects.equals(contextType, failureInformation.contextType) &&
           Objects.equals(catchingContext,failureInformation.catchingContext) &&
           isCatchingContextParent == failureInformation.isCatchingContextParent &&
           Objects.equals(goal, failureInformation.goal) &&
           Objects.equals(failedActions, failureInformation.failedActions) &&
           Objects.equals(failureReasons, failureInformation.failureReasons) &&
           Objects.equals(actionStatus, failureInformation.actionStatus);
  }

  @Override
  public int hashCode() {
    int result = 1;
    result = 31 * result + Objects.hashCode(actor);
    result = 31 * result + Objects.hashCode(contextType);
    result = 31 * result + Objects.hashCode(catchingContext);
    result = 31 * result + Objects.hashCode(goal);
    result = 31 * result + Objects.hashCode(failedActions);
    result = 31 * result + Objects.hashCode(failureReasons);
    result = 31 * result + Objects.hashCode(actionStatus);
    return 31 * result + (isCatchingContextParent ? 1 : 0);
  }
}
