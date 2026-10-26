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
  public Predicate goal;
  public List<Predicate> failedActions;
  public List<Predicate> failureReasons;
  public ActionStatus actionStatus;
  public List<String> excludedPolicies;

  // only used for action recovery (not goal recovery)
  public Predicate catchingContext;
  public boolean isCatchingContextParent = false;

  /**
   * Get the match score for this FailureInformation compared against the PolicyConstraints.
   * Any value >= 0 indicates a match. Negative values indicate no match.
   *
   * @param constraints
   * @return
   */
  public int getNumberOfConstraintMatches(PolicyConstraints constraints) {
    int matches = 0;

    if (goal == null  || failureReasons == null || actionStatus == null) {
      log.warn("Incomplete failure information. Can not match against any policy constraints.");
      return -1;
    }

    //If supplied policy is present in the excludedList (has exhausted its attempt count without success), do not match again
    if (excludedPolicies.contains(constraints.methodName)) {
      return -1;
    }

    //Recovery policy type and catching context type need to match (action vs goal)
    if (contextType == RecoveryType.ACTION && constraints.recoveryType != contextType) {
      return -1;
    }

    // if there are failedActions or excludedFailedActions constraints, and the failedAction is unknown, the policy can't be used
    if (failedActions.isEmpty() && (!constraints.failedActions.isEmpty() || !constraints.excludedFailedActions.isEmpty())) {
      return -1;
    }

    // Additional action recovery only checks
    if (constraints.recoveryType.isAction()) {
      // If an explicit catching action signature is defined, this context must match it
      if (!constraints.catchingActions.isEmpty()) {
        if (catchingContext != null && constraints.catchingActions.stream().anyMatch(a -> catchingContext.instanceOf(a))) {
          ++matches;
        } else {
          return -1;
        }
      } else if (!isCatchingContextParent) {
        // Otherwise, this context must be a direct parent of the failed context to match
        return -1;
      }
    }

    // if there are action statuses constraints, the failed action status must match one in the list
    if (!constraints.actionStatuses.isEmpty()) {
      if (!constraints.actionStatuses.contains(actionStatus)) {
        return -1;
      } else {
        ++matches;
      }
    }

    // if there are goals constraints, the failed goal must match one in the list
    if (!constraints.goals.isEmpty()) {
      if (constraints.goals.stream().noneMatch(g -> goal.instanceOf(g))) {
        return -1;
      } else {
        ++matches;
      }
    }

    // if there are excluded goals constraints, the failed goal must not match one in the list
    // this does not add to the match score
    if (!constraints.excludedGoals.isEmpty()) {
      if (constraints.excludedGoals.stream().anyMatch(g -> goal.instanceOf(g))) {
        return -1;
      }
    }

    // if there are failed actions constraints, at least one failed action must match one in the list
    if (!constraints.failedActions.isEmpty()) {
      if (constraints.failedActions.stream().noneMatch(action -> failedActions.stream().anyMatch(action2 -> action2.instanceOf(action)))) {
        return -1;
      } else {
        ++matches;
      }
    }

    // if there are excluded failed actions constraints, none of the failed actions can match any in the constraint list
    // excluded failed actions do not add to the match score
    if (!constraints.excludedFailedActions.isEmpty()) {
      if (constraints.excludedFailedActions.stream().anyMatch(action -> failedActions.stream().anyMatch(action2 -> action2.instanceOf(action)))) {
        return -1;
      }
    }

    // if there are failure reason constraints, all the failed action's failure reasons must be contained in the constraint list
    // if an action fails without failure reasons, it does not match
    if (!constraints.failureReasons.isEmpty()) {
      //TODO: change made in demo crunch - double check. Was it intended that a match would be made when a policy has explicit failure reasons but but the actual failure had no justification?
      if (failureReasons.isEmpty() || !failureReasons.stream().allMatch(reason -> constraints.failureReasons.stream().anyMatch(validReason -> reason.instanceOf(validReason)))) {
        return -1;
      } else {
        ++matches;
      }
    }

    // if there are excluded failure reason constraints, none of the failed action's failure reasons can match any in the constraint list
    // excluded failure reasons do not add to the match score
    if (!constraints.excludedFailureReasons.isEmpty()) {
      if (failureReasons.stream().anyMatch(reason -> constraints.excludedFailureReasons.stream().anyMatch(invalidReason -> reason.instanceOf(invalidReason)))) {
        return -1;
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
