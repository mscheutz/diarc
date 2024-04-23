/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.recovery;

import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.ActionConstraints;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.ParameterizedAction;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.execution.ActionContext;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.execution.util.ContextUtils;
import edu.tufts.hrilab.action.execution.GoalContext;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.selector.ActionSelector;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;

import java.util.*;

public class ExplorationGoalRecovery extends GoalRecovery {
  /**
   * Last action status received during shouldAttemptRecovery call.
   */
  private ActionStatus actionStatus;
  /**
   * Last justification received during shouldAttemptRecovery call. This is the justification for the actionStatus.
   */
  private Justification justification;
  /**
   * Information about the last failure that is trying to be recovered from.
   */
  private FailureInformation lastFailureInfo;
  /**
   * The last selected recovery policy action. It is cached here, so a new selection call
   * isn't needed for selectRecoveryPolicy, since the initial selection is made during
   * shouldAttemptRecovery.
   */
  private ActionDBEntry lastSelectedRecoveryAction;
  /**
   * Last action that failed that is trying to be recovered from. This is used for termination criteria
   * which is why it's separate from the FailureInformation.
   */
  private Predicate lastFailedAction;
  /**
   * Cache of failure reasons for lastFailedAction. Used to populate explorationCount for termination criteria.
   */
  private Map<Predicate, Integer> lastFailedActionReasons = new HashMap<>();
  /**
   * Max times to explore the same failure with no progress before giving up.
   */
  private int maxRepeatedExplorations = 5;

  public ExplorationGoalRecovery() {
    super();
  }

  /**
   * Keeps track of what explorations have been attempted which is used to calculate the termination criteria for recovery.
   *
   * @param failedAction
   * @param failureReason
   */
  private void updateLastRecoveryAttempt(Predicate failedAction, Predicate failureReason) {
    if (failedAction == null && lastFailedAction == null) {
      // null failed action (again)
      int count = lastFailedActionReasons.getOrDefault(failureReason, 0) + 1;
      lastFailedActionReasons.put(failureReason, count);
    } else if (failedAction == null && lastFailedAction != null) {
      // null failed action (first time in a row)
      lastFailedAction = null;
      lastFailedActionReasons.clear();
      lastFailedActionReasons.put(failureReason, 1);
    } else if (failedAction != null && lastFailedAction == null) {
      // new failed action (first in a row)
      lastFailedAction = failedAction;
      lastFailedActionReasons.clear();
      lastFailedActionReasons.put(failureReason, 1);
    } else if (lastFailedAction.equals(failedAction)) {
      // same failed action (again) -- keep track of failure reasons while same action keeps failing
      // this helps deal with cyclical failures for the same failed action
      int count = lastFailedActionReasons.getOrDefault(failureReason, 0) + 1;
      lastFailedActionReasons.put(failureReason, count);
    } else {
      // new failed action (first in a row)
      lastFailedAction = failedAction;
      lastFailedActionReasons.clear();
      lastFailedActionReasons.put(failureReason, 1);
    }
  }

  /**
   * Termination criteria for giving up on recovery.
   *
   * @return
   */
  private boolean hasExhaustedExplorationOptions(Predicate failedAction, Predicate failureReason) {
    if (failedAction == null && lastFailedAction == null) {
      // no exploration goal could be constructed (again)
      return lastFailedActionReasons.getOrDefault(failureReason, 0) > maxRepeatedExplorations;
    } else if (failedAction == null && lastFailedAction != null) {
      // no exploration goal could be constructed (first time in a row)
      return false;
    } else if (failedAction != null && lastFailedAction == null) {
      // exploration goal (first in a row)
      return false;
    } else if (lastFailedAction.equals(failedAction)) {
      return lastFailedActionReasons.getOrDefault(failureReason, 0) > maxRepeatedExplorations;
    } else {
      // exploration goal (first in a row)
      return false;
    }
  }

  @Override
  public boolean shouldAttemptRecovery(ActionStatus actionStatus, Justification justification, GoalContext goalContext) {
    // update local fields
    this.actionStatus = actionStatus;
    this.justification = justification;
    log.debug("Checking if should attempt recovery. " + actionStatus + "    " + justification.getFailureReason());

    // update termination criteria
    Predicate failedAction = getFailedAction(goalContext);
    Predicate failureReason = getFailureReasonPredicate();
    updateLastRecoveryAttempt(failedAction, failureReason);

    // check termination criteria
    if (hasExhaustedExplorationOptions(failedAction, failureReason)) {
      recoveryStatus = RecoveryStatus.EXIT;
      return false;
    }

    if (actionStatus.isFailure() && actionStatus != ActionStatus.FAIL) {
      // ActionStatus.FAIL is only the case if an action script explicitly exits during execution (i.e., exit(FAIL))
      // NOTE: ActionStatus.FAIL happens during the cancelGoal goal when no matching goal is found, and we don't want to recover in that case

      switch (actionStatus) {
        case FAIL_RECOVERY:
          // if failure recovery fails, just try to re-plan until termination criteria kicks in
          recoveryStatus = RecoveryStatus.PLAN;
          return true;
        case FAIL_PRECONDITIONS:
        case FAIL_RETURNVALUE:
        case FAIL_NOTFOUND:
          if (hasRecoveryPolicy(goalContext)) {
            recoveryStatus = RecoveryStatus.RECOVER;
            return true;
          } else {
            recoveryStatus = RecoveryStatus.EXIT;
            return false;
          }
        case FAIL_POSTCONDITIONS:
          // bring agent belief in line with world state (i.e., discrepancies)
          Set<Predicate> failureReasons = new HashSet<>(justification.getFailureReason());
          Utilities.correctBeliefState(failureReasons);

          if (hasRecoveryPolicy(goalContext)) {
            recoveryStatus = RecoveryStatus.RECOVER;
            return true;
          } else {
            recoveryStatus = RecoveryStatus.EXIT;
            return false;
          }
      }
    }

    log.debug("Not recovering from goal: " + goalContext.getCommand() + " with action status: " + actionStatus);
    recoveryStatus = RecoveryStatus.EXIT;
    return false;
  }

  @Override
  public ParameterizedAction selectNextAction(Goal goal, ActionConstraints constraints, StateMachine stateMachine) {
    return ActionSelector.getInstance().selectActionForGoal(goal, constraints, stateMachine);
  }

  @Override
  public ParameterizedAction selectRecoveryPolicy(GoalContext goalContext) {
    // populate arguments for ParameterizedAction
    Map<String, Object> arguments = new HashMap<>();
    List<ActionBinding> inputArgs = lastSelectedRecoveryAction.getInputRoles();
    arguments.put(inputArgs.get(0).getName(), lastFailureInfo.actor);
    arguments.put(inputArgs.get(1).getName(), lastFailureInfo.failedAction);
    arguments.put(inputArgs.get(2).getName(), lastFailureInfo.failureReasons);
    arguments.put(inputArgs.get(3).getName(), lastFailureInfo.goal);


    ParameterizedAction recoveryPolicy = new ParameterizedAction(lastSelectedRecoveryAction, arguments);
    return recoveryPolicy;
  }

  /**
   * Checks if there are any valid recovery policies for the current failure. If there is at least one
   * policy available, selects one, and caches it for later use in selectRecoveryPolicy.
   * @param goalContext
   * @return
   */
  private boolean hasRecoveryPolicy(GoalContext goalContext) {
    // don't attempt to recover if no recovery policy exists
    lastSelectedRecoveryAction = selectRecoveryAction(goalContext);
    if (lastSelectedRecoveryAction == null) {
      log.debug("Not recovering. No recovery policy could be found. Not recovering from goal: " + goalContext.getGoal());
      return false;
    }

    return true;
  }

  /**
   * Gets all valid recovery policy actions from the Database.
   * @param goalContext
   * @return
   */
  private ActionDBEntry selectRecoveryAction(GoalContext goalContext) {
    // populate policy constraints based on current goal context
    lastFailureInfo = new FailureInformation();
    lastFailureInfo.goal = goalContext.getGoal().getPredicate();
    lastFailureInfo.actor = goalContext.getGoal().getActor();
    lastFailureInfo.actionStatus = actionStatus;
    lastFailureInfo.failedAction = getFailedAction(goalContext);
    lastFailureInfo.failureReasons = justification.getFailedConditions();

    // look up recovery policy options
    List<ActionDBEntry> recoveryPolicies = Database.getRecoveryPolicyDB().getRecoveryPolicies(lastFailureInfo);

    if (recoveryPolicies.isEmpty()) {
      return null;
    } else if (recoveryPolicies.size() > 1) {
      log.warn("More than one valid recovery policy found. Using first one in list.");
    }

    return recoveryPolicies.get(0);
  }

  /**
   * Helper method to find the failed action in the context tree, and return
   * the Predicate form of the failed action.
   *
   * @param goalContext
   * @return
   */
  private Predicate getFailedAction(GoalContext goalContext) {
    Predicate failedActionPredicate;
    if (this.actionStatus == ActionStatus.FAIL_NOTFOUND) {
      // failed to plan or find action to execute in pursuit of goal -- no particular action failed
      failedActionPredicate = goalContext.getGoal().getPredicate();
    } else {
      // get failed action context. only need to check from goal context's current child (don't care about contexts from previous attempts).
      ActionContext failedAction = ContextUtils.getActionContext(goalContext.getChildContexts().getCurrent(), justification.getStep(), this.actionStatus);
      if (failedAction == null) {
        log.warn("Could not determine failed action.");
        return null;
      }

      // TODO: HACK: remove this polycraft specific hack
      if (justification.getStep().getName().equals("teleport_to")) {
        Context parent = failedAction.getParentContext();
        while (parent != null && !parent.isAction()) {
          parent = parent.getParentContext();
        }
        failedAction = (ActionContext) parent;
      }
      failedActionPredicate = failedAction.getSignatureInPredicateForm();
    }

    return failedActionPredicate;
  }

  /**
   * Helper method to return a single Predicate based on the failure Justification.
   * @return
   */
  private Predicate getFailureReasonPredicate() {
    Predicate failureReasonPred;
    List<Predicate> failureReasons = justification.getFailedConditions();
    if (failureReasons.isEmpty()) {
      failureReasonPred = Factory.createPredicate("unknown()");
    } else if (failureReasons.size() > 1) {
      failureReasonPred = Factory.createPredicate("and", failureReasons);
    } else {
      failureReasonPred = new Predicate(failureReasons.get(0));
    }
    return failureReasonPred;
  }
}
