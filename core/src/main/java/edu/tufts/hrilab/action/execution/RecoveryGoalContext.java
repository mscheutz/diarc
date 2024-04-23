/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.execution;

import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.ParameterizedAction;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.recovery.GoalRecovery;

public class RecoveryGoalContext extends GoalContext {
  private ParameterizedAction recoveryAction;

  public RecoveryGoalContext(Context caller, StateMachine sm, ParameterizedAction recoveryAction, ExecutionType executionType, GoalRecovery recovery) {
    super(caller, sm, new Goal(recoveryAction.getBoundActionSignature()), executionType, recovery);
    this.recoveryAction = recoveryAction;
  }

  @Override
  public void doStep() {
    ActionContext recoveryContext = new ActionContext(this, this.stateMachine, recoveryAction.getEntry(), recoveryAction.getBindings(), this.executionType, goal.getActor());
    childContexts.add(recoveryContext);
  }

  @Override
  protected void resetConcreteContext() {
    if (getExecType() != ExecutionType.SIMULATE_PERFORMANCE) {
      childContexts.clear();
    }
  }

  /**
   * Overriding setStatus to "catch" failures when a recovery operation should take place,
   * and also to convert un-caught failures to FAIL_RECOVERY so the parent goal context can handle
   * recovery failures differently from regular goal/action failures.
   *
   * TODO: This method is synchronized to handle canceling. Is there a better way to handle this?
   *
   * @param eStatus       the execution status of the currently running event
   * @param justification the reason for the context's execution status
   */
  @Override
  public synchronized void setStatus(ActionStatus eStatus, Justification justification) {

    if (eStatus == ActionStatus.INITIALIZED) {
      // allow context to be (re)set to INITIALIZED, which is required for a context to be reset for re-use
      super.setStatus(eStatus, justification);
    } else if (this.isTerminated()) {
      log.warn("Trying to set status of previously terminated context: " + this.getSignatureInPredicateForm()
              + ". Current status is: " + this.getStatus() + ", tried to set to: " + eStatus);
    } else if (!eStatus.isTerminated() || eStatus == ActionStatus.CANCEL || eStatus == ActionStatus.SUCCESS) {
      log.debug("Can't recover from action status: " + eStatus);
      super.setStatus(eStatus, justification);
    } else if (goalRecovery.shouldAttemptRecovery(eStatus, justification, this)) {
      log.debug("[setStatus] not propagating failure: " + eStatus);
      super.setStatus(ActionStatus.RECOVERY, justification);
    } else if (eStatus.isFailure()) {
      log.debug("[setStatus] changing failure status to FAIL_RECOVERY from: " + eStatus);
      super.setStatus(ActionStatus.FAIL_RECOVERY, justification);
    } else {
      log.debug("[setStatus] passing exit handling to super class: " + eStatus);
      super.setStatus(eStatus, justification);
    }
  }

  /**
   * Make a copy of this node in the execution tree for simulation purposes
   *
   * @param newParent new parent context
   * @return
   */
  @Override
  public RecoveryGoalContext copy(Context newParent) {
    // TODO: probably change the following so that it has the proper execution type
    RecoveryGoalContext newGoalContext = new RecoveryGoalContext(newParent, newParent.stateMachine, recoveryAction, newParent.getExecType(), goalRecovery);
    copyInternal(newGoalContext);
    return newGoalContext;
  }

}
