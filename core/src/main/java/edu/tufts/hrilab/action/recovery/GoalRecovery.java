/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.recovery;

import edu.tufts.hrilab.action.ActionConstraints;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.ParameterizedAction;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.execution.GoalContext;
import edu.tufts.hrilab.action.justification.Justification;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public abstract class GoalRecovery {

  protected Logger log = LoggerFactory.getLogger(this.getClass());

  protected RecoveryStatus recoveryStatus = RecoveryStatus.PLAN;

  /**
   * Get current recovery status/mode.
   * @return
   */
  public final RecoveryStatus getStatus() {
    return recoveryStatus;
  }

  /**
   * Update recovery status based on what has happened since the last goalRecovery check-in.
   * For example, if in recovery mode, and a recovery policy has been executed, GoalRecovery needs
   * to be updated with the results of that recovery policy. This method provides the mechanism for that update.
   * While it's unlikely this method needs to be overridden, it can be in order to handle corner cases.
   *
   * @param goalContext
   */
  public void updateRecoveryStatus(GoalContext goalContext) {
    if (recoveryStatus == RecoveryStatus.RECOVER && goalContext.getChildContexts().getCurrent().isSuccess()) {
      // if in RECOVER mode, and last goal (i.e., recover goal) succeeded, go back to PLAN mode
      recoveryStatus = RecoveryStatus.PLAN;
    }
  }

  /**
   * Select the next action to be executed. This is only called in PLAN mode, not for RECOVER mode.
   *
   * @param goal
   * @param constraints
   * @param stateMachine
   * @return
   */
  public abstract ParameterizedAction selectNextAction(Goal goal, ActionConstraints constraints, StateMachine stateMachine);

  /**
   * Determines if a recovery attempt should be made, and locally sets the recovery mode (e.g., PLAN, RECOVER).
   * @param actionStatus
   * @param justification
   * @param goalContext
   * @return
   */
  public abstract boolean shouldAttemptRecovery(ActionStatus actionStatus, Justification justification, GoalContext goalContext);

  /**
   * Select a recovery policy from the Database of policies. Can return null
   * if there are no valid policies available. Only called when in RECOVER mode.
   * @return
   */
  public abstract ParameterizedAction selectRecoveryPolicy(GoalContext goalContext);
}
