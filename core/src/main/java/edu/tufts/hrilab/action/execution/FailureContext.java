/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * FailureContext is an empty Context used to indicate that there was a failure setting up
 * a Context. For instance, a FailureContext is instantiated when it is not possible to find an
 * ActionDBEntry to setup an ActionContext. The FailureContext is instantiated instead and
 * contains information about the error (status, failconds, ...).
 *
 * @author luca
 */

package edu.tufts.hrilab.action.execution;

import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.justification.Justification;

public class FailureContext extends Context {
  private Justification failureJustification;
  private ActionStatus failureStatus;

  /**
   * FailureContext constructor. Initializes the context, sets the failconditions and exits.
   *
   * @param caller        parent context
   * @param status        (failure) status to exit with
   * @param justification failure conditions (if any)
   */
  FailureContext(Context caller, StateMachine sm, ActionStatus status, Justification justification) {
    super(caller, sm, status.toString());
    this.failureStatus = status;
    this.failureJustification = justification;
  }

  FailureContext(Context caller, StateMachine sm, ActionStatus status, Justification justification, ExecutionType executionType) {
    super(caller, sm, status.toString(), executionType);
    this.failureStatus = status;
    this.failureJustification = justification;
  }

  @Override
  public void doStep() {
    switch (this.getExecType()) {
      case SIMULATE_ACT:
      case ACT:
        this.setStatus(failureStatus, failureJustification);
        break;
      default:
        break; //TODO: need to figure out how to fail when not executing. for example if an action is not found, need to fail,
      //      but if failure is a part of a script, then don't want to execute
    }
  }

  @Override
  protected Context getNextStepForType() {
    return null;
  }

  @Override
  public FailureContext copy(Context newParent) {
    return new FailureContext(newParent, newParent.stateMachine, failureStatus, failureJustification);
  }
}
