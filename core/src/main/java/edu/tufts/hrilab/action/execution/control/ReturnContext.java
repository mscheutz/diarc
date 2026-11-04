/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.execution.Context;

/**
 * This is used to explicitly return from an action script early, just as you would from Java.
 * Here, the exit will only propagate up the context tree until an action script or asynchronous
 * context is encountered. This scoping is similar to Java in that a return only returns from a Java method.
 */
public class ReturnContext extends Context {

  public ReturnContext(Context c, StateMachine sm) {
    super(c,sm , "RETURN");
  }

  @Override
  public void doStep() {
    log.debug("[doStep] method entered.");

    setStatus(ActionStatus.RETURN);
  }

  @Override
  protected Context getNextStepForType() {
    return null;
  }

  @Override
  public ReturnContext copy(Context newParent) {
    ReturnContext newReturn = new ReturnContext(newParent, newParent.getStateMachine());
    copyInternal(newReturn);
    return newReturn;
  }
}
