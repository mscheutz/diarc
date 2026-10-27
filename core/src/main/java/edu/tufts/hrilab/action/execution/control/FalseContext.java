/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.state.StateMachine;

public class FalseContext extends Context {
  public FalseContext(Context parent, StateMachine sm) {
    super(parent, sm, "FALSE");
    setLogicalValue(false);
  }

  @Override
  protected Context getNextStepForType() {
    return null;
  }

  @Override
  public FalseContext copy(Context newParent) {
    FalseContext copy = new FalseContext(newParent, newParent.getStateMachine());
    copyInternal(copy);
    return copy;
  }

  @Override
  public void doStep() {
    setLogicalValue(false);
  }
}
