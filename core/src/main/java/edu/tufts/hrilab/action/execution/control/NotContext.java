/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.execution.Context;

public class NotContext extends Context {
  public NotContext(Context c, StateMachine sm) {
    super(c, sm, "NOT");
  }

  @Override
  protected Context getNextStepForType() {
    switch (this.getExecType()) {
      case SIMULATE_ACT:
      case ACT:
        if (childContexts.getNextIndex() == 0) {
          return childContexts.getNextAndIncrement(); // Return whatever is inside NOT (in order to evaluate it)
        } else {
          setLogicalValue(!childContexts.get(0).getLogicalValue());
          return null; // We're done here
        }
      default:
        return childContexts.getNextAndIncrement(); // get each child context, logical value doesn't matter for simulation capability
    }
  }

  /**
   * Override default implementation to ignore ActionStatus of NotContext, and
   * only depend on the logical value of the child context.
   *
   * @return
   */
  @Override
  public boolean getLogicalValue() {
    return (!childContexts.get(0).getLogicalValue());
  }

  @Override
  public NotContext copy(Context newParent) {
    NotContext newNot = new NotContext(newParent, newParent.getStateMachine());
    copyInternal(newNot);
    return newNot;
  }
}
