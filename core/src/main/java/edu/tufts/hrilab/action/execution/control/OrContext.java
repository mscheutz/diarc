/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.execution.Context;

public class OrContext extends Context {

  public OrContext(Context c, StateMachine sm) {
    super(c, sm, "OR");
  }

  @Override
  protected Context getNextStepForType() {
    Context next = childContexts.getNextAndIncrement();
    switch (this.getExecType()) {
      case SIMULATE_ACT:
      case ACT:
        if (next != null) {
          return next; // Execute whatever's inside the OR
        } else {
          // OR all child logical values and set this as the OR's logical value
          boolean value = childContexts.anyMatch(child -> child.getLogicalValue());
          setLogicalValue(value);
          return null; // We're done here...
        }
      default:
        return childContexts.getNextAndIncrement(); // get each child context
    }
  }

  @Override
  public OrContext copy(Context newParent) {
    OrContext newOr = new OrContext(newParent, newParent.getStateMachine());
    copyInternal(newOr);
    return newOr;
  }

}
