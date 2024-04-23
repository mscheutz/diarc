/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.execution.Context;

public class AndContext extends Context {

  public AndContext(Context c, StateMachine sm) {
    super(c, sm, "AND");
  }

  @Override
  protected Context getNextStepForType() {
    switch (getExecType()) {
      case SIMULATE_ACT:
      case ACT:
        Context next = childContexts.getNextAndIncrement();
        if (next != null) {
          return next; // Execute whatever's inside the AND
        } else {
          // AND all child logical values and set this as the AND's logical value
          boolean value = childContexts.allMatch(child -> child.getLogicalValue());
          this.setLogicalValue(value);
          return null; // We're done here...
        }
      default:
        return childContexts.getNextAndIncrement(); // get each child context
    }
  }

  @Override
  public AndContext copy(Context newParent) {
    AndContext newAnd = new AndContext(newParent, newParent.getStateMachine());
    for (Context child : childContexts.getChildrenContexts()) {
      newAnd.childContexts.add(child.copy(newAnd));
    }
    return newAnd;
  }

}
