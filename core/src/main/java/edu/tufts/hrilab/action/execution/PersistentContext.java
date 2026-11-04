/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.execution;

import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.state.StateMachine;

/**
 * This represents the semantics of a persistent action in an action script.
 *
 * @author willie
 */
public class PersistentContext extends Context {

  public PersistentContext(Context c, StateMachine sm, Context child) {
    super(c, sm, "persistent");
    this.childContexts.add(child);
  }

  protected void setChild(Context child) {
    this.childContexts.clear();
    this.childContexts.add(child);
  }

  @Override
  protected Context getNextStepForType() {
    switch (this.getExecType()) {
      case SIMULATE_ACT:
      case ACT:
        //Don't reset context on resume
        if (this.getChildContexts().get(0).getStatus() != ActionStatus.RESUME) {
          this.childContexts.get(0).resetContext();
        }
        this.childContexts.setNextIndex(1); //Need to increment so notion of the "current" child is correct.
        return this.childContexts.get(0);
      default:
        return this.childContexts.getNextAndIncrement(); // get each child once

    }
  }

  @Override
  public PersistentContext copy(Context newParent) {
    return new PersistentContext(newParent, newParent.stateMachine, getChildContexts().getChildrenContexts().get(0));
  }

}
