/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.execution.control;

import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.execution.Context;

/**
 * @author willie
 */
public class TrueContext extends Context {

  public TrueContext(Context c, StateMachine sm) {
    super(c, sm, "TRUE");
  }

  @Override
  protected Context getNextStepForType() {
    return null;
  }

  @Override
  public TrueContext copy(Context newParent) {
    TrueContext newTrue = new TrueContext(newParent, newParent.getStateMachine());
    copyInternal(newTrue);
    return newTrue;
  }
}
