/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.execution;

public enum ExecutionType {
  /*
  ACT:  execute actions and update main state machine
  OBSERVE:  do not execute actions, observe conditions
  SIMULATE:  branch off state machine
             do not execute actions
             check pre / post conditions
  SIMULATE_ACT:  branch off state machine
                 create simulation environment
                 execute actions
                 check pre / post conditions
  SIMULATE_PERFORMANCE: used to estimate the expected performance of a task
                        branch off state machine - no belief (probably need to figure out how to incorporate belief)
                        sample execution from performance measures
                        check state machine for pre / post conditions
  */
  ACT,
  OBSERVE,
  SIMULATE,
  SIMULATE_ACT, // tmf: is this case necessary? seemingly it is used for physics simulator, but i'm not sure it's used
  SIMULATE_PERFORMANCE;


  public boolean isSimulation() {
    switch(this) {
      case ACT:
      case OBSERVE:
        return false;
      default:
        return true;
    }
  }

  public boolean shouldExecute() {
    switch(this) {
      case ACT:
      case SIMULATE_ACT:
        return true;
      default:
        return false;
    }
  }

  public boolean shouldObserveConditions() {
    switch (this) {
      case ACT:
      case OBSERVE:
      case SIMULATE_PERFORMANCE: // should only check state machine though
        return true;
      case SIMULATE:
      case SIMULATE_ACT:
      default:
        return false;
    }
  }
}
