/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.execution;

import edu.tufts.hrilab.action.ActionConstraints;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.justification.Justification;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class RootContext extends Context {
  private final static Logger log = LoggerFactory.getLogger(RootContext.class);

  /**
   * RootContext constructor
   *
   * @param constraints root action constraints (applies to all child contexts)
   * @param sm          state machine for root context
   */
  public RootContext(ActionConstraints constraints, StateMachine sm) {
    super(null, sm, "root", ExecutionType.ACT);
    this.constraints = constraints;
  }

  /**
   * RootContext constructor
   *
   * @param constraints root action constraints (applies to all child contexts)
   * @param sm          state machine for root context
   */
  public RootContext(ActionConstraints constraints, StateMachine sm, ExecutionType executionType) {
    super(null, sm, "root", executionType);
    this.constraints = constraints;
  }

  /**
   * Private constructor for creation of simulation root context.
   *
   * @param sm
   * @param executionType
   */
  private RootContext(RootContext caller, StateMachine sm, ExecutionType executionType) {
    super(caller, sm, "simroot", executionType);
  }

  /**
   * RootContext for simulations
   *
   * @param sm            state machine for simulation
   * @param executionType execution type for the new root (should be some simulation state)
   */
  public RootContext createSimulationRoot(StateMachine sm, ExecutionType executionType) {
    return new RootContext(this, sm, executionType);
  }

  @Override
  protected Context getNextStepForType() {
    return null;
  }

  @Override
  public void setStatus(ActionStatus eStatus, Justification justification) {

  }

  /**
   * Prune children that are terminated.
   * TODO: use historyLength
   *
   * @param historyLength ms of history to keep
   */
  public void prune(long historyLength) {
    log.debug("Pruning children...");
    int prePruneSize = childContexts.size();
    childContexts.removeIf(child -> child.isTerminated());
    int postPruneSize = childContexts.size();
    log.debug("Removed " + (prePruneSize - postPruneSize) + " of " + prePruneSize + " children.");
  }

  // this probably shouldn't be called
  @Override
  public RootContext copy(Context newParent) {
    return createSimulationRoot(newParent.stateMachine, getExecType());
  }
}

