package edu.tufts.hrilab.action.manager;

/** Compatibility entry point for exclusive, non-queued execution. */
public class SingleGoalExecutionManager extends ExecutionManager {
  public SingleGoalExecutionManager() {
    super(SchedulingPolicy.SINGLE_GOAL);
  }
}
