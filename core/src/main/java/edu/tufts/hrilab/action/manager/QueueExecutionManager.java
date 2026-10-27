/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.manager;

/** Compatibility entry point for priority-ordered, queued execution. */
public class QueueExecutionManager extends ExecutionManager {
  public QueueExecutionManager() {
    super(SchedulingPolicy.QUEUE);
  }
}
