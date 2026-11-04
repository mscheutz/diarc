/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.planner;

import ai.thinkingrobots.trade.TRADEService;

public interface PlanningAgent {
  @TRADEService
  public boolean isBusy();
  @TRADEService
  public void setBusy(boolean busy);
}
