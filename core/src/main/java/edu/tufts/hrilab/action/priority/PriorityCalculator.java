/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.priority;

import edu.tufts.hrilab.action.ActionInterpreter;

/**
 *
 * @author willie
 */
public interface PriorityCalculator {
  
  public double calculate(ActionInterpreter action);
  
}
