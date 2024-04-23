/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.priority;

import edu.tufts.hrilab.action.ActionInterpreter;

/**
 *
 * @author willie
 */
public class LinearPriorityCalculator implements PriorityCalculator {

  @Override
  public double calculate(ActionInterpreter action) {
    // intentionally do nothing
    return 0.0;
  }
  
}
