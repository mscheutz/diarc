/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.priority;

import edu.tufts.hrilab.action.ActionInterpreter;

/**
 *
 * @author willie
 */
public class UtilityUrgencyPriorityCalculator implements PriorityCalculator {

  @Override
  public double calculate(ActionInterpreter action) {
    double elapsedTime = System.currentTimeMillis() - action.getActionStartTime();
    double allowedTime = action.getActionMaxTime();
    if (elapsedTime <= allowedTime) {
      double minUrg = action.getActionMinUrg();
      double maxUrg = action.getActionMaxUrg();
      double urgency = elapsedTime / allowedTime;
      urgency = urgency * (maxUrg - minUrg) + minUrg;
      double utility = action.getActionBenefit() - action.getActionCost();
      return urgency * utility;
    } else {
      // don't change priority if we've already gone over the time limit
      // or should it become greatly deprioritized?
      return action.getActionPriority();
    }
  }

}
