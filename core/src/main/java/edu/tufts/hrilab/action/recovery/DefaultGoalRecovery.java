/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.recovery;

import edu.tufts.hrilab.action.ActionConstraints;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.ParameterizedAction;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.execution.GoalContext;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.selector.ActionSelector;

public class DefaultGoalRecovery extends GoalRecovery {

  @Override
  public ParameterizedAction selectNextAction(Goal goal, ActionConstraints constraints, StateMachine stateMachine) {
    return ActionSelector.getInstance().selectActionForGoal(goal, constraints, stateMachine);
  }

  @Override
  public boolean shouldAttemptRecovery(ActionStatus actionStatus, Justification justification, GoalContext goalContext) {
    return false;
  }

  @Override
  public ParameterizedAction selectRecoveryPolicy(GoalContext goalContext) {
    return null;
  }

}
