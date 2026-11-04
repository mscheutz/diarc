/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author dave
 */

package edu.tufts.hrilab.action.selector;

import java.util.concurrent.ThreadLocalRandom;
import java.util.List;

import edu.tufts.hrilab.action.ActionConstraints;
import edu.tufts.hrilab.action.ParameterizedAction;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.state.StateMachine;

public class RandomActionSelector extends ActionSelector {

  @Override
  public ParameterizedAction selectActionForGoal(Goal goal, ActionConstraints constraints, StateMachine stateMachine) {
    List<ParameterizedAction> candidateActions = getCandidateActions(goal, constraints, stateMachine);
    int index = ThreadLocalRandom.current().nextInt(0, candidateActions.size());
    return candidateActions.get(index);

  }
}

