/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author dave
 */

package edu.tufts.hrilab.action.selector;

import java.util.List;
import java.util.stream.Collectors;

import edu.tufts.hrilab.action.ActionConstraints;
import edu.tufts.hrilab.action.ParameterizedAction;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.state.StateMachine;


public class UtilitarianActionSelector extends ActionSelector {
  @Override
  public ParameterizedAction selectActionForGoal(Goal goal, ActionConstraints constraints, StateMachine stateMachine) {

    List<ParameterizedAction> possibleActions = getCandidateActions(goal, constraints, stateMachine);

    List<ParameterizedAction> sorted = possibleActions.stream()
            .sorted(UtilitarianActionSelector::compareUtility)     // sort by utility
            .collect(Collectors.toList());

    ParameterizedAction best = sorted.stream()
            .filter((s) -> (s.getJustification() == null) || s.getJustification().getValue())      // keep actions that do not violate constraints
            .findFirst().orElse(null);                                                             // select best, or null if no actions to choose from.

    if (best != null) {
      return best;                                      // return best action with support
    } else {
      return sorted.stream().findFirst().orElse(null);  // return best action without support
    }
  }

  /**
   * Compares the utility of two ParameterizedActions
   *
   * @param a ParameterizedAction
   * @param b ParameterizedAction
   * @return -1 if utility of a is greater than b, 0 if equal utility, 1 otherwise.
   */
  private static int compareUtility(ParameterizedAction a, ParameterizedAction b) {
    double delta = (b.getEntry().getBenefit() - b.getEntry().getCost()) -
            (a.getEntry().getBenefit() - a.getEntry().getCost());

    if (delta > 0)
      return -1;
    else if (delta < 0)
      return 1;
    else
      return 0;
  }


}
