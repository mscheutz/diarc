/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.selector;

import edu.tufts.hrilab.action.ActionConstraints;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.ParameterizedAction;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.planner.Planner;
import edu.tufts.hrilab.action.planner.ffplanner.FFPlanner;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class GoalPlanningActionSelector extends ActionSelector {

  private static final Logger log = LoggerFactory.getLogger(GoalPlanningActionSelector.class);
  private final Planner planner;
  private final PreconditionPlanningSelector precondPlanner;

  /**
   * This should stay protected to prevent instantiation without using the ActionSelection.getInstance singleton getter.
   */
  public GoalPlanningActionSelector() {
    planner = new FFPlanner();
    precondPlanner = new PreconditionPlanningSelector();
  }

  /**
   * Main implementation of the action selector. Performs a problem escalation, starting with checking if goal
   * can be satisfied by a single pre-existing action,
   *
   * @param goal         the goal that the selected action should accomplish
   * @param constraints  a returned goal should satisfy any constraints
   * @param stateMachine in case the action selector cares about the state
   */
  @Override
  public ParameterizedAction selectActionForGoal(Goal goal, ActionConstraints constraints, StateMachine stateMachine) {
    log.debug("Selecting action for goal: " + goal);

    if (goal.isObservation() || goal.isAction() || "handled".equals(goal.getPredicate().getName()) || "handleSemantics".equals(goal.getPredicate().getName()) || "knows".equals(goal.getPredicate().getName())) {
      // If goal is specifying a particular action or observation action, look up action using utilitarian action selector,
      // and then attach planned action to reach pre-conditions (if any)

      ParameterizedAction selectedAction = new UtilitarianActionSelector().selectActionForGoal(goal, constraints, stateMachine);
      if (selectedAction == null) {
        return null;
      }
    }

    ParameterizedAction selectedAction = precondPlanner.selectActionForGoal(goal, constraints, stateMachine);
    if (selectedAction != null) {
      return selectedAction;
    } else {
      return planner.plan(goal, constraints, stateMachine);
    }
  }
}

