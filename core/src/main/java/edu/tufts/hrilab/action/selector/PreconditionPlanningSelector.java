/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.selector;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.*;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.util.Utilities;

import java.util.List;
import java.util.stream.Collectors;

//todo: add javadocs

public class PreconditionPlanningSelector extends ActionSelector {

  private final UtilitarianActionSelector uSelector;

  PreconditionPlanningSelector() {
    uSelector = new UtilitarianActionSelector();
  }

  @Override
  public ParameterizedAction selectActionForGoal(Goal goal, ActionConstraints constraints, StateMachine stateMachine) {
    //TODO:add special case to call utilitarian action selector for handled goals
    if (goal.isObservation() || goal.isAction() || "handled".equals(goal.getPredicate().getName()) || "handleSemantics".equals(goal.getPredicate().getName()) || "knows".equals(goal.getPredicate().getName())) {
      // If goal is specifying a particular action or observation action, look up action using utilitarian action selector,
      // and then attach planned action to reach pre-conditions (if any)

      ParameterizedAction selectedAction = uSelector.selectActionForGoal(goal, constraints, stateMachine);
      if (selectedAction == null) {
        return null;
      }

      //check preconds
      List<Predicate> boundPreconditions = getPreconds(selectedAction);
      boolean preconditionsSatisfied = checkPreconditions(boundPreconditions);
      if (preconditionsSatisfied) {
        return selectedAction;
      } else {
        // call planner
        return planForPreconditions(goal, constraints, stateMachine, boundPreconditions);
      }

    }
    return null;
  }

  private List<Predicate> getPreconds(ParameterizedAction selectedAction) {
    return selectedAction.getPreConditions().stream().filter(condition -> condition.getVars().isEmpty()).collect(Collectors.toList());
  }

  private boolean checkPreconditions(List<Predicate> boundPreconditions) {
    // no pre-conditions to be met
    if (boundPreconditions.isEmpty()) {
      return true;
    }

    // check if pre-conditions are already satisfied
    return boundPreconditions.stream().allMatch(condition -> {
      try {
        return TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport").argTypes(Predicate.class)).call(Boolean.class, condition);
      } catch (TRADEException e) {
        log.error("Could not check pre-condition: " + condition,e);
        return false;
      }
    });
  }

  private ParameterizedAction planForPreconditions(Goal goal, ActionConstraints constraints, StateMachine stateMachine, List<Predicate> boundPreconditions) {
    // find plan to satisfy pre-conditions
    Predicate preconditionGoalPredicate = Utilities.createAndPredicate(boundPreconditions);
    Goal preconditionGoal = new Goal(goal.getActor(), preconditionGoalPredicate);
    ParameterizedAction preconditionAction = new GoalPlanningActionSelector().selectActionForGoal(preconditionGoal, constraints, stateMachine);

    if (preconditionAction == null) {
      // plan for pre-conditions couldn't be found
      return null;
    }

    // combine preconditionAction and selectedAction into one ParameterizedAction
    ActionDBEntry.Builder combinedADB = new ActionDBEntry.Builder("planned");
    combinedADB.prependEventSpecs(preconditionAction.getEntry().getEventSpecs()); // steps to reach pre-conditions
    combinedADB.addEventSpec(new EventSpec(goal.getPredicate())); // original action step

    return new ParameterizedAction(combinedADB.build(false), goal);
  }
}
