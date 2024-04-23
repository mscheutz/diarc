/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.selector;

import edu.tufts.hrilab.action.ActionConstraints;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.ParameterizedAction;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.planner.Planner;
import edu.tufts.hrilab.action.planner.ffplanner.FFPlanner;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;

public class TypedActionSelector extends ActionSelector {

  private static final Logger log = LoggerFactory.getLogger(TypedActionSelector.class);
  private Planner planner;

  /**
   * This should stay protected to prevent instantiation without using the ActionSelection.getInstance singleton getter.
   */
  protected TypedActionSelector() {
    planner = new FFPlanner();
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

    if (goal.isObservation()){
      return selectActionByLookUp(goal,constraints,stateMachine,Database.getObserverDB().getObservers(goal.getActor(), goal.getPredicate()));
    }
//    else if(goal.getPredicate().getName().equals("prepare")){
//      return selectPrepareAction();
//    }
    else if(goal.isAction()){
      return selectActionByLookUp(goal,constraints,stateMachine,Database.getActionDB().getActionsBySignature(goal.getPredicate()));
    } else if("handled".equals(goal.getPredicate().getName())
            || "knows".equals(goal.getPredicate().getName())){

      return selectActionByLookUp(goal,constraints,stateMachine,Database.getActionDB().getActionsByEffect(goal.getActor(), goal.getPredicate()));


    } else {
      //todo:brad: added to let you submit action signatures to goal specs in asl, so you can dynamically build goal predicates in action scripts, this can be remove once action selection is better and we don't need to do it in action scripts.
       ParameterizedAction selectedAction = selectActionByLookUp(goal,constraints,stateMachine,Database.getActionDB().getActionsBySignature(goal.getPredicate()));

       if(selectedAction != null){
         goal.setIsAction(true);
         return selectedAction;
       } else {
         // call planner
         return planner.plan(goal, constraints, stateMachine);
      }
    }
  }

  public ParameterizedAction selectActionByLookUp(Goal goal, ActionConstraints constraints, StateMachine stateMachine, List<ActionDBEntry> entries) {

    List<ParameterizedAction> possibleActions = getCandidateActions(goal, constraints, stateMachine,entries);

    ParameterizedAction best = possibleActions.stream()
            .filter((s) -> (s.getJustification() == null) || s.getJustification().getValue())      // keep actions that do not violate constraints
            .findFirst().orElse(null);                                                             // select best, or null if no actions to choose from.

    if(best != null) {
      return best;                                      // return best action with support
    } else {
      return possibleActions.stream().findFirst().orElse(null);  // return best action without support
    }
  }

  protected static List<ParameterizedAction> getCandidateActions(Goal goal, ActionConstraints constraints, StateMachine stateMachine, List<ActionDBEntry> dbLookUp) {

    List<ParameterizedAction> candidates = new ArrayList<>();

    // build the ParameterizedActions, and
    // filter them by javaType checking the actionParameters (i.e., input args)
    for (ActionDBEntry adb : dbLookUp) {
      ParameterizedAction parameterizedAction = new ParameterizedAction(adb, goal);
      if (parameterizedAction.hasValidArguments()) {
        candidates.add(parameterizedAction);
      }
    }

    // we probably want this to filter the results instead of just annotating the actions?
    // this the only reason this method takes a stateMachine and a Constraint,
    // it would be nice to put these to better use... -db
    for (ParameterizedAction action : candidates) {
      Justification justification = constraints.verifyConstraints(action, stateMachine);
      action.setJustification(justification);
    }

    return candidates;
  }
//  private ParameterizedAction selectPrepareAction(Goal g){
//    Predicate goalPred = g.getPredicate();
//    Symbol itemType;
//    Predicate options= Factory.createPredicate("none()");
//    Symbol trayRef= Factory.createSymbol("none");
//
//    //arg 0 is the actor
//    if(goalPred.getArgs().size()>=2){
//      itemType= goalPred.get(1);
//    }
//    else{
//      log.error("badly formed prepare goal: "+g+" . Could not select action.");
//      return null;
//    }
//    if(options.getArgs().size()>=3){
//      itemType= goalPred.get(2);
//    }
//    if(goalPred.getArgs().size()==4){
//      trayRef= goalPred.get(3);
//    }
//
//
//  }

}

