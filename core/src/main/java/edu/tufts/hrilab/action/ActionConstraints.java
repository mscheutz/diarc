/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action;

import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.justification.OrJustification;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.fol.Factory;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.stream.Collectors;

/**
 *
 * @author willie
 */
public class ActionConstraints {
  protected final static Logger log = LoggerFactory.getLogger(ActionConstraints.class);
  private final List<String> allowedActors = new ArrayList<>();
  private final List<String> forbiddenActors = new ArrayList<>();
  private final List<String> forbiddenActions = new ArrayList<>();
  private final List<String> allowedActions = new ArrayList<>();
  private final List<Predicate> forbiddenStates = new ArrayList<>();
  
  public void addForbiddenAction(String type) {
    forbiddenActions.add(type);
  }

  public void addAllowedAction(String type) {
    allowedActions.add(type);
  }

  public void addForbiddenState(Predicate p) {
    forbiddenStates.add(p);
  }

  public Set<ActionDBEntry> filterByConstraints(Set<ActionDBEntry> entries) {
    if(!allowedActions.isEmpty()) {
      entries = entries.stream().filter(e -> allowedActions.contains(e.getName())).collect(Collectors.toSet());
    }

    if(!forbiddenActions.isEmpty()) {
      entries = entries.stream().filter(e -> !forbiddenActions.contains(e.getName())).collect(Collectors.toSet());
    }

    return entries;
  }

  /**
   * Verifies whether a goal does not violate the constraints.
   * @param goal goal predicate
   * @param sm state machine
   * @return True if constraints are met
   */
  public Justification verifyConstraints(Predicate goal, StateMachine sm) {
    return verifyConstraints(goal.toString(), Collections.singletonList(goal), sm);
  }

  /**
   * Verifies whether an action selection meets all the the constraints.
   * @param action action selection to be checked
   * @param sm state machine
   * @return True if constraints are met
   */
  public Justification verifyConstraints(ParameterizedAction action, StateMachine sm) {
    return verifyConstraints(action.getEntry().getType(), action.getEffects(), sm);
  }

  /**
   * Verifies whether an action meets all the the constraints.
   * @param action action
   * @param effects effects of action
   * @param sm state machine
   * @return True if constraints are met
   */
  public Justification verifyConstraints(ActionDBEntry action, List<Effect> effects, StateMachine sm) {
    List<Predicate> effectPredicates = new ArrayList<>();
    effects.forEach(e -> effectPredicates.add(e.getPredicate()));
    return verifyConstraints(action.getType(), effectPredicates, sm);
  }

  /**
   * Verifies whether an action meets all the the constraints.
   * @param actionName action name
   * @param effects effects of action
   * @param sm state machine
   * @return True if constraints are met
   */
  private Justification verifyConstraints(String actionName, List<Predicate> effects, StateMachine sm) {

    if(!allowedActions.isEmpty() && !allowedActions.contains(actionName)) {
      log.debug("Action " + actionName + " is forbidden!");
      return new ConditionJustification(false, Factory.createPredicate("forbiddenAction", actionName));
    }

    // Check if action is forbidden
    if (forbiddenActions.contains(actionName)) {
      log.debug("Action " + actionName + " is forbidden!");
      return new ConditionJustification(false,
          Factory.createPredicate("forbiddenAction", actionName));
    }

    // Check if action would result in forbidden state
    OrJustification fsCheck = sm.disjunctionHolds(forbiddenStates, effects);
    if(fsCheck.getValue()) {
      OrJustification fsJustification = new OrJustification();
      for (Justification justification : fsCheck.getJustifications()) {
        if (justification.getValue()) {
          fsJustification.addJustification(justification);
        }
      }
      log.debug("Action " + actionName + " results in a forbidden state!");
      fsJustification.addJustification(new ConditionJustification(true, Factory.createPredicate("forbiddenState", actionName)));
      fsJustification.setValue(false);
      return fsJustification;
    }
    return new ConditionJustification(true);
  }

  public List<String> getAllowedActors() {
    return allowedActors;
  }

  public void addAllowedActor(String actor) {
    allowedActors.add(actor);
  }

  public void removeAllowedActor(String actor) {
    allowedActors.remove(actor);
  }

  public List<String> getForbiddenActors() {
    return forbiddenActors;
  }
}
