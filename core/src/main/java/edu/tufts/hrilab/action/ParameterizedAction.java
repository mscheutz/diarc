/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author dave et al.
 */

package edu.tufts.hrilab.action;

import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.util.Utilities;
import edu.tufts.hrilab.fol.Factory;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.stream.Collectors;

import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.action.db.ActionDBEntry;

import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.AndJustification;
import edu.tufts.hrilab.action.justification.Justification;

public class ParameterizedAction {

  private static final Logger log = LoggerFactory.getLogger(ParameterizedAction.class);

  private ActionDBEntry entry;

  /**
   * TODO: consider changing this to Map<Variable,Object>
   */
  private Map<String, Object> actionParameters;

  private boolean persistent;
  private Observable observable;
  private AndJustification justification;
  private Symbol actor;

  /**
   * Create ParameterizedAction using a selected ActionDBEntry, Goal (which is used to
   * populate action arguments), and actor. It's possible that actor is redundant information
   * also contained in Goal, but in some cases the actor can not be extracted from the Goal,
   * so it is always required.
   *
   * @param entry
   * @param goal
   */
  public ParameterizedAction(ActionDBEntry entry, Goal goal) {
    observable = goal.getObservableEnum();
    persistent = goal.isPersistent();
    actor = goal.getActor();

    if (observable.shouldObserve()) {
      actionParameters = collectArgsForObservation(entry, goal.getPredicate());
    } else {
      actionParameters = collectArgsForGoalState(entry, goal.getPredicate());
    }

    // try to set actionParameters actor field
    // NOTE: actor consistency check happens in hasValidArguments
    if (!actionParameters.containsKey("?actor")) {
      log.debug("[ParameterizedAction] no actor role defined in the goal predicate: " + goal + ". Setting to: " + actor);
      actionParameters.put("?actor", actor);
    }

    String netBenefit = Double.toString(entry.getBenefit() - entry.getCost());
    Predicate utilityPredicate = Factory.createPredicate("utility", entry.getName(), netBenefit);
    addJustification(new ConditionJustification(true, utilityPredicate));

    this.entry = entry;
  }

  public ParameterizedAction(ActionDBEntry entry, Map<String, Object> arguments) {
    this.entry = entry;
    this.actionParameters = arguments;
    if (arguments.containsKey("?actor")) {
      this.actor = (Symbol) arguments.get("?actor");
    } else {
      log.error("No ?actor contained in arguments.");
    }
    this.observable = Observable.FALSE;
    this.persistent = false;
    this.justification = new AndJustification();
  }

  /**
   * Checks that the actionParameters can be converted to the DBEntry's role's javaTypes. Also checks
   * that the actor in the goal matches the actor extracted from the goal/observation predicate.
   *
   * @return
   */
  public boolean hasValidArguments() {
    for (Map.Entry<String, Object> param : actionParameters.entrySet()) {
      String roleName = param.getKey();
      Object roleValue = param.getValue();
      ActionBinding role = entry.getRole(roleName);
      if (role == null || !Utilities.isAssignable(roleValue.getClass(), role.getJavaType())) {
        return false;
      }
    }

    // this check is needed to filter out cases where an action's bindings from a goal predicate
    // are inconsistent with the actor executing the goal.
    // For example: if "shafer" tries to execute the goal "see(dempster,obstacle)" which happens
    // in the case of "shafer, does dempster see an obstacle"
    if (!actionParameters.get("?actor").equals(actor)) {
      // make sure actionParameters and actor (from goal) have same actor value
      log.debug("[ParameterizedAction] actionParameters and actor (from goal) have mismatched values: "
              + actionParameters.get("?actor") + " " + actor);
      return false;
    }

    return true;
  }

  public ActionDBEntry getEntry() {
    return entry;
  }

  public Map<String, Object> getBindings() {
    return new HashMap<>(actionParameters);
  }

  public boolean isPersistent() {
    return persistent;
  }

  public boolean isObservation() {
    return observable.shouldObserve();
  }

  public AndJustification getJustification() {
    return justification;
  }

  public void setJustification(AndJustification justification) {
    this.justification = justification;
  }

  public void setJustification(Justification justification) {
    this.justification = new AndJustification(justification);
  }

  public void addJustification(Justification justification) {
    if (this.justification == null) {
      this.justification = new AndJustification(justification);
    } else {
      this.justification.addJustification(justification);
    }
  }

  private Map<String, Object> collectArgsForObservation(ActionDBEntry action, Predicate observationGoal) {
    // if observationGoal is negated (i.e., of form "not(something(...))") temporarily strip the "not" off
    Predicate nonNegatedObsGoal;
    if (observationGoal.getName().equalsIgnoreCase("not")) {
      if (!observationGoal.get(0).isPredicate()) {
        log.error("[collectArgsForObservation] observationGoal has incorrect format: " + observationGoal);
      }
      nonNegatedObsGoal = (Predicate) observationGoal.get(0);
    } else {
      nonNegatedObsGoal = observationGoal;
    }

    // what state do we expect after the observation action is performed
    List<Predicate> observedStateList = action.getObservations().stream().filter(nonNegatedObsGoal::instanceOf).collect(Collectors.toList());

    Predicate observedState;
    if (observedStateList.isEmpty()) {
      log.error("Could not find matching observation for " + nonNegatedObsGoal + " in " + action.getObservations());
      return null;
    } else if (observedStateList.size() > 1) {
      log.error("Found more than one matching observation for " + nonNegatedObsGoal + " in " + action.getObservations());
      return null;
    } else {
      observedState = observedStateList.get(0);
    }

    // convert bindings to types: string for variable name, object for value
    Map<String, Object> arguments = new HashMap<>();
    Map<Variable, Symbol> bindings = observedState.getBindings(nonNegatedObsGoal);
    for (Variable v : bindings.keySet()) {
      arguments.put(v.getName(), bindings.get(v));
    }

    // include required roles
    List<ActionBinding> roles = action.getRequiredInputRoles();
    if (roles.size() == 1 && Term.class.isAssignableFrom(roles.get(0).getJavaType())) {
      arguments.put(roles.get(0).getName(), nonNegatedObsGoal);
    } else {
      log.error("The observer action '" + action.getName() + "' should only take a single Term as argument/role instead of " + roles);
      return null;
    }
    return arguments;
  }

  /**
   * Attempt to match the goal predicate with the action-signature or a post-condition. If a matching
   * predicate form is found, the arguments can be extracted from the goal predicate by matching the goal predicate args
   * to the free variables in the matching action-signature or post-condition.
   *
   * @param action
   * @param goal   goal state or action specification in predicate form
   * @return
   */
  private Map<String, Object> collectArgsForGoalState(ActionDBEntry action, Predicate goal) {
    Map<String, Object> arguments = new HashMap<>();
    if (action.getRoles().size() > 1) {
      // if action has more than just ?actor role, try to get bindings for roles (can include local roles)
      // by matching goal predicate against action signature or post-condition
      Predicate matchingPredicate;

    // first check if goal is actually just an action
    Optional<Predicate> matchingSignatures = action.getSignatureOptions(true).stream().filter(goal::instanceOf).findFirst();
    if (matchingSignatures.isPresent()) {
      matchingPredicate = matchingSignatures.get();
    } else {
      // else goal must be a post-condition. find matching post-condition on action
      List<Effect> matchingPostconditions = action.getPostConditions().stream().filter(e -> goal.instanceOf(e.getPredicate())).collect(Collectors.toList());
      if (matchingPostconditions.isEmpty()) {
        log.error("Could not find matching post condition for " + goal + " in " + action.getPostConditions());
        return null;
      } else if (matchingPostconditions.size() > 1) {
        log.error("Found more than one matching post condition for " + goal + " in " + action.getPostConditions());
        return null;
      } else {
        matchingPredicate = matchingPostconditions.get(0).getPredicate();
      }
    }

    // convert bindings to types used within action (string for variable name, object for value)
    Map<Variable, Symbol> bindings = matchingPredicate.getBindings(goal);
    for (Variable v : bindings.keySet()) {
      arguments.put(v.getName(), bindings.get(v));
    }

    // include required roles
    for (ActionBinding role : action.getRequiredInputRoles()) {
      String name = role.getName();
      if (!arguments.containsKey(name)) {
        log.debug("Role " + name + " is not bound for action " + action.getName());
        arguments.put(name, name);
      }
    }
    }

    return arguments;
  }

  /**
   * Get preconditions of an action. Bound when possible, otherwise the
   * symbols are converted to (prolog) free variables.
   *
   * @return effects
   */
  public List<Predicate> getPreConditions() {
    List<Predicate> boundConditions = new ArrayList<>();
    List<Predicate> unboundConditions = new ArrayList<>();
    for (Condition pc : entry.getPreConditions()) {
      for (Predicate p : pc.getPredicates().keySet()) {
        unboundConditions.add(p);
      }
    }
    for (Predicate p : unboundConditions) {
      boundConditions.add(bindPredicate(p));
    }
    log.debug("bound preconditions are: "+boundConditions);
    return boundConditions;
  }

  /**
   * Get overall conditions of a action. Bound when possible, otherwise the
   * symbols are converted to (prolog) free variables.
   *
   * @return effects
   */
  public List<Predicate> getOverallConditions() {
    List<Predicate> boundConditions = new ArrayList<>();
    List<Predicate> unboundConditions = new ArrayList<>();
    for (Condition pc : entry.getOverallConditions()) {
      for (Predicate p : pc.getPredicates().keySet()) {
        unboundConditions.add(p);
      }
    }
    for (Predicate p : unboundConditions) {
      boundConditions.add(bindPredicate(p));
    }
    log.debug("bound overall conditions are: "+boundConditions);
    return boundConditions;
  }

  /**
   * Get effects of a action. Effects are bound when possible, otherwise the
   * symbols are converted to (prolog) free variables.
   *
   * @return effects
   */
  public List<Predicate> getEffects() {
    List<Effect> unboundEffects = new ArrayList<>(entry.getEffects());
    List<Predicate> boundEffects = new ArrayList<>();

    for (Effect e : unboundEffects) {
      Predicate p = e.getPredicate();
      boundEffects.add(bindPredicate(p));
    }

    return boundEffects;
  }

  /**
   * Helper method to bind unbound predicates to ParameterizedAction's actionParameters.
   *
   * @param p
   * @return
   */
  public Predicate bindPredicate(Term p) {
    List<Symbol> boundArgs = new ArrayList<>();
    for (Symbol s : p.getArgs()) {
      if (Utilities.isScriptVariable(s)) {  // is a variable
        if (Utilities.isLocalVariable(s.toString())) {
          // local variable (i.e., starts with !)
          List<ActionBinding> arg = entry.getRoles().stream().filter(r -> r.name.equals(s.getName())).collect(Collectors.toList());
          if (arg.size() != 1) {
            log.error("No role defined for local variable: " + s);
            boundArgs.add(s);
          } else if (arg.get(0).isBound()) {
            boundArgs.add(Factory.createSymbol(arg.get(0).getBindingDeep().toString()));
          } else if (arg.get(0).hasDefaultValue()) {
            boundArgs.add(Factory.createSymbol(arg.get(0).defaultValue.toString()));
          } else {
            boundArgs.add(s);
          }
        } else {
          // input variable (i.e., starts with ?)
        if (actionParameters.containsKey(s.getName())) {
          Object value = actionParameters.get(s.getName());
          if (value == null) {
            boundArgs.add(Factory.createSymbol("null"));
          } else if (value.toString().equals(s.getName())) { // Variable is not bound
            // replace symbol (variable) with a free variable (e.g. "?arg" turns into "arg")
            boundArgs.add(Factory.createSymbol(s.getName().substring(1).toUpperCase()));
          } else { // Variable is bound to a value
            // replace symbol with the corresponding value
            boundArgs.add(Factory.createFOL(value.toString()));
          }
        } else if (entry.getRoles().stream().anyMatch(r -> r.name.equals(s.getName()))) {
            List<ActionBinding> arg = entry.getRoles().stream().filter(r -> r.name.equals(s.getName())).collect(Collectors.toList());
            if (arg.size() != 1) {
              log.error("No role defined for variable: " + s);
              boundArgs.add(s);
            } else if (arg.get(0).hasDefaultValue()) {
              boundArgs.add(Factory.createSymbol(arg.get(0).defaultValue.toString()));
            } else {
              log.error("Required arg has no passed in value or default value: " + s);
              boundArgs.add(s);
            }
        } else {
          log.error("Could not find symbol in requiredArgs. This should not happen!");
          boundArgs.add(Factory.createSymbol(s.getName().substring(1).toUpperCase()));
          }
        }
      } else if (s.hasArgs()) {
        boundArgs.add(bindPredicate((Term)s));
      } else {
        // not a variable, just keep it.
        boundArgs.add(s);
      }
    }
    return Factory.createPredicate(p.getName(), boundArgs);
  }

  /**
   * Get the top-level steps (i.e., the EventSpecs of the selected actionDBEntry)
   * of this action selection in predicate form. This is currently used by NLG to generate
   * action script narrations (via a getActDesc action script).
   * <p>
   * It's possible that this method shouldn't live here, but should
   * perhaps be moved to a new class for script narration. This is an early-stage
   * implementation of this idea, so it's unclear how this should be architected.
   *
   * @return
   */
  public Predicate getStepsInPredicateForm() {
    List<Predicate> predicateSteps = new ArrayList<>();
    Map<String, String> localRoles = new HashMap();
    for(ActionBinding localRole : entry.getLocalRoles()) {
      localRoles.put(localRole.getName(), localRole.getBinding().toString());
    }

    // iterate through actionDBEntry's eventSpecs
    for (EventSpec eventSpec : entry.getEventSpecs()) {
      List<String> eventSpecArgs = new ArrayList<>();
      switch (eventSpec.getType()) {
        case ACTION:
          // get actor binding
          String actor = eventSpec.getActor();
          if (Utilities.isScriptVariable(actor)) {
            // if eventSpec has null actor, assume it's the same as the top-level action
            actor = actionParameters.get(actor).toString();
          }
          eventSpecArgs.add(actor);
        case GOAL:
          // get non-actor arg bindings
          for (String arg : eventSpec.getInputArgs()) {
            if (Utilities.isScriptVariable(arg)) { //check local or input
              if (actionParameters.containsKey(arg)) {
                eventSpecArgs.add(actionParameters.get(arg).toString());
              }else if(localRoles.containsKey(arg)){
                eventSpecArgs.add(localRoles.get(arg));
              }else {
                eventSpecArgs.add("null");
              }
            } else {
              eventSpecArgs.add(arg);
            }
          }

          Predicate eventPred = Factory.createPredicate(eventSpec.getCommand(), eventSpecArgs.toArray(new String[0]));
          predicateSteps.add(eventPred);
        default:
          log.warn("[getStepsInPredicateForm] only returning ACTSPEC steps.");
          break;
      }
    }

    return Factory.createPredicate("steps", predicateSteps);
  }

  public Predicate getBoundActionSignature() {
    return bindPredicate(this.getEntry().getSignature(true));
  }

  @Override
  public String toString() {
    return getBoundActionSignature().toString();
  }

}
