/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.recovery;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.Condition;
import edu.tufts.hrilab.action.ConditionType;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.Observable;
import edu.tufts.hrilab.action.ParameterizedAction;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.recovery.Utilities;
import edu.tufts.hrilab.belief.common.MemoryLevel;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.polycraft.recovery.util.ExploreUtils;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

public class FailedReturnValuePolicy {
  private static Logger log = LoggerFactory.getLogger(FailedReturnValuePolicy.class);

  private boolean failedReturnValueExploration = true; //make false if we want to test no exploration in failed return value

  /**
   * Register this class with TRADE.
   */
  public void registerWithTrade(List<String> groups) {
    try {
      TRADE.registerAllServices(this,groups);
    } catch (TRADEException e) {
      log.error("Error trying to register with TRADE.", e);
    }
  }

  /**
   * Explore an action that has failed with FAIL_RETURNVALUE. This failure case means that a primitive action has failed,
   * often with no failure semantics.
   *
   * @param actor
   * @param brokenActionSignature
   * @param failureReasons
   * @param goal
   * @return
   */
  @TRADEService
  @Action
  public Justification executeFailedReturnValuePolicy(Symbol actor, Predicate brokenActionSignature, List<Predicate> failureReasons, Predicate goal) throws TRADEException {
    // so we don't try to repair broken actions if we're exploring actions we expect to fail
      if ((boolean )TRADE.getAvailableService(new TRADEServiceConstraints().name("isCurrentlyExploring")).call(Object.class)) {
        return new ConditionJustification(true);
      }

      // TODO: implement this somehow!
    // so we don't enter an endless loop
//    if (currentlyExploringBrokenAction.get() > 1) {
//      return;
//    }

    if (brokenActionSignature == null) {
      return new ConditionJustification(false);
    }
    ActionDBEntry brokenAction = Utilities.getActionBySignature(brokenActionSignature);
    if (brokenAction == null) {
      return new ConditionJustification(false);
    }

    // This means that a pre-condition is either missing or wrong

    // if craft action failed, assume it's a fake recipe, and disable it
    if (brokenAction.getName().startsWith("craft") || brokenAction.getName().startsWith("sensed_craft")) {
      log.debug("Disabling failed craft action: " + brokenAction);
      Database.getInstance().disableActionDBEntry(brokenAction);
      // TODO: report novelty here
      return new ConditionJustification(true);
    } else if (brokenActionSignature.getName().startsWith("trade") || brokenActionSignature.getName().equals("interact_with") || brokenActionSignature.getName().equals("get_trades_from")) {
      //trade recipe is wrong or trader is busy
      return new ConditionJustification(true);
    } else if (brokenActionSignature.getName().startsWith("move")) {
      // block/actor in path
      // belief state for at(target_loc) must be wrong. ignore this for now and just try to replan
      return new ConditionJustification(true);
    } else if (brokenActionSignature.size() == 1) {
      // this exploration only handles actions with a single non-?actor arg (i.e., not approach_object, find_safe, etc)

      // TODO: create list of pre-conditions to try, and iterate through those pre-conditions until the action works

      // TODO: report novelty here
      if (brokenActionSignature.getName().startsWith("collect_from")) {
        try {
          String failureMessage = TRADE.getAvailableService(new TRADEServiceConstraints().name("getFailureMessage")).call(String.class, brokenActionSignature.getName());
          if (failureMessage != null && failureMessage.startsWith("Already collected from")) {
            //the container has already been collected from, let's try to get it sooner next time
            Symbol subject = brokenActionSignature.get(0);
            Predicate p = Factory.createPredicate("fluent_equals(container(" + subject + ",X),Y)");
            //get item we thought was in container
            List<Map<Variable, Symbol>> maps = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, p);
            for (Map<Variable, Symbol> binding : maps) {
              //assert that container is empty of item and that we want to explore it next time we can
              Predicate newp = Factory.createPredicate("fluent_equals(container(" + subject + "," + binding.get(new Variable("X")) + "),0)");
              Predicate oldp = Factory.createPredicate("fluent_equals(container(" + subject + "," + binding.get(new Variable("X")) + "),"+binding.get(new Variable("Y"))+")");
              TRADE.getAvailableService(new TRADEServiceConstraints().name("retractBelief")).call(Object.class, oldp, MemoryLevel.EPISODIC);
              TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBelief")).call(Object.class, newp, MemoryLevel.EPISODIC);
            }
            TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBelief")).call(Object.class, Factory.createPredicate("toExplore", brokenActionSignature, brokenActionSignature), MemoryLevel.UNIVERSAL);
            return new ConditionJustification(true); // done;
          }
        } catch (TRADEException e) {
          log.error("Failed to get failure message: ", e);
        }
      }

      // first use any novelties (even those that have been explored) as new preconditions
      // currently, just using object novelties
      // TODO: make sure this doesn't cause an infinite recursive recovery
      Set<Predicate> objectNovelties = ExploreUtils.getDetectedNovelties(Factory.createSymbol("1"));
      for (Predicate objectNovelty : objectNovelties) {
        Symbol novelObject = objectNovelty.get(0);
        Symbol inputObj = brokenActionSignature.get(0);
        if (novelObject.equals(inputObj)) {
          // not possible to hold object if that's the same as the object we're trying to perform the action on
          // e.g., try to first hold fence before breaking the fence
          continue;
        }

        // check if broken action already has holding(self, X) pre-condition
        Predicate holdingConditionPred = Factory.createPredicate("holding", actor, Factory.createVariable("H"));
        ParameterizedAction parameterizedAction = new ParameterizedAction(brokenAction, new Goal(actor, brokenActionSignature));
        if (ExploreUtils.getExistingCondition(parameterizedAction, holdingConditionPred) != null) {
          // broken action already has holding(self, novelObject) pre-condition, don't need to try that
          continue;
        }

        Predicate holdingPred = Factory.createPredicate("holding", actor, novelObject);
        Predicate goalPred = Factory.createPredicate("goal", actor, holdingPred);
        Justification selectJustification = ExploreUtils.doGoal(goalPred);
        if (selectJustification.getValue()) {
          Justification breakJustification = ExploreUtils.doGoal(brokenActionSignature);
          if (breakJustification.getValue()) {
            // get broken action input argument value
            Variable inputVar = Factory.createVariable(brokenAction.getInputRoles().get(1).name);
            Predicate typeConstraint = Factory.createPredicate("equals", inputVar, inputObj);

            // make new action and add holding(novelObj) and object(type) pre-conditions
            ActionDBEntry.Builder newActionBuilder = new ActionDBEntry.Builder(brokenAction);
            newActionBuilder.setType(brokenActionSignature + "_holding_" + novelObject.getName());
            newActionBuilder.addCondition(new Condition(holdingPred, ConditionType.PRE, Observable.TRUE));
            newActionBuilder.addCondition(new Condition(typeConstraint, ConditionType.PRE, Observable.FALSE));
            newActionBuilder.build(true); // add modified action to DB

            // update broken action to have not(object(type)) pre-condition
            ActionDBEntry.Builder brokenActionBuilder = new ActionDBEntry.Builder(brokenAction);
            Database.getInstance().disableActionDBEntry(brokenAction); // disable broken action
            brokenActionBuilder.addCondition(new Condition(typeConstraint.toNegatedForm(), ConditionType.PRE, Observable.FALSE));
            brokenActionBuilder.build(true); // add modified action to DB

            return new ConditionJustification(true); // done
          }
        }
      }

      if (!failedReturnValueExploration) {
        log.info("FAIL_RETURNVALUE explorations disabled.");
        return new ConditionJustification(false);
      }

      Symbol subject = brokenActionSignature.get(0);
      //iterate through actions
      List<Predicate> actions = new ArrayList<>();

      try {
        if (TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, Factory.createPredicate("type", subject, Factory.createSymbol("breakable")))) {
          actions.add(new Predicate("break_and_pickup", actor, subject));
        }
        if (TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, Factory.createPredicate("type", subject, Factory.createSymbol("placeable")))) {
          actions.add(new Predicate("place", actor, subject));
        }
        if (TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, Factory.createPredicate("type", subject, Factory.createSymbol("physobj")))) {
          actions.add(new Predicate("collect_from", actor, subject));
          actions.add(new Predicate("select", actor, subject));
          actions.add(new Predicate("use",actor, subject));
        }
      } catch (Exception e) {
        log.error("Error in calling belief:", e);
      }

      for (Predicate action : actions) {
        if (!action.equals(brokenActionSignature)) {
          Justification interactJustification = ExploreUtils.doGoal(action);
          if (interactJustification.getValue()) {
            log.debug("Action worked.");
          }
        }
      }

      // replace subject with free variable
      Variable subjectVar = Factory.createVariable("Y");
      List<Symbol> genericArgs = brokenActionSignature.getArgs().stream().map(arg ->
              (arg.equals(subject) ? subjectVar : arg)).collect(Collectors.toList());
      Predicate genericPredicate = Factory.createPredicate(brokenActionSignature.getName(), genericArgs);

      List<Symbol> attempted = new ArrayList<>();
      attempted.add(subject);
      Map<Variable, Symbol> tmap = typeExploration(subject, genericPredicate, attempted);
      //change action to match new information

      if (!tmap.isEmpty()) {
        Symbol inputObj = tmap.get(Factory.createVariable("Y"));
        Variable inputVar = Factory.createVariable(brokenAction.getInputRoles().get(1).name);

        try {
          Set<Term> novelties = new HashSet<>();
          novelties.add(Factory.createPredicate("failed", "return"));
          TRADE.getAvailableService(new TRADEServiceConstraints().name("reportNovelties")).call(Object.class, novelties);
        } catch (TRADEException e) {
          log.error("Exception reporting novelties: ",e);
        }

        Predicate objectTypePred = Factory.createPredicate("equals",inputVar,subject);
        //make new action
        ActionDBEntry.Builder fixedActionBuilder = new ActionDBEntry.Builder(brokenAction);
        //fixedActionBuilder.setType(brokenActionSignature + "_type_" + inputObj.getName());
        fixedActionBuilder.addCondition(new Condition(objectTypePred.toNegatedForm(), ConditionType.PRE, Observable.TRUE));

        Database.getInstance().removeActionDBEntry(brokenAction);
        fixedActionBuilder.build(true);  // add modified action to DB

        return new ConditionJustification(true);
      }

      //try original one more time (will pick up infrequent trader interaction novelty)
      ExploreUtils.doGoal(brokenActionSignature);
    }
    return new ConditionJustification(false);
  }

  public Map<Variable, Symbol> typeExploration(Symbol subject, Predicate genericPredicate, List<Symbol> attempted) {
    Predicate query = Factory.createPredicate("subtype(" + subject + ",X)");

    try {
      List<Map<Variable, Symbol>> results = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, query);

      query = Factory.createPredicate("type(Y,X)");

      //try goal with different arguments of same type
      for (Map<Variable, Symbol> map : results) {
        Predicate testtype = query.copyWithNewBindings(map);
        List<Map<Variable, Symbol>> resultsoftype = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, testtype);

        for (Map<Variable, Symbol> tmap : resultsoftype) {
          Symbol inputObj = tmap.get(Factory.createVariable("Y"));
          Predicate objquery = Factory.createPredicate("typeobject", inputObj, Factory.createVariable("T"));
          if (TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, objquery)) {
            if (!attempted.contains(inputObj)) {
              attempted.add(inputObj);
              Predicate exploreGoal = genericPredicate.copyWithNewBindings(tmap);
              try {
                Justification interactJustification = ExploreUtils.doGoal(exploreGoal);
                if (interactJustification.getValue()) {
                  return tmap;
                }
              } catch (Exception e) {
                log.debug(exploreGoal + " failed with error: " + e);
              }
            }
          }
        }
      }
      for (Map<Variable, Symbol> map : results) {
        // trying one abstraction level higher -- stopping at physical
        Symbol subtype = map.get(Factory.createVariable("X"));
        if (subtype.toString().equals("physical")) {
          return new HashMap<>();
        }
        return typeExploration(subtype, genericPredicate, attempted);
      }
    } catch (TRADEException e) {
      log.error("Error calling belief to explore broken action.", e);
    }
    return new HashMap<>();
  }
}
