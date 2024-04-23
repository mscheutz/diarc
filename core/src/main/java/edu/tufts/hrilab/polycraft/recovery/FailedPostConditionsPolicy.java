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
import edu.tufts.hrilab.action.Effect;
import edu.tufts.hrilab.action.EffectType;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.Observable;
import edu.tufts.hrilab.action.ParameterizedAction;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.justification.AndJustification;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.util.Utilities;
import edu.tufts.hrilab.belief.common.MemoryLevel;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.polycraft.recovery.util.ExploreUtils;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.stream.Collectors;

public class FailedPostConditionsPolicy {
  private static Logger log = LoggerFactory.getLogger(FailedPostConditionsPolicy.class);

  /**
   * Register this class with TRADE.
   */
  public void registerWithTrade(List<String> groups) {
    try {
      //TODO:brad: will this ever need ot be in a group?
      TRADE.registerAllServices(this,groups);
    } catch (TRADEException e) {
      log.error("Error trying to register with TRADE.", e);
    }
  }

  /**
   * Explore an action that has failed with FAIL_POSTCONDITONS.
   *
   * @param actor
   * @param brokenActionSignature
   * @param failureReasons
   * @param goal
   * @return
   */
  @TRADEService
  @Action
  public Justification executeFailedPostConditionsPolicy(Symbol actor, Predicate brokenActionSignature, List<Predicate> failureReasons, Predicate goal) {
    if (brokenActionSignature == null) {
      return new ConditionJustification(false);
    }

    ActionDBEntry brokenAction = edu.tufts.hrilab.action.recovery.Utilities.getActionBySignature(brokenActionSignature);

    // failed post-condition (or discrepancy)
    // CASES:
    // (1) discrepancy(x,y) --> use action model to either (a) do nothing or (b) add new post-condition to action
    // (2) non-discrepancy failed post-condition has local vars (start with !) --> update belief, not action
    // (3) failed post-condition with no local vars (fully bound or only input vars) --> update existing post-condition

    // create action builder that will get populated with fixed post-conditions
    AndJustification justification = new AndJustification();
    ActionDBEntry.Builder fixedActionBuilder = new ActionDBEntry.Builder(brokenAction);
    ParameterizedAction parameterizedBrokenAction = new ParameterizedAction(brokenAction, new Goal(actor, brokenActionSignature));

    Set<Predicate> novelties = new HashSet<>();

    // determine if new action should be created, or if existing action should be modified
    boolean createNewAction;
    if (brokenActionSignature.size() > 0) {
      // if broken action is already a special case for a specific type of input object (i.e., equals(?inputArg, type) pre-cond),
      // modify the existing action instead of creating a new special case action
      Symbol inputArg = brokenActionSignature.get(0);
      Predicate typeConstraint = Factory.createPredicate("equals", inputArg, inputArg);
      if (ExploreUtils.getExistingCondition(parameterizedBrokenAction, typeConstraint) != null) {
        // action is only for inputArg type --> don't create new action, just modify existing action
        createNewAction = false;
      } else {
        // create new action --> change new action name to not conflict with existing action
        fixedActionBuilder.setType(brokenAction.getType() + "_" + inputArg);
        createNewAction = true;
      }
    } else {
      // broken action has no arguments (besides ?actor role), assuming we want to modify the existing action in this case
      createNewAction = false;
    }

    Term generateConstantExploration = null;

    for (Predicate failure : failureReasons) {
      // get broken effect from broken action
      Effect brokenEffect = ExploreUtils.getExistingEffect(parameterizedBrokenAction, failure);
      if (brokenEffect == null) {
        log.warn("Could not find broken Effect.");
        justification.addJustification(new ConditionJustification(false, Factory.createPredicate("fixed", failure)));
        continue;
      }

      if (failure.toUnnegatedForm().getName().equals("discrepancy")) {
        // CASE 1 -- actually of the form not(discrepancy(x,y))
        String brokenActionName = brokenAction.getName();
        Term discrepancy = failure.toUnnegatedForm();
        Term beliefState = (Term) discrepancy.get(0);
        Term worldState = (Term) discrepancy.get(1);

        //new constants or subtypes have appeared
        if (worldState.getName().equals("constant") || worldState.getName().equals("subtype")) {
          novelties.add((Predicate) failure.toUnnegatedForm());

          try {
            Set<Predicate> initPreds = new HashSet<>();
            Symbol s = worldState.get(0);
            //add to universal beliefs
            TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBelief")).call(Object.class, worldState, MemoryLevel.UNIVERSAL);

            if (worldState.getName().equals("constant")) {
              generateConstantExploration = worldState;
            } else {

              //add new fluents for the object to keep track of quantities of it
              // only want objects that are physobj type

              List<Map<Variable, Symbol>> fluentbindings = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, Factory.createPredicate("visiblefluent(X,Y,Z)"));
              for (Map<Variable, Symbol> fluent : fluentbindings) {
                Predicate typequery = Factory.createPredicate("type", s, fluent.get(new Variable("Z")));
                if (TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, typequery)) {
                  //fluent is visible
                  // if condition to check whether the predicated below exists or not. X
                  Predicate fluentquery;
                  if (fluent.get("Y").equals("none")) {
                    fluentquery = Factory.createPredicate("fluent_equals(" + fluent.get(new Variable("X")) + "(" + s + "),X)");
                    if (!TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, fluentquery, MemoryLevel.EPISODIC)) {
                      initPreds.add(Factory.createPredicate("fluent_equals(" + fluent.get(new Variable("X")) + "(" + s + "),0)"));
                    }
                  } else {
                    fluentquery = Factory.createPredicate("fluent_equals(" + fluent.get(new Variable("X")) + "(" + fluent.get("Y") + "," + s + "),X)");
                    if (!TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, fluentquery, MemoryLevel.EPISODIC)) {
                      initPreds.add(Factory.createPredicate("fluent_equals(" + fluent.get(new Variable("X")) + "(" + fluent.get("Y") + "," + s + "),0)"));
                    }
                  }
                }
              }


              TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBeliefs")).call(Object.class, initPreds, MemoryLevel.EPISODIC);

            }
          } catch (TRADEException e) {
            log.error("Cannot assert init predicates: ", e);
          }
        } else if (beliefState.getName().equals("fluent_equals")) {
          try {
            List<Map<Variable, Symbol>> navigationActions = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, Factory.createPredicate("navigation(X)"));
            for (Map<Variable, Symbol> binding : navigationActions) {
              Symbol action = binding.get(new Variable("X"));
              if (brokenActionName.startsWith(action.getName())) {
                // allow fluent_increase discrepancy in the case of the agent moving (accidentally picks something up)
                continue;
              }
            }
          } catch (TRADEException e) {
            log.error("Error calling queryBelief", e);
          }
        } else if (isChangeable(failure, parameterizedBrokenAction)) {
          // TODO: how to handle this in general?
          // log changeability in belief instead of hardcoding it here
          // for now, treat all sapling inventory increases by 1 as "normal" bc any action that happens near where a oak_log
          // once was can cause this discrepancy (i.e., sprouting while our agent is standing there)
          continue;
//        } else if (brokenActionName.startsWith("break_and_pickup") &&
//                (brokenActionPredicate.get(0).toString().equals("oak_log") || brokenActionPredicate.get(0).toString().equals("tree_tap")) &&
//                failure.toString().contains("sapling")) {
//          //don't update action when picking up sapling while breaking and picking up oak_log or tree_tap
//          continue;
        } else {
          if (beliefState.getName().equals("fluent_equals")) {
            ActionDBEntry fixedAction = fixedActionBuilder.build(true);
            Predicate fixedActionPredicate = Factory.createPredicate(fixedAction.getName(), brokenActionSignature.getArgs());
            ParameterizedAction parameterizedFixedAction = new ParameterizedAction(fixedAction, new Goal(actor, fixedActionPredicate));

            Effect fixedEffect = ExploreUtils.generateNewPostCondition(parameterizedFixedAction, discrepancy);
            Predicate existingEffectPred = fixedEffect.getPredicate();
            existingEffectPred.set(existingEffectPred.size() - 1, Factory.createVariable("Q")); // get matching effect regardless of quantity
            Effect existingEffect = ExploreUtils.getExistingEffect(parameterizedFixedAction, existingEffectPred);
            if (existingEffect == null) {
              // add new post-condition to action
              fixedActionBuilder.prependEffect(fixedEffect);
            } else {
              // update existing post-condition -- this only happens when this discrepancy has a "matching"
              // post-condition. The discrepancy is a redundant failure, but will be incorrectly handled by the matching
              // failed post-condition because the discrepancy will be updated in belief with the correct world state value before the
              // failure can be handled (e.g., observing fluent_increase will always be 0 bc belief has been updated with world value).
              // E.g., "discrepancy(fluent_equals(inventory(self,sapling),3),fluent_equals(inventory(self,sapling),4))"
              // and "not(fluent_increase(inventory(self,sapling),0))"
              fixedActionBuilder.replaceEffect(existingEffect, fixedEffect);
            }

            //report novelty
            novelties.add(failure);

          } else if (beliefState.toUnnegatedForm().getName().equals("at")) {
            // if item is a physobj (not an actor), update world count to match
            Predicate worldCountPred;
            try {
              Symbol item = beliefState.toUnnegatedForm().get(0);
              Predicate physobjQuery = Factory.createPredicate("type", item, Factory.createSymbol("physobj"));
              if (TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, physobjQuery)) {
                Predicate worldItem = Factory.createPredicate("world", item);
                if (beliefState.isNegated()) {
                  // incorrect belief is not(at(item,x,y)) --> increase world count
                  worldCountPred = Factory.createPredicate("fluent_increase", worldItem, Factory.createSymbol("1"));
                } else {
                  // incorrect belief is at(item,x,y) --> decrease world count
                  worldCountPred = Factory.createPredicate("fluent_decrease", worldItem, Factory.createSymbol("1"));
                }
                TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBelief")).call(Object.class, worldCountPred, MemoryLevel.EPISODIC);
              }
            } catch (TRADEException e) {
              log.error("Error trying to handle at() discrepancy: " + failure);
            }
            continue;
          } else {
            log.warn("This algorithm doesn't currently handle this kind of discrepancy: " + failure);
            justification.addJustification(new ConditionJustification(false, Factory.createPredicate("fixed", failure)));
            //report novelty
            novelties.add(failure);
            continue;
          }
        }
      } else if (brokenEffect.getPredicate().getVars().stream().anyMatch(var -> Utilities.isLocalVariable(var))) {
        // CASE 2
        // effect contains local variable, so we assume that the belief needs to be fixed, not the action
        // e.g., actionCost(break_break,oak_log,!actionCost)
        // we actually ONLY want to do this for actionCost predicates -- TODO: why is this is the case?

        // bind effect's input vars (i.e., starts with ?)
        Map<Variable, Symbol> nonLocalVarBindings = brokenEffect.getPredicate().getBindings(failure).entrySet()
                .stream().filter(entry -> !Utilities.isLocalVariable(entry.getKey()))
                .collect(Collectors.toMap(e -> e.getKey(), e -> e.getValue()));
        Predicate brokenActionPredicateNonLocalBound = brokenEffect.getPredicate().copyWithNewBindings(nonLocalVarBindings);

        // observe current state of effect -- state will be asserted to belief automatically if successfully observed
        Predicate observationGoalPred = Factory.createPredicate("observe", actor, brokenActionPredicateNonLocalBound);
        Justification observationJustification = ExploreUtils.doGoal(observationGoalPred);
        if (observationJustification.getValue()) {
          // add the observation result to UNIVERSAL, so it remains across games
          try {
            List<Map<Variable, Symbol>> bindings = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, brokenActionPredicateNonLocalBound);
            // TODO: how to know which things to assert to universal memory to persist across episodes?
            if (brokenActionPredicateNonLocalBound.toString().contains("cost")) {
              TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBelief")).call(Object.class, brokenActionPredicateNonLocalBound.copyWithNewBindings(bindings.get(0)), MemoryLevel.UNIVERSAL);
            }
          } catch (TRADEException e) {
            log.error("Exception trying to add updated post-condition belief to UNIVERSAL memory: " + brokenActionPredicateNonLocalBound);
          }
        } else {
          justification.addJustification(new ConditionJustification(false, Factory.createPredicate("fixed", failure)));
        }
        //report novelty
        novelties.add(brokenActionPredicateNonLocalBound);
      } else {
        // CASE 3
        // TODO : generalize this case by submitting generic "observe(did,failure)" goal and getting results
        // TODO: handle the negated case: not(fluent_equals())
        if (failure.getName().startsWith("fluent")) {
          // if fixed action no longer has broken effect, it must have been fixed/replaced by a redundant discrepancy() post-cond
          if (!fixedActionBuilder.hasEffect(brokenEffect)) {
            log.debug("Fixed action no longer contains broken effect: " + brokenEffect);
            continue;
          }

          try {
            if (TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, new Predicate("unfixable", brokenActionSignature.getName()))) {
              log.debug("Not attempting to fix action; we don't want to update it.");
              continue;
            }
          } catch (TRADEException e) {
            log.error("Error calling querySupport", e);
          }

          Predicate stateQuery = failure.clone();
          Variable quantityVar = Factory.createVariable("Q");
          stateQuery.set(stateQuery.size() - 1, quantityVar);
          try {
            List<Map<Variable, Symbol>> queryResults;
            if (failure.getName().startsWith("fluent") && !failure.getName().startsWith("fluent_equals")) {
              queryResults =TRADE.getAvailableService(new TRADEServiceConstraints().name("observeFluent")).call(List.class, actor, stateQuery);
            } else {
              queryResults =TRADE.getAvailableService(new TRADEServiceConstraints().name("observeState")).call(List.class, stateQuery);
            }
            log.debug("observeState results: " + stateQuery + " " + queryResults);
            if (!queryResults.isEmpty()) {
              // update action's post-condition, while preserve brokenEffect's free-variables
              Predicate fixedEffectPred = brokenEffect.getPredicate().clone();
              fixedEffectPred.set(fixedEffectPred.size() - 1, queryResults.get(0).get(quantityVar));
              fixedActionBuilder.replaceEffect(brokenEffect, new Effect(fixedEffectPred, EffectType.SUCCESS));
            } else {
              log.warn("Could not find true state of: " + stateQuery);
              justification.addJustification(new ConditionJustification(false, Factory.createPredicate("fixed", failure)));
            }
          } catch (TRADEException e) {
            log.error("Exception fixing postcondition", e);
            justification.addJustification(new ConditionJustification(false, Factory.createPredicate("fixed", failure)));
          }
          novelties.add(failure);
        } else if (failure.toUnnegatedForm().getName().equals("facing_obj")) {
          // TODO: this is not very general
          // try to observe the actual state of the failed post-condition to bring belief in line with state of the world,
          // and then try to re-plan

          // observe current state of effect -- state will be asserted to belief automatically if successfully observed
          // observe facing_obj(self, X, one) so that something gets asserted to belief (one always exists, two does not)
          Term generalizedEffectPred = Factory.createPredicate("facing_obj", actor, Factory.createVariable("OBJ"), Factory.createSymbol("one"));
          Predicate observationGoalPred = Factory.createPredicate("observe", actor, generalizedEffectPred);
          Justification observationJustification = ExploreUtils.doGoal(observationGoalPred);
          if (!observationJustification.getValue()) {
            log.warn("C this kind of post-condition failure: " + failure);
            justification.addJustification(new ConditionJustification(false, Factory.createPredicate("fixed", failure)));
          }
          // don't report novelty in this case
        } else {
          log.warn("This algorithm doesn't currently handle this kind of failure: " + failure);
          justification.addJustification(new ConditionJustification(false, Factory.createPredicate("fixed", failure)));
          novelties.add(failure);
        }
      }
    }

    if (generateConstantExploration != null) {
      try {
        // generate list of explorations to try
        Set<Predicate> explorationsToTry = TRADE.getAvailableService(new TRADEServiceConstraints().name("generateExplorationsToTry")).call(Set.class, generateConstantExploration);
        // assert explorations to try, which will be removed as they're explored
        // this allows explorations to continue into subsequent games if explorations can't be completed
        TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBeliefs")).call(Object.class, explorationsToTry, MemoryLevel.UNIVERSAL);
        //add to universal beliefs
      } catch (TRADEException e) {
        log.error("Error reporting novelties: ", e);
      }
    }

    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("reportNovelties")).call(Object.class, novelties);
    } catch (TRADEException e) {
      log.error("Error reporting novelties: ", e);
    }

    // if all failures have been handled successfully and the fixed action is different from the failed action
    if (justification.getValue() && !ExploreUtils.areActionsEffectivelyEqual(brokenAction, fixedActionBuilder.build(false))) {

      if (createNewAction) {
        // create object(?breakable01,objectToBreak) predicate
        Symbol inputArg = brokenActionSignature.get(0);
        Predicate typeConstraint = Factory.createPredicate("equals", Factory.createVariable(brokenAction.getInputRoles().get(1).name), inputArg);

        // update broken action to add not(object(?breakable01,objectToBreak)) pre-condition
        ActionDBEntry.Builder brokenActionBuilder = new ActionDBEntry.Builder(brokenAction);
        brokenActionBuilder.addCondition(new Condition(typeConstraint.toNegatedForm(), ConditionType.PRE, Observable.FALSE));
        Database.getInstance().removeActionDBEntry(brokenAction);
        brokenActionBuilder.build(true);

        // add object(?breakable01,objectToBreak) pre-condition so new action only applies to specific inputArg type
        fixedActionBuilder.addCondition(new Condition(typeConstraint, ConditionType.PRE, Observable.FALSE));
        fixedActionBuilder.build(true);
      } else {
        // update existing action by removing brokenAction and adding fixed version
        Database.getInstance().removeActionDBEntry(brokenAction);
        fixedActionBuilder.build(true);
      }
    }

    return justification;
  }

  boolean isChangeable(Predicate failure, ParameterizedAction parameterizedBrokenAction) {
    //check if the failure is something that we consider acceptably changeable
    Predicate effect = new Predicate("changeable", ExploreUtils.generateNewPostCondition(parameterizedBrokenAction, failure).getPredicate());
    try {
      if (TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, effect)) {
        return true;
      }
    } catch (TRADEException e) {
      log.error("Error calling querySupport", e);
    }
    return false;
  }

}
