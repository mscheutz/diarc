/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.recovery.util;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.Condition;
import edu.tufts.hrilab.action.Effect;
import edu.tufts.hrilab.action.EffectType;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.Observable;
import edu.tufts.hrilab.action.ParameterizedAction;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

public class ExploreUtils {

  static private Logger log = LoggerFactory.getLogger(ExploreUtils.class);

  /**
   * Helper method to get the properties of an object.
   * <p>
   * not_present [T/F]: if all other properties are false, this will be true
   * inventory [T/F]: if object is in agent's inventory
   * block [T/F]: if object is in the world (i.e., fluent_equals(world(object), >0))
   * floating [T/F]: if object type is floating
   * recipe [T/F]: if a craft action to craft the object exists
   *
   * @param object
   * @return
   */
  static public Map<String, Boolean> checkObjectStatus(String object) {
    Map<String, Boolean> objectStatusMap;

    objectStatusMap = new HashMap<>();
    objectStatusMap.put("not_present", true);
    objectStatusMap.put("inventory", false);
    objectStatusMap.put("block", false);
    objectStatusMap.put("floating", false);
    objectStatusMap.put("recipe", false);

    try {
      // Finding if object is in inventory
      Predicate query = Factory.createPredicate("fluent_equals", "inventory(self," + object + ")", "X");
      log.debug("query: " + query);
      List<Map<Variable, Symbol>> inventoryObjectQuantity = (List<Map<Variable, Symbol>>) TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(Object.class, query);
      log.debug("inventory: " + inventoryObjectQuantity); // [{X=1.0}]

      for (Map<Variable, Symbol> quantity : inventoryObjectQuantity) {
        Variable X = new Variable("X");
        double q = Double.parseDouble(quantity.get(X).toString()); // 1.0
        if (q > 0) {
          objectStatusMap.put("inventory", true);
        }
      }

      // check if the object is a floating entity
      query = Factory.createPredicate("floating", object);
      log.debug("query: " + query);
      List<Map<Variable, Symbol>> novelObjects = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, query);
      log.debug("floating: " + novelObjects);
      // If the list has an empty map inside it, that means objectName is floating - [{}].
      // If the list is empty that means objectName is not floating - [].
      if (novelObjects.size() > 0) {
        objectStatusMap.put("floating", true);
      }

      // query if its a block
      query = Factory.createPredicate("fluent_equals", "world(" + object + ")", "X");
      log.debug("query: " + query);
      List<Map<Variable, Symbol>> blockObjects = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, query);
      log.debug("world: " + blockObjects); //world = [{X=6.0}]
      for (Map<Variable, Symbol> quantity : blockObjects) {
        Variable X = new Variable("X");
        double q = Double.parseDouble(quantity.get(X).toString()); // 6.0

        if (q > 0) {
          objectStatusMap.put("block", true);
        }
      }

      // query if there is a recipe action for object
      String recipeAction = "craft_" + object;
      String sensedRecipeAction = "sensed_craft_" + object;
      if (Database.getActionDB().actionExists(recipeAction) || Database.getActionDB().actionExists(sensedRecipeAction)) {
        objectStatusMap.put("recipe", true);
      }

      for (Map.Entry<String, Boolean> status : objectStatusMap.entrySet()) {
        if (!status.getKey().equals("not_present") && status.getValue()) {
          objectStatusMap.put("not_present", false);
          break;
        }
      }

    } catch (TRADEException e) {
      log.error("Error getting object status.", e);
    }

    return objectStatusMap;
  }

  /**
   * Helper method to submit a goal, and wait for it to terminate. Note that if a goal with a matching goal predicate
   * is already active, a new goal will not be submitted, and false will be returned.
   *
   * @param goalPredicate
   * @return if the goal was successfully executed
   */
  static public Justification doGoal(Predicate goalPredicate) {
    Goal goal = new Goal(goalPredicate);
    try {
      //TODO: Do we want goals from this method to skip/override the execution manager?
      //      Depending on the answer to the above, do we only care if a matching goal is currently active or submitted?
      //Currently made methods in GM and swapped calls here to do so
      List<Predicate> currGoals = TRADE.getAvailableService(new TRADEServiceConstraints().name("getActiveGoals")).call(List.class, goal.getActor());
      if (currGoals.contains(goalPredicate)) {
        log.info("Goal is already a current goal. Skipping this goal to prevent a deadlock. Goal: " + goalPredicate);
        return new ConditionJustification(false);
      }

      log.info("Submitting goal: " + goalPredicate);
      long goalID = TRADE.getAvailableService(new TRADEServiceConstraints().name("submitGoal")).call(Long.class, goalPredicate);
      GoalStatus goalStatus = TRADE.getAvailableService(new TRADEServiceConstraints().name("joinOnGoal")).call(GoalStatus.class, goalID);
      log.info("Goal Status for goal (ID=" + goalID + "): " + goalStatus);
      if (goalStatus == GoalStatus.SUCCEEDED) {
        return new ConditionJustification(true);
      } else if (goalStatus == GoalStatus.FAILED) {
        log.debug("Goal failed: " + goalPredicate);
        Justification goalJustification = TRADE.getAvailableService(new TRADEServiceConstraints().name("getGoalFailConditions")).call(Justification.class, goalID);
        return goalJustification;
      } else {
        return new ConditionJustification(false);
      }
    } catch (TRADEException e) {
      log.error("[doGoal] Error for goal: " + goalPredicate, e);
    }
    return new ConditionJustification(false);
  }

  /**
   * Convenience method to retrieve the action cost from Belief (observe==false) or by making a game state observation (observation==true).
   * Returns null if the cost couldn't be found.
   */
  static public Double getActionCost(Predicate actionCostQuery, boolean observe) {
    try {
      List<Map<Variable, Symbol>> queryResults;
      if (observe) {
        queryResults =TRADE.getAvailableService(new TRADEServiceConstraints().name("observeCost")).call(List.class, actionCostQuery);
      } else {
        queryResults =TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, actionCostQuery);
      }

      if (!queryResults.isEmpty()) {
        return Double.parseDouble(queryResults.get(0).get(actionCostQuery.get(actionCostQuery.size() - 1)).toString());
      } else {
        log.warn("Couldn't get existing action cost: " + actionCostQuery);
        return null;
      }
    } catch (TRADEException e) {
      log.error("Error querying belief for actionCost: " + actionCostQuery, e);
    }

    return null;
  }

  /**
   * Convenience method to get novelties that have been detected (and reported).
   *
   * @param level
   * @return
   */
  static public Set<Predicate> getDetectedNovelties(Symbol level) {
    Set<Predicate> novelties = new HashSet<>();
    Variable noveltyVar = Factory.createVariable("NOVELTY");
    Predicate noveltyPred = Factory.createPredicate("novelty", noveltyVar, level);
    try {
      List<Map<Variable, Symbol>> noveltyBindings = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, noveltyPred);
      noveltyBindings.forEach(binding -> novelties.add((Predicate) binding.get(noveltyVar)));
    } catch (TRADEException e) {
      log.error("Error getting novelties from Belief.");
    }

    return novelties;
  }

  static public Condition getExistingCondition(ParameterizedAction parameterizedAction, Predicate conditionPredicate) {
    List<Condition> existingConditions = parameterizedAction.getEntry().getConditions().stream().filter(
            condition -> {
              if (condition.isDisjunction()) {
                log.warn("[getExistingCondition] does not currently check against disjunctive conditions.");
                return false;
              }
              Predicate existingConditionPred = parameterizedAction.bindPredicate(condition.getPredicates().keySet().iterator().next());
              return conditionPredicate.instanceOf(existingConditionPred) || existingConditionPred.instanceOf(conditionPredicate);
            }
    ).collect(Collectors.toList());

    if (existingConditions.isEmpty()) {
      log.debug("No existing condition could be found. Action: " + parameterizedAction + " effect: " + conditionPredicate);
      return null;
    } else if (existingConditions.size() != 1) {
      log.error("Unique existing effect couldn't be found. Action: " + parameterizedAction + " effect: " + conditionPredicate);
    }
    return existingConditions.get(0);
  }

  static public Effect getExistingEffect(ParameterizedAction parameterizedAction, Predicate effectPredicate) {
    List<Effect> existingEffects = parameterizedAction.getEntry().getEffects().stream().filter(
            effect -> {
              Predicate existingEffectPred = parameterizedAction.bindPredicate(effect.getPredicate());
              return effectPredicate.instanceOf(existingEffectPred) || existingEffectPred.instanceOf(effectPredicate);
            }
    ).collect(Collectors.toList());

    if (existingEffects.isEmpty()) {
      log.debug("No existing effect could be found. Action: " + parameterizedAction + " effect: " + effectPredicate);
      return null;
    } else if (existingEffects.size() != 1) {
      log.error("Unique existing effect couldn't be found. Action: " + parameterizedAction + " effect: " + effectPredicate);
    }
    return existingEffects.get(0);
  }

  static public boolean areActionsEffectivelyEqual(ActionDBEntry action1, ActionDBEntry action2) {
    if (!action1.getConditions().equals(action2.getConditions())) return false;
    if (!action1.getEffects().stream().filter(effect -> !effect.isAutoGenerated()).collect(Collectors.toList())
            .equals(action2.getEffects().stream().filter(effect -> !effect.isAutoGenerated()).collect(Collectors.toList())))
      return false;
    if (!action1.getEventSpecs().equals(action2.getEventSpecs())) return false;
    return true;
  }

  /**
   * Generate a new Effect based on a discrepancy novelty, when there is no existing
   * Effect that has been violated.
   *
   * TODO: generalize arguments in generated Effect (e.g., self -> ?actor, etc)
   *
   * @param discrepancy
   * @return
   */
  static public Effect generateNewPostCondition(ParameterizedAction parameterizedAction, Term discrepancy) {
    discrepancy = discrepancy.toUnnegatedForm();
    Term beliefState = (Term) discrepancy.get(0);
    Term worldState = (Term) discrepancy.get(1);
    if (beliefState.getName().equals("fluent_equals")) {
      double beliefAmount = Double.valueOf(beliefState.get(1).getName());
      double worldAmount = Double.valueOf(worldState.get(1).getName());
      double changeAmount = worldAmount - beliefAmount;

      // create new fluent change predicate
      List<Symbol> args = beliefState.getArgs();
      args.set(1, Factory.createSymbol(Double.toString(Math.abs(changeAmount))));
      Predicate newFluentChange;
      if (changeAmount > 0) {
        newFluentChange = Factory.createPredicate("fluent_increase", args);
      } else {
        newFluentChange = Factory.createPredicate("fluent_decrease", args);
      }

//      newFluentChange = unbindPredicate(parameterizedAction, newFluentChange);

      return new Effect(newFluentChange, EffectType.SUCCESS, Observable.TRUE);
    } else {
      log.error("Can't generate new Effect for discrepancy: " + discrepancy);
      return null;
    }
  }

//  /**
//   * Use bindings from parameterized action to unbind values in a predicate.
//   * @param parameterizedAction
//   * @return
//   */
//  static public Predicate unbindPredicate(ParameterizedAction parameterizedAction, Predicate predicate) {
//    // create value -> variable map from bindings
//    Map<Symbol, Variable> reverseBindings = new HashMap<>();
//    parameterizedAction.getBindings().forEach((var,val) -> reverseBindings.put(Util.createSymbol(val.toString()), Util.createVariable(var)));
//
//    Map<Symbol, Variable> reverseBindingsToApply = new HashMap<>();
//    predicate.getOrderedLeaves().stream().filter(leaf -> reverseBindings.containsKey(leaf)).forEach(leaf -> reverseBindingsToApply.put(leaf, reverseBindings.get(leaf)));
//
//    predicate.applyBindingMap();
//  }

  /**
   * Generate a new Effect, based on an existing incorrect Effect, and a discrepancy novelty.
   * For example, if there's an existing fluent_increase effect, but a different amount has been observed.
   *
   * @param effect
   * @param discrepancy
   * @return
   */
  static public Effect generateUpdatedPostCondition(Effect effect, Term discrepancy) {
    Term beliefState = (Term) discrepancy.get(0);
    Term worldState = (Term) discrepancy.get(1);

    double beliefAmount = Double.valueOf(beliefState.get( 1).getName());
    double worldAmount = Double.valueOf(worldState.get(1).getName());
    double diff = worldAmount - beliefAmount;

    // combine discrepancy amount with expected change amount
    Predicate currFluentChange = effect.getPredicate();
    double currFluentAmount = Double.valueOf(currFluentChange.get(1).getName());
    double newAmount = 0;
    if (currFluentChange.getName().equals("fluent_increase")) {
      newAmount = diff + currFluentAmount;
    } else if (currFluentChange.getName().equals("fluent_decrease")) {
      newAmount = diff - currFluentAmount;
    }

    // create new fluent change predicate
    List<Symbol> args = currFluentChange.getArgs();
    args.set(1, Factory.createSymbol(Double.toString(Math.abs(newAmount))));
    Predicate newFluentChange;
    if (newAmount > 0) {
      newFluentChange = Factory.createPredicate("fluent_increase", args);
    } else {
      newFluentChange = Factory.createPredicate("fluent_decrease", args);
    }

    return new Effect(newFluentChange, EffectType.SUCCESS, Observable.TRUE);
  }

  /**
   * Convenience method to query belief for all concrete objects of a particular semantic type.
   * @param semanticType
   * @return
   */
  static public List<Symbol> getObjectsOfType(Symbol semanticType) {
    List<Symbol> objects = new ArrayList<>();
    Variable var = Factory.createVariable("OBJ");
    Predicate query = Factory.createPredicate("typeobject", var, semanticType); // e.g., typeobject(OBJ,trader)
    List<Map<Variable, Symbol>> queryBindings;
    try {
      queryBindings =TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, query);
    } catch (TRADEException e) {
      log.error("Exception getting objects of type: " + semanticType, e);
      return objects;
    }
    for (Map<Variable, Symbol> queryBinding : queryBindings) {
      Symbol object = queryBinding.get(var);
      objects.add(object);
    }
    return objects;
  }
}
