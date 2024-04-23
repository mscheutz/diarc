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
import edu.tufts.hrilab.action.Observable;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.belief.common.MemoryLevel;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.polycraft.msg.Utilities;
import edu.tufts.hrilab.polycraft.recovery.util.ExploreUtils;
import edu.tufts.hrilab.polycraft.util.Recipe;

import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class ExplorationComponent extends DiarcComponent {
  private volatile AtomicInteger currentlyExploringBrokenAction = new AtomicInteger();
  private volatile AtomicBoolean currentlyExploringDetectedNovelties = new AtomicBoolean();

  // recovery policies
  private FailedToPlanPolicy failedToPlanPolicy = new FailedToPlanPolicy();
  private FailedPreConditionsPolicy failedPreConditionsPolicy = new FailedPreConditionsPolicy();
  private FailedPostConditionsPolicy failedPostConditionsPolicy = new FailedPostConditionsPolicy();
  private FailedReturnValuePolicy failedReturnValuePolicy = new FailedReturnValuePolicy();

  private Predicate lastBrokenAction;
  private Predicate topLevelGoal = Factory.createPredicate("goal(self,fluent_geq(inventory(self,pogo_stick),1))");
  public ExplorationComponent() {
    super();
    //TODO:brad: should these be in this component's groups?
    failedToPlanPolicy.registerWithTrade(this.getMyGroups());
    failedPreConditionsPolicy.registerWithTrade(this.getMyGroups());
    failedPostConditionsPolicy.registerWithTrade(this.getMyGroups());
    failedReturnValuePolicy.registerWithTrade(this.getMyGroups());
  }

  /**
   * Check if any exploration is currently being executed.
   *
   * @return
   */
  @TRADEService
  public Predicate getLastBrokenAction() {
    return lastBrokenAction;
  }

  /**
   * Check if any exploration is currently being executed.
   *
    * @return
   */
  @TRADEService
  public boolean isCurrentlyExploring() {
    return currentlyExploringDetectedNovelties.get();
  }

  @TRADEService
  public void executeExploration(Predicate explorationToTry) {
    Predicate explorationGoal;
    Symbol s = explorationToTry.get(1);
    // TODO: HACK: fix this wherever the goal is created
    if (!s.toString().endsWith(")")) {
      explorationGoal = Factory.createPredicate(s+"()");
    } else {
      explorationGoal = (Predicate) s;
    }

    //we need to know that we're exploring a detected novelty so that we don't try to repair any failed return values here
    currentlyExploringDetectedNovelties.set(true);

    boolean explored = false;
    Justification goalJustification = ExploreUtils.doGoal(explorationGoal);
    currentlyExploringDetectedNovelties.set(false);
    if (!goalJustification.getValue()) {
      // TODO: check reason for failure and only mark as explored if desired action was attempted
      //    (e.g., failure to plan bc object no longer exists shouldn't count as explored)

      // add not(equals(?breakable01,objectToBreak)) pre-condition to attempted action
      Symbol inputArg = explorationGoal.get(0);
      ActionDBEntry action = Database.getActionDB().getAction(explorationGoal.getName());
      ActionDBEntry.Builder actionBuilder = new ActionDBEntry.Builder(action);
      Predicate typeConstraint = Factory.createPredicate("equals", Factory.createVariable(action.getInputRoles().get(1).name), inputArg);
      actionBuilder.addCondition(new Condition(typeConstraint.toNegatedForm(), ConditionType.PRE, Observable.FALSE));
      Database.getInstance().removeActionDBEntry(action);
      actionBuilder.build(true);
      explored = true;
    } else {
      explored = true;
    }

    // mark as explored, or as still needing to explore
    try {
      if (explored) {
        Predicate exploredPredicate = Factory.createPredicate("explored", explorationToTry.getArgs());
        TRADE.getAvailableService(new TRADEServiceConstraints().name("retractBelief")).call(Object.class, explorationToTry);
        TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBelief")).call(Object.class, exploredPredicate);
      } else {
        TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBelief")).call(Object.class, explorationToTry);
      }
    } catch (TRADEException e) {
      log.error("Error updating belief with explored state.", e);
    }
  }

  @TRADEService
  public void initializeNovelty(Term novelty) {
    // previously "object" novelty
    if (novelty.getName().equals("constant")) {
      // initialize other info needed in belief for object types
      Symbol novelObject = novelty.get(0);
      try {
        // if novel object is a physobj, add additional subtyypes
        if (TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, Factory.createPredicate("typeobject", novelObject.toString(), "physobj"))) {
        // TODO: we should test that these properties hold before asserting them, or at least retract them if they don't hold
        Predicate breakablePred = Factory.createPredicate("subtype", novelObject.toString(), "breakable");
        Predicate placeablePred = Factory.createPredicate("subtype", novelObject.toString(), "placeable");

        Set<Predicate> novelObjectAssertions = new HashSet<>();
        novelObjectAssertions.add(breakablePred);
        novelObjectAssertions.add(placeablePred);
        TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBeliefs")).call(Object.class, novelObjectAssertions, MemoryLevel.UNIVERSAL);

        novelObjectAssertions = new HashSet<>();
        Predicate inventoryItem = Factory.createPredicate("inventory", Factory.createSymbol("self"), novelObject);
        Predicate worldItem = Factory.createPredicate("world", novelObject);
        Variable varX = Factory.createVariable("X");
        Symbol zero = Factory.createSymbol("0");
        if (!TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, Factory.createPredicate("fluent_equals", inventoryItem, varX))) {
          novelObjectAssertions.add(Factory.createPredicate("fluent_equals", inventoryItem, zero));
        }
        if (!TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, Factory.createPredicate("fluent_equals", worldItem, varX))) {
          novelObjectAssertions.add(Factory.createPredicate("fluent_equals", worldItem, zero));
        }
        TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBeliefs")).call(Object.class, novelObjectAssertions, MemoryLevel.EPISODIC);
        }
      } catch (TRADEException e) {
        log.error("Error initializing new object properties in Belief.", e);
      }
    } else if (novelty.getName().equals("recipe")) {
      // create action for new recipe
      Recipe.createRecipeAction(novelty);
    } else if (novelty.getName().equals("subtype")) {
      try {
        TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBelief")).call(Object.class,novelty,MemoryLevel.UNIVERSAL);
      } catch (TRADEException e) {
        log.error("Error initializing subtype property in Belief.",e);
      }
    } else {
      log.warn("[initializeBeliefsForNovelty] can't handle novelty: " + novelty);
    }
  }

  @TRADEService
  public Set<Predicate> generateExplorationsToTry(Term objectNovelty) {
    Set<Predicate> explorationsToTry = new HashSet<>();
    // assume actor is self
    Symbol actor = Factory.createSymbol("self");

    // formerly "object" -- changed to "constant" for pddl compatibility
    if (objectNovelty.getName().equals("constant")) {
      Symbol novelObject = objectNovelty.get(0);
      log.debug("novelObject: " + novelObject);
      try {
        if (TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, Factory.createPredicate("typeobject", novelObject.toString(), "agent"))) {
          // novel object is an actor, try to interact with it
          Predicate interactGoal = Factory.createPredicate("interact_with", novelObject); // e.g., interact_with(supplier_105))
          Predicate explorePred = Factory.createPredicate("toExplore", objectNovelty, interactGoal);
          explorationsToTry.add(explorePred);
        } else if (TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, Factory.createPredicate("typeobject", novelObject.toString(), "physobj"))) {
          // get object properties
          Map<String, Boolean> objectStatusMap = ExploreUtils.checkObjectStatus(novelObject.toString()); // not_present, inventory, floating, block, recipe
          log.debug("objectStatusMap: " + objectStatusMap);

          ///////////////////////////////////////////////////////////////////
          // exploration to try to break novel object (if object is in the world and not floating)
          ///////////////////////////////////////////////////////////////////
          if (objectStatusMap.get("block") && !objectStatusMap.get("floating")) {

            // try to break novel object
            Predicate goal = Factory.createPredicate("break_and_pickup", actor, novelObject); // e.g., break_and_pickup(axe)

            // mark as explored
            Predicate exploredPredicate = Factory.createPredicate("explored", objectNovelty, goal);
            try {
              TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBelief")).call(Object.class, exploredPredicate);
            } catch (TRADEException e) {
              log.error("Error marking exploredPredicate: " + exploredPredicate, e);
            }

            //submit breaking goal now so we don't have to explore any of the actions that require breaking
            currentlyExploringDetectedNovelties.set(true);
            Justification subGoalJustification = ExploreUtils.doGoal(goal);
            currentlyExploringDetectedNovelties.set(false);
            if (!subGoalJustification.getValue()) {
              // can't break novel object -- retract breakable
              Predicate breakablePred = Factory.createPredicate("subtype(" + novelObject + ",breakable)");
              try {
                TRADE.getAvailableService(new TRADEServiceConstraints().name("retractBelief")).call(Object.class, breakablePred);
              } catch (TRADEException e) {
                log.error("Error removing breakable subtype: ", e);
              }
              return new HashSet<>();
            }
          }

          ///////////////////////////////////////////////////////////////////
          // explorations to try to select the object, and then try to break other objects in the world
          ///////////////////////////////////////////////////////////////////

          // get all physobj object types in the world with world count >= 1
          Predicate worldPred = Factory.createPredicate("fluent_equals(world(X), Y)");
          List<Map<Variable, Symbol>> worldObjects = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, worldPred);

          // get all object types in the world
          List<Symbol> worldObjectsList = new ArrayList<>();
          for (Map<Variable, Symbol> object2 : worldObjects) {
            Variable Y = Factory.createVariable("Y");
            Symbol obj = object2.get(Factory.createVariable("X"));
            double quant = Double.parseDouble(object2.get(Y).toString());
            Predicate physobjPred = Factory.createPredicate("type", obj, Factory.createSymbol("physobj"));
            if (quant > 0 && TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, physobjPred)) {
              worldObjectsList.add(obj);
            }
          }
          log.debug("worldObjectsList: " + worldObjectsList);

          // get existing action to modify (actually modifying a copy)
          String existingActionName = "break_and_pickup";
          ActionDBEntry existingAction = Database.getActionDB().getAction(existingActionName);

          // modify original existing action to have not(holding(?actor,novelObject)) pre-condition
          // NOTE: this should happen regardless of new action success/failure
          ActionDBEntry.Builder existingActionBuilder = new ActionDBEntry.Builder(existingAction);
          Predicate holdingPredicate = Factory.createPredicate("holding", Factory.createVariable("?actor"), novelObject);
          Predicate notHoldingPred = Factory.createPredicate("not", holdingPredicate);
          existingActionBuilder.addCondition(new Condition(notHoldingPred, ConditionType.PRE, Observable.TRUE));
          Database.getInstance().removeActionDBEntry(existingAction);
          existingActionBuilder.build(true);

          // create new action to be used during exploration and add it to the DB
          ActionDBEntry.Builder newActionBuilder = new ActionDBEntry.Builder(existingAction);
          String newActionName = existingActionName + "_holding_" + novelObject;
          newActionBuilder.setType(newActionName);
          newActionBuilder.addCondition(new Condition(holdingPredicate, ConditionType.PRE, Observable.TRUE));
          newActionBuilder.build(true);

          for (Symbol objectToBreak : worldObjectsList) {
            Predicate breakGoal = Factory.createPredicate(newActionName, objectToBreak); // break_and_pickup_holding_axe(oak_log)
            Predicate explorePred = Factory.createPredicate("toExplore", objectNovelty, breakGoal);
            explorationsToTry.add(explorePred);
          }
        }
      } catch (TRADEException e) {
        log.error("Error generating explorations to try for novelty: " + objectNovelty, e);
      }
    }

    return explorationsToTry;
  }

  /**
   * Check if the novelObject is an environmental novelty.
   *
   * @param actor
   * @param novelObject
   * @return
   * @throws TRADEException
   */
  private boolean isEnvironmental(Symbol actor, Symbol novelObject) throws TRADEException {
    // first check if actor is already in same block as novel object
    Variable xVar = Factory.createVariable("X");
    Variable yVar = Factory.createVariable("Y");
    Predicate actorAt = Factory.createPredicate("at", actor, xVar, yVar);
    List<Map<Variable, Symbol>> actorAtBindings = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief")).call(List.class, actorAt);
    Predicate novelObjAt = Factory.createPredicate("at", novelObject, xVar, yVar);
    if (Utilities.queryBeliefSupport(novelObjAt.copyWithNewBindings(actorAtBindings.get(0)))) {
      return true;
    }

    // if not co-located in block with novel object, try to face it, and move forward into it
    Predicate facingObjectGoal = Factory.createPredicate("facing_obj", actor, novelObject, Factory.createSymbol("one"));
    if (ExploreUtils.doGoal(facingObjectGoal).getValue()) {
      // now facing novel object -- try to walk into novel object block
      Predicate moveForwardGoal = Factory.createPredicate("move(W)"); // move forward with no cond/effect checks
      if (ExploreUtils.doGoal(moveForwardGoal).getValue()) {
        // must be "air" variant (i.e., can move into novel object block)
        return true;
      }
    }

    return false;
  }

  /**
   * If the novelObject is an environmental novelty, update beliefs to reflect this, and re-initialize
   * belief and the map to make use of this information.
   *
   * @param novelObject
   */
  private void updateWithNewEnvironmentalType(Symbol novelObject) {
    // mark novel object as "air" subtype, and retract breakable and placeable
    Predicate breakablePred = Factory.createPredicate("subtype", novelObject.toString(), "breakable");
    Predicate placeablePred = Factory.createPredicate("subtype", novelObject.toString(), "placeable");
    Predicate airPred = Factory.createPredicate("subtype", novelObject.toString(), "air");
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("retractBelief")).call(Object.class, breakablePred);
      TRADE.getAvailableService(new TRADEServiceConstraints().name("retractBelief")).call(Object.class, placeablePred);
      TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBelief")).call(Object.class, airPred, MemoryLevel.UNIVERSAL);
    } catch (TRADEException e) {
      log.error("Error updating novel object's subtypes: " + novelObject, e);
    }

    // re-initialize world to regenerate map, entryways, beliefs, etc
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("initializeBeliefs")).call(Object.class);
    } catch (TRADEException e) {
      log.error("Error trying to re-initialzeBeliefs with environmental novelty.", e);
    }
  }

}
