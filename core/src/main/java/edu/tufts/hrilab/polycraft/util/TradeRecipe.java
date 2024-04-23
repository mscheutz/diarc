/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.util;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.Condition;
import edu.tufts.hrilab.action.ConditionType;
import edu.tufts.hrilab.action.Effect;
import edu.tufts.hrilab.action.EffectType;
import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.Observable;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.belief.common.MemoryLevel;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.polycraft.actions.Interact;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class TradeRecipe {
  static private Logger log = LoggerFactory.getLogger(TradeRecipe.class);

  static public void createTradeAction(Interact.TradeOption trade, String actorName, String actorID, int tradeCount) {

    String actionName = "trade_" + tradeCount;
    if (!Database.getActionDB().actionExists(actionName)) {
      ActionDBEntry.Builder newActionBuilder = new ActionDBEntry.Builder(actionName);
      // generate conds/effects for trade inputs
      StringBuilder tradeInput = new StringBuilder("input(");
      for (Map.Entry<String, Integer> entry : trade.input.entrySet()) {
        String inputSemanticType = SymbolResolver.toGridItem(entry.getKey());
        String inputArgName = "?" + inputSemanticType;
        Symbol inputQuant = Factory.createSymbol(entry.getValue().toString());

        // add input args
        newActionBuilder.addRole(new ActionBinding.Builder(inputArgName, Symbol.class).setSemanticType(inputSemanticType).build());
        // add pre-conditions
        Predicate inventory = Factory.createPredicate("inventory", "?actor", inputArgName);
        Condition newPrecondition = new Condition(Factory.createPredicate("fluent_geq", inventory, inputQuant), ConditionType.PRE, Observable.TRUE);
        newActionBuilder.addCondition(newPrecondition);
        // add effects
        Effect newEffect = new Effect(Factory.createPredicate("fluent_decrease", inventory, inputQuant), EffectType.SUCCESS, Observable.TRUE);
        newActionBuilder.addEffect(newEffect);
        // start making event spec (i.e., action step)
        tradeInput.append(inputSemanticType).append(",").append(inputQuant).append(",");
      }
      tradeInput.replace(tradeInput.length() - 1, tradeInput.length(), ")");

      // generate effects for trade outputs
      StringBuilder tradeOutput = new StringBuilder("output(");
      for (Map.Entry<String, Integer> entry : trade.output.entrySet()) {
        String key = entry.getKey();
        String outputItem = SymbolResolver.toGridItem(key);
        Symbol outputQuant = Factory.createSymbol(entry.getValue().toString());
        Predicate inventory = Factory.createPredicate("inventory", "?actor", outputItem);
        Effect newEffect = new Effect(Factory.createPredicate("fluent_increase", inventory, outputQuant), EffectType.SUCCESS, Observable.TRUE);
        newActionBuilder.addEffect(newEffect);
        tradeOutput.append(outputItem).append(",").append(outputQuant).append(",");
      }
      tradeOutput.replace(tradeOutput.length() - 1, tradeOutput.length(), ")");

      // adding precond to face the trader
      Condition newPrecondition = new Condition(Factory.createPredicate("facing_obj", "?actor", actorName, "two"), ConditionType.PRE);
      newActionBuilder.addCondition(newPrecondition);

      // TODO: add actionCost pre- and post-condition

      // add post-condition discrepancy check (this must be the last effect)
      Effect newEffect = new Effect(Factory.createPredicate("not(discrepancy(!x,!y))"), EffectType.SUCCESS, Observable.TRUE);
      newActionBuilder.addEffect(newEffect);
      newActionBuilder.addRole(new ActionBinding.Builder("!x", edu.tufts.hrilab.fol.Symbol.class).setIsLocal(true).build());
      newActionBuilder.addRole(new ActionBinding.Builder("!y", edu.tufts.hrilab.fol.Symbol.class).setIsLocal(true).build());

      // adding action specs
      StringBuilder tradeRecipe = new StringBuilder("trade(").append(tradeInput).append(",").append(tradeOutput).append(")");
      newActionBuilder.addEventSpec(new EventSpec.Builder(EventSpec.EventType.OPERATOR)
              .setCommand("log")
              .addInputArg("\"info\"").addInputArg("\">> " + actionName + "\"")
              .build());
      newActionBuilder.addEventSpec(new EventSpec(Factory.createPredicate("trade", actorID, tradeRecipe.toString())));

      // Querying existing trades from belief to detect novelty
      try {
        Predicate tradeToUse = Factory.createPredicate(tradeRecipe.toString());
        boolean tradePresent = TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport")).call(boolean.class, tradeToUse);
        if (!tradePresent) {
          // report novelty
          Set<Predicate> tradeToUseSet = new HashSet<>();
          tradeToUseSet.add(tradeToUse);
          TRADE.getAvailableService(new TRADEServiceConstraints().name("reportNovelties")).call(Object.class, tradeToUseSet);

          // add new trade recipe to belief
          TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBelief")).call(Object.class, tradeToUse, MemoryLevel.UNIVERSAL);
        }
      } catch (TRADEException e) {
        log.error("[Interact exception] Error querying trade.");
      }

      newActionBuilder.build(true);
      log.debug("Generated new trade action " + actionName + tradeRecipe);
    }
  }

  /**
   * Create actions for each novel recipe.
   */
  static public void createTradeActions(List<Interact.TradeOption> tradeRecipes, String actorName, String actorID, int tradeCount) {
    // create new craft action for recipe
    for (Interact.TradeOption trade : tradeRecipes) {
      TradeRecipe.createTradeAction(trade, actorName, actorID, tradeCount++);
    }
  }
}
