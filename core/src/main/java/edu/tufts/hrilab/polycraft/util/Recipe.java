/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.util;

import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.Condition;
import edu.tufts.hrilab.action.ConditionType;
import edu.tufts.hrilab.action.Effect;
import edu.tufts.hrilab.action.EffectType;
import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.Observable;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

public class Recipe {
  static private Logger log = LoggerFactory.getLogger(Recipe.class);

  static public Term createRecipeAction(Term recipe) {
    if (recipe.getName().equals("recipe")) {
      log.debug("Creating new action for recipe: " + recipe);
      int size = recipe.size();
      String target = recipe.get(size - 2).toString(); // get the target of the recipe.
      Symbol targetQuantity = Factory.createSymbol(recipe.get(size - 1).toString());// get the quantity of the target item the recipe makes.

      String actionName = "sensed_craft_" + target;
      if (!Database.getActionDB().actionExists(actionName)) {
        ActionDBEntry.Builder newActionBuilder = new ActionDBEntry.Builder(actionName);
        Map<String, Integer> itemCounts = new HashMap<>();

        // get input ingredient counts for items needed in recipe
        for (int i = 0; i < recipe.size() - 2; ++i) {
          String slot = recipe.get(i).toString();
          log.debug("slot = " + slot);
          if (!(slot.equals("0"))) { // for cases when there is something in the recipe. For example s = "plank";
            if ((itemCounts.containsKey(slot))) {
              itemCounts.put(slot, itemCounts.get(slot) + 1);
            } else {
              itemCounts.put(slot, 1);
            }
          }
        }

        // precondition for making the agent go near the crafting table.
        // TODO: how to know which craft actions need to have facing_obj(?actor,crafting_table,one) pre-condition?
        Condition newPrecondition = new Condition(Factory.createPredicate("facing_obj", "?actor", "crafting_table", "one"), ConditionType.PRE);
        newActionBuilder.addCondition(newPrecondition);

        for (String key : itemCounts.keySet()) {
          String inputArgName = "?"+key;
          String inputSemanticType = key;
          Symbol inputCount = Factory.createSymbol(itemCounts.get(key).toString());
          Predicate inventoryItem = Factory.createPredicate("inventory", "?actor", inputArgName);

          // add input args
          newActionBuilder.addRole(new ActionBinding.Builder(inputArgName, Symbol.class).setSemanticType(inputSemanticType).build());
          // generate the pre conditions
          newPrecondition = new Condition(Factory.createPredicate("fluent_geq", inventoryItem, inputCount), ConditionType.PRE, Observable.TRUE);
          newActionBuilder.addCondition(newPrecondition);
          // generate the effects
          Effect newEffect = new Effect(Factory.createPredicate("fluent_decrease", inventoryItem, inputCount), EffectType.SUCCESS, Observable.TRUE);
          newActionBuilder.addEffect(newEffect);
        }

        // add the effect of the target in inventory.
        Predicate inventoryItem = Factory.createPredicate("inventory", "?actor", target);
        Effect newEffect = new Effect(Factory.createPredicate("fluent_increase", inventoryItem, targetQuantity), EffectType.SUCCESS, Observable.TRUE);
        newActionBuilder.addEffect(newEffect);

        // TODO: add actionCost pre- and post-condition

        // add post-condition discrepancy check (this must be the last effect)
        newEffect = new Effect(Factory.createPredicate("not(discrepancy(!x,!y))"), EffectType.SUCCESS, Observable.TRUE);
        newActionBuilder.addEffect(newEffect);
        newActionBuilder.addRole(new ActionBinding.Builder("!x", edu.tufts.hrilab.fol.Symbol.class).setIsLocal(true).build());
        newActionBuilder.addRole(new ActionBinding.Builder("!y", edu.tufts.hrilab.fol.Symbol.class).setIsLocal(true).build());

        // add action step (and logging step)
        newActionBuilder.addEventSpec(new EventSpec(Factory.createPredicate("craft", recipe)));
        newActionBuilder.addEventSpec(new EventSpec.Builder(EventSpec.EventType.OPERATOR)
                .setCommand("log")
                .addInputArg("\"info\"").addInputArg("\">> crafted sensed_" + target + "\"")
                .build());
        newActionBuilder.build(true);

        return Factory.createPredicate(actionName);
      } else {
        log.warn("Action " + actionName + " already exists.");
      }
    }

    return null;
  }

  /**
   * Create actions for each novel recipe.
   */
  static public void createRecipeActions(Set<? extends Term> novelRecipes) {
    // create new craft action for recipe
    for (Term recipe : novelRecipes) {
      Recipe.createRecipeAction(recipe);
    }
  }

}
