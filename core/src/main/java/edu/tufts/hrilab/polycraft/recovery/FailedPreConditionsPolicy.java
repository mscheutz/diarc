/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.recovery;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.Condition;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.ParameterizedAction;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.justification.AndJustification;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.recovery.Utilities;
import edu.tufts.hrilab.belief.common.MemoryLevel;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.polycraft.recovery.util.ExploreUtils;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.List;

public class FailedPreConditionsPolicy {
  private static Logger log = LoggerFactory.getLogger(FailedPreConditionsPolicy.class);
  /**
   * Register this class with TRADE.
   */
  public void registerWithTrade(List<String> groups) {
    try {
      TRADE.registerAllServices(this, groups);
    } catch (TRADEException e) {
      log.error("Error trying to register with TRADE.", e);
    }
  }

  /**
   * Explore an action that has failed with FAIL_PRECONDITONS.
   *
   * @param actor
   * @param brokenActionSignature
   * @param failureReasons
   * @param goal
   * @return
   */
  @TRADEService
  @Action
  public Justification executeFailedPreConditionsPolicy(Symbol actor, Predicate brokenActionSignature, List<Predicate> failureReasons, Predicate goal) {
    if (brokenActionSignature == null) {
      return new ConditionJustification(false);
    }

    ActionDBEntry brokenAction = Utilities.getActionBySignature(brokenActionSignature);
    AndJustification justification = new AndJustification();

    for (Predicate failure : failureReasons) {
      // get failed pre-condition from failed action
      ParameterizedAction parameterizedAction = new ParameterizedAction(brokenAction, new Goal(actor, brokenActionSignature));
      Condition failedCondition = ExploreUtils.getExistingCondition(parameterizedAction, failure);
      if (failedCondition == null) {
        log.warn("Could not find failed Condition.");
        justification.addJustification(new ConditionJustification(false, Factory.createPredicate("fixed", failure)));
        continue;
      }

      // e.g., fluent_equals(cost_2(collect, ?safe), !actionCost)
      if (failure.getName().equals("fluent_equals") && failure.get(0).getName().startsWith("cost_")) {
        log.warn("No action cost found. Adding initial cost to Belief for: " + failure);
        try {
          // first check for baseline action cost in belief
          Predicate baselineCostFunction = Factory.createPredicate("cost_1", ((Term)failure.get(0)).get(0));
          Predicate baselineCostQuery = Factory.createPredicate("fluent_equals", baselineCostFunction, Factory.createVariable("COST"));
          Double cost = ExploreUtils.getActionCost(baselineCostQuery, false);
          if (cost == null) {
            // no baseline cost found, assume zero cost, which will be corrected in the post-condition recovery
            cost = 0.0;
          }
          Term actionCostAssertion = Factory.createPredicate("fluent_equals", failure.get(0), Factory.createSymbol(cost.toString()));
          TRADE.getAvailableService(new TRADEServiceConstraints().name("assertBelief")).call(Object.class, actionCostAssertion, MemoryLevel.UNIVERSAL);
        } catch (TRADEException e) {
          log.error("Error initializing new object properties in Belief.", e);
        }
      } else {
        // try to observe the actual state of the failed pre-condition to bring belief in line with state of the world,
        // and then try to re-plan

        // observe current state of effect -- state will be asserted to belief automatically if successfully observed
        Term generalizedConditionPred = failure.toUnnegatedForm().clone();
        // TODO: this is not very general and only really handles fluent, holding, facing_obj conditions
        if (generalizedConditionPred.getName().equals("facing_obj")) {
          // observer facing_obj(?actor, X, one) so that something gets asserted to belief (one always exists, two does not)
          generalizedConditionPred.set(generalizedConditionPred.size() - 1, Factory.createSymbol("one"));
          generalizedConditionPred.set(generalizedConditionPred.size() - 2, Factory.createVariable("OBJ"));
        } else if (generalizedConditionPred.getName().equals("at")) {
          generalizedConditionPred.set(generalizedConditionPred.size() - 1, Factory.createVariable("Y"));
          generalizedConditionPred.set(generalizedConditionPred.size() - 2, Factory.createVariable("X"));
        } else {
          generalizedConditionPred.set(generalizedConditionPred.size() - 1, Factory.createVariable("OBS"));
        }
        Predicate observationGoalPred = Factory.createPredicate("observe", actor, generalizedConditionPred);
        Justification observationJustification = ExploreUtils.doGoal(observationGoalPred);
        if (!observationJustification.getValue()) {
          log.warn("This algorithm doesn't currently handle this kind of pre-condition failure: " + failure);
          justification.addJustification(new ConditionJustification(false, Factory.createPredicate("fixed", failure)));
        }
      }
    }

    return justification;
  }
}
