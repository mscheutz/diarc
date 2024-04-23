/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.observers;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.Observes;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class OperatorCheck {
  private static Logger log = LoggerFactory.getLogger(OperatorCheck.class);

  // automatically register operator observers with TRADE
  static {
    OperatorCheck checker = new OperatorCheck();
    try {
      TRADE.registerAllServices(checker, new ArrayList<>());
    } catch (TRADEException e) {
      log.error("Error registering OperatorCheck.", e);
    }
  }

  @TRADEService
  @Observes({"fluent_increase(X,Y)", "fluent_decrease(X,Y)", "fluent_leq(X,Y)", "fluent_geq(X,Y)"})
  static public List<Map<Variable, Symbol>> observeFluent(Symbol actor, Term state) {

    List<Map<Variable, Symbol>> results = new ArrayList<>();
    log.debug("[observeFluent] Observing: " + state);

    // if second arg is a Term, get the numeric world value (else it's a Variable or a numeric value) and
    // set it in the state query
    if (state.get(1).isTerm()) {
      Predicate queryPred = Factory.createPredicate("fluent_equals", state.get(1), Factory.createVariable("Y"));
      double actualAmount = getWorldStateAmount(actor, queryPred);
      state = Factory.createPredicate(state.getName(), state.get(0), Factory.createSymbol(String.valueOf(actualAmount)));
    }

    // get numeric belief value and world value of first arg
    Predicate queryPred = Factory.createPredicate("fluent_equals", state.get(0), Factory.createVariable("Y"));
    double beliefAmount = getBeliefStateAmount(queryPred); // IMPORTANT: this must be called before getWorldStateAmount updates belief
    double worldAmount = getWorldStateAmount(actor, queryPred); // this will update belief

    Map<Variable, Symbol> binding = new HashMap<>();
    switch (state.getName()) {
      case "increase":
      case "fluent_increase": {
        Symbol amount = state.get(1);
        if (amount.isVariable()) {
          binding.put((Variable) amount, Factory.createSymbol(String.valueOf(worldAmount - beliefAmount)));
          results.add(binding);
        } else {
          if ((worldAmount - beliefAmount) == Double.valueOf(amount.toString())) {
            results.add(binding); // i.e., true
          }
        }
        break;
      }
      case "decrease":
      case "fluent_decrease": {
        Symbol amount = state.get(1);
        if (amount.isVariable()) {
          binding.put((Variable) amount, Factory.createSymbol(String.valueOf(beliefAmount - worldAmount)));
          results.add(binding);
        } else {
          if ((beliefAmount - worldAmount) == Double.valueOf(amount.toString())) {
            results.add(binding); // i.e., true
          }
        }
        break;
      }
      case "leq":
      case "fluent_leq": {
        Symbol amount = state.get(1);
        if (!amount.isVariable()) {
          if (worldAmount <= Double.valueOf(amount.toString())) {
            results.add(binding); // i.e., true
          }
        } else {
          log.error("Malformed state has free variable in place of numeric amount: " + state);
        }
        break;
      }
      case "geq":
      case "fluent_geq": {
        Symbol amount = state.get(1);
        if (!amount.isVariable()) {
          if (worldAmount >= Double.valueOf(amount.toString())) {
            results.add(binding); // i.e., true
          }
        } else {
          log.error("Malformed state has free variable in place of numeric amount: " + state);
        }
        break;
      }
    }
    log.debug("results " + results);
    return results;
  }

  /**
   * Get last known belief state amount for "(fluent_)equals(someValueHere, Y)"
   *
   * @param equalsQuery
   * @return
   */
  static private double getBeliefStateAmount(Predicate equalsQuery) {
    try {
      List<Map<Variable, Symbol>> queryResults = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief").argTypes(Predicate.class)).call(List.class, equalsQuery);
      if (queryResults.isEmpty()) {
        return 0;
      } else if (queryResults.size() > 1) {
        log.warn("[getBeliefStateAmount] Found multiple results for belief query: " + equalsQuery);
      }

      Variable amountVar = (Variable) equalsQuery.get(1);

      // if no results from belief state, assume zero amount
      if (queryResults.isEmpty()) {
        return 0;
      } else {
        return Double.parseDouble(queryResults.get(0).get(amountVar).getName());
      }
    } catch (TRADEException e) {
      log.error("[getBeliefStateAmount] Error querying belief.");
      return 0;
    }
  }

  /**
   * Get game/world state amount for "(fluent_)equals(someValueHere, Y)"
   *
   * @param equalsQuery
   * @return
   */
  static private double getWorldStateAmount(Symbol actor, Predicate equalsQuery) {
    // NOTE: this is a little unorthodox, but having GM execute an observe action will automatically add the
    //       observation to belief, which we don't want in this cae. For instance, if this observation is for
    //       an increase(something,3) observation, the equals(something,X) observation would update belief with
    //       the new equals value, and then increase observation will also update belief, causing a double count

    // get TRADE service that can observe equalsQuery predicate
    List<ActionDBEntry> observers;
    try {
      observers = TRADE.getAvailableService(new TRADEServiceConstraints().name("getObservers").argTypes(Symbol.class,Predicate.class)).call(List.class, actor, equalsQuery);
    } catch (TRADEException e) {
      log.error("Error getting observers for: " + equalsQuery + " for actor: " + actor, e);
      return 0;
    }

    if (observers.isEmpty()) {
      log.debug("No observers found for: " + equalsQuery + " for actor: " + actor + ". Querying Belief instead.");
      return getBeliefStateAmount(equalsQuery);
    } else if (observers.size() > 1) {
      log.error("More than one observer found for: " + equalsQuery + " for actor: " + actor);
    }
    ActionDBEntry observer = observers.get(0);
    TRADEServiceInfo service = observer.getServiceInfo();

    // call service to make observation
    List<Map<Variable, Symbol>> observationResults = null;
    try {
      if (service.serviceParameterNames.length > 1) {
        observationResults = service.call(List.class, actor, equalsQuery);
      } else {
        observationResults = service.call(List.class,service, equalsQuery);
      }
    } catch (TRADEException e) {
      log.error("Error calling Observer service while trying to observe: " + equalsQuery);
    }

    // get results and return numeric value
    if (observationResults == null || observationResults.size() > 1) {
      log.error("Unexpected observation results. Returning 0 for query: " + equalsQuery);
      return 0;
    } else if (observationResults.isEmpty()) {
      return 0;
    }
    String value = observationResults.get(0).get(equalsQuery.get(1)).getName();
    return Double.parseDouble(value);
  }

}
