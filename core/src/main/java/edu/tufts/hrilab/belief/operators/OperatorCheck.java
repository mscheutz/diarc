/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.belief.operators;

import edu.tufts.hrilab.action.util.Utilities;
import edu.tufts.hrilab.belief.BeliefComponent;
import edu.tufts.hrilab.belief.common.MemoryLevel;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.pddl.Operators;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * This class contains all the numeric operator logic that is not possible in prolog (prolog is bad with numbers,
 * especially non-integers). This serves as a wrapper that sits between the BeliefComponent and Prolog instance.
 */
public class OperatorCheck {
  private static Logger log = LoggerFactory.getLogger(OperatorCheck.class);

  public static boolean isOperator(Term state) {
    if (Operators.DIARC_TO_PDDL.containsKey(state.getName())) {
      return true;
    }
    return false;
  }

  public static void assertFluent(Term belief, MemoryLevel level, BeliefComponent beliefComponent) {
    Symbol firstArg = belief.get(0);
    Symbol secondArg = belief.get(1);
    if (secondArg.isTerm()) {
      // second arg is a function, find its numeric value
      Double value = getNumericValue(secondArg, level, beliefComponent);

      // update belief with numeric value
      belief = Factory.createPredicate(belief.getName(), firstArg, Factory.createSymbol(value.toString()));
    }

    switch (belief.getName()) {
      case "equals":
      case "fluent_equals": {
        level = getCorrectedMemoryLevel(belief, level, beliefComponent);
        removeFluentEquality(belief, level, beliefComponent);
        log.debug("fluent assertion: " + belief);
        beliefComponent.assertBelief(belief, level, false);
        break;
      }
      case "increase":
      case "fluent_increase": {
        level = getCorrectedMemoryLevel(belief, level, beliefComponent);
        double old_val = removeFluentEquality(belief, level, beliefComponent);
        double new_val = old_val + Double.parseDouble(belief.get(1).getName());
        Term new_belief = makeFluentEquality(belief.get(0), new_val);
        log.debug("fluent increase: " + new_belief);
        beliefComponent.assertBelief(new_belief, level, false);
        break;
      }
      case "decrease":
      case "fluent_decrease": {
        level = getCorrectedMemoryLevel(belief, level, beliefComponent);
        double old_val = removeFluentEquality(belief, level, beliefComponent);
        double new_val = old_val - Double.parseDouble(belief.get(1).getName());
        Term new_belief = makeFluentEquality(belief.get(0), new_val);
        log.debug("fluent decrease: " + new_belief);
        beliefComponent.assertBelief(new_belief, level, false);
        break;
      }
      default:
        log.debug("Ignoring operator assertion. Asserting it anyways: " + belief);
        beliefComponent.assertBelief(belief, level, false);
    }
  }

  public static List<Map<Variable, Symbol>> query(Term fluentQuery, MemoryLevel level, BeliefComponent beliefComponent) {
    log.debug("fluent query: " + fluentQuery);
    List<Map<Variable, Symbol>> results = new ArrayList<>();
    if (fluentQuery.size() != 2) {
      log.error("Querying with operator that does not have 2 args: " + fluentQuery);
      return results;
    }

    Symbol firstArg = fluentQuery.get(0);
    Symbol secondArg = fluentQuery.get(1);
    switch (fluentQuery.getName()) {
      case "equals": {
        // TODO: merge this case with fluent_equals case
        if (firstArg.equals(secondArg)) {
          // i.e., true
          results.add(new HashMap<>());
        }
        return results;
      }
      case "fluent_equals": {
        log.debug("fluent query: " + fluentQuery);
        if (firstArg.equals(secondArg)) {
          // i.e., true
          results.add(new HashMap<>());
          return results;
        } else {
          // can't assume this case can be handled by directly querying belief. e.g., equals(mul(a,b),c)
          boolean firstArgHasVars = (firstArg.isTerm() && !((Term) firstArg).getVars().isEmpty()) || firstArg.isVariable();
          boolean secondArgHasVars = (secondArg.isTerm() && !((Term) secondArg).getVars().isEmpty()) || secondArg.isVariable();

          // can't currently handle the case where free-variables exist within operators
          if (firstArgHasVars && (firstArg.isTerm() && isOperator((Term)firstArg)) ||
                  (secondArgHasVars && secondArg.isTerm() && isOperator((Term)secondArg))) {
            log.warn("Cannot handle free-variables in operators: " + fluentQuery);
            return results;
          }

          if (firstArgHasVars && secondArgHasVars) {
            // query belief directly
            return beliefComponent.queryBelief(fluentQuery, level, false);
          } else if (firstArgHasVars) {
            // get value for second arg and then query belief
            Double value = getNumericValue(secondArg, level, beliefComponent);
            Predicate query = Factory.createPredicate("fluent_equals", firstArg, Factory.createSymbol(value.toString()));
            return beliefComponent.queryBelief(query, level, false);
          } else if (secondArgHasVars) {
            // second arg must be a Variable (otherwise would have to be a numeric operator, which is caught by check above)
            // get value for first arg and assign it to second arg variable
            if (!secondArg.isVariable()) {
              log.warn("Cannot handle free-variables in second arg: " + fluentQuery);
              return results;
            }
            Double value = getNumericValue(firstArg, level, beliefComponent);
            Map<Variable, Symbol> result = new HashMap<>();
            result.put((Variable) secondArg, Factory.createSymbol(value.toString()));
            results.add(result);
            return results;
          } else {
            // no free-variables in fluentQuery
            Double firstValue = getNumericValue(firstArg, level, beliefComponent);
            Double secondValue = getNumericValue(secondArg, level, beliefComponent);
            if (firstValue != null && secondValue != null && firstValue.equals(secondValue)) {
              results.add(new HashMap<>());
            }
            return results;
          }
        }
      }
      case "leq":
      case "fluent_leq": {
        // TODO: allow the first value to contain free-variables
        Double firstValue = getNumericValue(firstArg, level, beliefComponent);
        Double secondValue = getNumericValue(secondArg, level, beliefComponent);
        if (firstValue != null && secondValue != null && firstValue <= secondValue) {
          results.add(new HashMap<>());
        }
        return results;
      }
      case "geq":
      case "fluent_geq": {
        // TODO: allow the first value to contain free-variables
        Double firstValue = getNumericValue(firstArg, level, beliefComponent);
        Double secondValue = getNumericValue(secondArg, level, beliefComponent);
        if (firstValue != null && secondValue != null && firstValue >= secondValue) {
          results.add(new HashMap<>());
        }
        return results;
      }
      case "less":
      case "fluent_less": {
        // TODO: allow the first value to contain free-variables
        Double firstValue = getNumericValue(firstArg, level, beliefComponent);
        Double secondValue = getNumericValue(secondArg, level, beliefComponent);
        if (firstValue != null && secondValue != null && firstValue < secondValue) {
          results.add(new HashMap<>());
        }
        return results;
      }
      case "greater":
      case "fluent_greater": {
        // TODO: allow the first value to contain free-variables
        Double firstValue = getNumericValue(firstArg, level, beliefComponent);
        Double secondValue = getNumericValue(secondArg, level, beliefComponent);
        if (firstValue != null && secondValue != null && firstValue > secondValue) {
          results.add(new HashMap<>());
        }
        return results;
      }
      default:
        log.warn("Cannot query operator: " + fluentQuery);
        return new ArrayList<>();
    }
  }

  /**
   * Helper method to get the MemoryLevel containing the specified query term. This is needed to know at which memory
   * level a fluent should be updated.
   *
   * @param query
   * @param defaultLevel    level returned if query term is not found at any level
   * @param beliefComponent
   * @return
   */
  private static MemoryLevel getCorrectedMemoryLevel(Term query, MemoryLevel defaultLevel, BeliefComponent beliefComponent) {
    // only fluent_equals are explicitly stored in belief, so generate appropriate query to check level
    query = Factory.createPredicate("fluent_equals", query.get(0), Factory.createVariable("V"));

    if (beliefComponent.querySupport(query, MemoryLevel.UNIVERSAL, false)) {
      return MemoryLevel.UNIVERSAL;
    } else if (beliefComponent.querySupport(query, MemoryLevel.EPISODIC, false)) {
      return MemoryLevel.EPISODIC;
    } else if (beliefComponent.querySupport(query, MemoryLevel.WORKING, false)) {
      return MemoryLevel.WORKING;
    } else {
      // return level that was passed in
      log.debug("Query has no support at any memory level: " + query);
      return defaultLevel;
    }
  }

  private static double removeFluentEquality(Term fluent, MemoryLevel level, BeliefComponent beliefComponent) {
    Variable var = Factory.createVariable("X");
    Predicate query = Factory.createPredicate("fluent_equals", fluent.get(0), var);

    List<Map<Variable, Symbol>> result = beliefComponent.queryBelief(query, level, false);
    if (result.isEmpty()) {
      return 0;
    } else if (result.size() > 1) {
      log.warn("[removeFluentEquality] More than one value exists for query: " + query);
    }

    Symbol oldValue = result.get(0).get(var);
    beliefComponent.retractBelief(query, level);

    return Double.parseDouble(oldValue.getName());
  }

  private static Double getNumericValue(Symbol function, MemoryLevel level, BeliefComponent beliefComponent) {
    if (!function.isTerm()) {
      if (Utilities.isNumeric(function)) {
        // already be a numeric value
        return Double.parseDouble(function.getName());
      } else if (function.isVariable()) {
        log.warn("Cannot get numeric value of a Variable: " + function);
        return null;
      } else {
        // must be a zero arity term/function
      }
    }

    // recursively get values of arguments
    switch (function.getName()) {
      case "mul":
      case "div":
      case "add":
      case "sub":
        return evaluateArithmeticOperator((Term) function, level, beliefComponent);
      default:
        // assume it's a pddl style function -- query belief for its value
        Variable valueVar = Factory.createVariable("V");
        Predicate query = Factory.createPredicate("fluent_equals", function, valueVar);
        List<Map<Variable, Symbol>> queryResults = beliefComponent.queryBelief(query, level, false);
        if (queryResults.isEmpty()) {
          // no results --> assume zero, make it explicit (by asserting it to belief), and return 0
          Predicate zeroCount = Factory.createPredicate("fluent_equals", function, Factory.createSymbol("0"));
          beliefComponent.assertBelief(zeroCount, level, false);
          return 0.0;
        } else if (queryResults.size() > 1) {
          log.warn("[getNumericValue] More than one value exists for query: " + query);
        }
        return Double.parseDouble(queryResults.get(0).get(valueVar).getName());
    }
  }

  private static Double evaluateArithmeticOperator(Term function, MemoryLevel level, BeliefComponent beliefComponent) {
    // recursively get values of arguments
    Double firstArg;
    Double secondArg;
    if (function.get(0).isTerm()) {
      firstArg = getNumericValue(function.get(0), level, beliefComponent);
    } else {
      firstArg = Double.parseDouble(function.get(0).getName());
    }
    if (function.get(1).isTerm()) {
      secondArg = getNumericValue(function.get(1), level, beliefComponent);
    } else {
      secondArg = Double.parseDouble(function.get(1).getName());
    }

    switch (function.getName()) {
      case "mul":
        return firstArg * secondArg;
      case "div":
        return firstArg / secondArg;
      case "add":
        return firstArg + secondArg;
      case "sub":
        return firstArg - secondArg;
      default:
        log.warn("Cannot handle arithmetic operator: " + function);
        return null;
    }
  }

  /**
   * Make a fluent_equals(function, value) predicate.
   *
   * @param function
   * @param value
   * @return
   */
  private static Term makeFluentEquality(Symbol function, double value) {
    return Factory.createPredicate("fluent_equals", function, Factory.createSymbol(Double.toString(value)));
  }
}
