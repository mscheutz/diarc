/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.state;

import edu.tufts.hrilab.action.util.Utilities;
import edu.tufts.hrilab.fol.*;
import edu.tufts.hrilab.pddl.Operators;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.List;
import java.util.Map;

public class SMOperatorCheck {
    private static final Logger log = LoggerFactory.getLogger(SMOperatorCheck.class);

    public static boolean isOperator(Term state) {
        return Operators.DIARC_TO_PDDL.containsKey(state.getName());
    }

    public static boolean assertFluent(Predicate belief, State state) {
        Symbol firstArg = belief.get(0);
        Symbol secondArg = belief.get(1);
        if (secondArg.isTerm()) {
            // second arg is a function, find its numeric value
            Double value = getNumericValue(secondArg, state);

            // update belief with numeric value
            belief = Factory.createPredicate(belief.getName(), firstArg, Factory.createSymbol(value.toString()));
        }

        switch (belief.getName()) {
            case "equals":
            case "fluent_equals": {
                removeFluentEquality(belief, state);
                log.debug("fluent assertion: " + belief);
                return state.addFact(belief, false);
            }
            case "increase":
            case "fluent_increase": {
                double old_val = removeFluentEquality(belief, state);
                double new_val = old_val + Double.parseDouble(belief.get(1).getName());
                Predicate new_belief = makeFluentEquality(belief.get(0), new_val);
                log.debug("fluent increase: " + new_belief);
                return state.addFact(new_belief, false);
            }
            case "decrease":
            case "fluent_decrease": {
                double old_val = removeFluentEquality(belief, state);
                double new_val = old_val - Double.parseDouble(belief.get(1).getName());
                Predicate new_belief = makeFluentEquality(belief.get(0), new_val);
                log.debug("fluent decrease: " + new_belief);
                return state.addFact(new_belief, false);
            }
            default:
                log.debug("Ignoring operator assertion. Asserting it anyways: " + belief);
                return state.addFact(belief, false);
        }
    }

    private static double removeFluentEquality(Predicate fluent, State state) {
        Variable var = Factory.createVariable("X");
        Predicate query = Factory.createPredicate("fluent_equals", fluent.get(0), var);

        List<Map<Variable, Symbol>> result = state.queryFluentValue(query);
        if (result.isEmpty()) {
            return 0;
        } else if (result.size() > 1) {
            log.warn("[removeFluentEquality] More than one value exists for query: " + query);
        }

        Symbol oldValue = result.get(0).get(var);
        Predicate toRemove = Factory.createPredicate("fluent_equals",fluent.get(0), oldValue);
        state.removeFact(toRemove);

        return Double.parseDouble(oldValue.getName());
    }

    private static Predicate makeFluentEquality(Symbol function, double value) {
        return Factory.createPredicate("fluent_equals", function, Factory.createSymbol(Double.toString(value)));
    }

    public static boolean query(Predicate fluentQuery, State state) {
        log.debug("fluent query: " + fluentQuery);
        if (fluentQuery.size() != 2) {
            log.error("Querying with operator that does not have 2 args: " + fluentQuery);
            return false;
        }

        Symbol firstArg = fluentQuery.get(0);
        Symbol secondArg = fluentQuery.get(1);
        switch (fluentQuery.getName()) {
            case "equals": {
                // TODO: merge this case with fluent_equals case
                return firstArg.equals(secondArg);
            }
            case "fluent_equals": {
                log.debug("fluent query: " + fluentQuery);
                if (firstArg.equals(secondArg)) {
                    return true;
                } else {
                    // can't assume this case can be handled by directly querying belief. e.g., equals(mul(a,b),c)
                    boolean firstArgHasVars = (firstArg.isTerm() && !((Term) firstArg).getVars().isEmpty()) || firstArg.isVariable();
                    boolean secondArgHasVars = (secondArg.isTerm() && !((Term) secondArg).getVars().isEmpty()) || secondArg.isVariable();

                    // can't currently handle the case where free-variables exist within operators
                    if (firstArgHasVars && (firstArg.isTerm() && isOperator((Term) firstArg)) ||
                            (secondArgHasVars && secondArg.isTerm() && isOperator((Term) secondArg))) {
                        log.warn("Cannot handle free-variables in operators: " + fluentQuery);
                        return false;
                    }

                    if (firstArgHasVars && secondArgHasVars) {
                        // query belief directly
                        return state.hasFact(fluentQuery);
                    } else if (firstArgHasVars) {
                        // get value for second arg and then query belief
                        Double value = getNumericValue(secondArg, state);
                        Predicate query = Factory.createPredicate("fluent_equals", firstArg, Factory.createSymbol(value.toString()));
                        return state.hasFact(query);
                    } else if (secondArgHasVars) {
                        // second arg must be a Variable (otherwise would have to be a numeric operator, which is caught by check above)
                        // get value for first arg and assign it to second arg variable
                        if (!secondArg.isVariable()) {
                            log.warn("Cannot handle free-variables in second arg: " + fluentQuery);
                            return false;
                        }
                        log.error("Cannot handle vars in State: " + fluentQuery);
                        return false;
                    } else {
                        // no free-variables in fluentQuery
                        Double firstValue = getNumericValue(firstArg, state);
                        Double secondValue = getNumericValue(secondArg, state);
                        return firstValue != null && firstValue.equals(secondValue);
                    }
                }
            }
            case "leq":
            case "fluent_leq": {
                // TODO: allow the first value to contain free-variables
                Double firstValue = getNumericValue(firstArg, state);
                Double secondValue = getNumericValue(secondArg, state);
                return firstValue != null && secondValue != null && firstValue <= secondValue;
            }
            case "geq":
            case "fluent_geq": {
                // TODO: allow the first value to contain free-variables
                Double firstValue = getNumericValue(firstArg, state);
                Double secondValue = getNumericValue(secondArg, state);
                return firstValue != null && secondValue != null && firstValue >= secondValue;
            }
            case "less":
            case "fluent_less": {
                // TODO: allow the first value to contain free-variables
                Double firstValue = getNumericValue(firstArg, state);
                Double secondValue = getNumericValue(secondArg, state);
                return firstValue != null && secondValue != null && firstValue < secondValue;
            }
            case "greater":
            case "fluent_greater": {
                // TODO: allow the first value to contain free-variables
                Double firstValue = getNumericValue(firstArg, state);
                Double secondValue = getNumericValue(secondArg, state);
                return firstValue != null && secondValue != null && firstValue > secondValue;
            }
            default:
                log.warn("Cannot query operator: " + fluentQuery);
                return false;
        }
    }

    private static Double getNumericValue(Symbol function, State state) {
        if (!function.isTerm()) {
            if (Utilities.isNumeric(function)) {
                // already be a numeric value
                return Double.parseDouble(function.getName());
            } else if (function.isVariable()) {
                log.warn("Cannot get numeric value of a Variable: " + function);
                return null;
            }
            // must be a zero arity term/function
        }

        // recursively get values of arguments
        switch (function.getName()) {
            case "mul":
            case "div":
            case "add":
            case "sub":
                return evaluateArithmeticOperator((Term) function, state);
            default:
                // assume it's a pddl style function -- query belief for its value
                Variable valueVar = Factory.createVariable("V");
                Predicate query = Factory.createPredicate("fluent_equals", function, valueVar);
                List<Map<Variable, Symbol>> queryResults = state.queryFluentValue(query);
                if (queryResults.isEmpty()) {
                    // no results --> assume zero, make it explicit (by asserting it to belief), and return 0
                    Predicate zeroCount = Factory.createPredicate("fluent_equals", function, Factory.createSymbol("0"));
                    state.addFact(zeroCount, false);
                    return (double) 0;
                } else if (queryResults.size() > 1) {
                    log.warn("[getNumericValue] More than one value exists for query: " + query);
                }
                return Double.parseDouble(queryResults.get(0).get(valueVar).getName());
        }
    }

    private static Double evaluateArithmeticOperator(Term function, State state) {
        // recursively get values of arguments
        Double firstArg;
        Double secondArg;
        if (function.get(0).isTerm()) {
            firstArg = getNumericValue(function.get(0), state);
        } else {
            firstArg = Double.parseDouble(function.get(0).getName());
        }
        if (function.get(1).isTerm()) {
            secondArg = getNumericValue(function.get(1), state);
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

}
