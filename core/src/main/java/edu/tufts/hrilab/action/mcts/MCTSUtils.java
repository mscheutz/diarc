/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.mcts;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Variable;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class MCTSUtils {

    //fixme: if anyone wants to figure out a better way to have a cartesian product of a map, feel free :)
    public static List<List<Pair<String, Symbol>>> populateBindings(List<ActionBinding> inputs) {
        List<List<Pair<String, Symbol>>> possibleBindings = new ArrayList<>();

        for (ActionBinding input : inputs) {
            //ignore the actor for now (always self for now)
            if (input.getName().equals("?actor")) {
                continue;
            }

            //Create a list for all the possible bindings for one input
            //Pairs are used to maintain input name for a given possible binding
            List<Pair<String, Symbol>> bindingList = new ArrayList<>();

            if (!input.isBound() && input.getJavaType().isAssignableFrom(Symbol.class)) {
                try {
                    //Get all potential objects via the input type
                    Predicate query = Factory.createPredicate("typeobject", "X", input.getSemanticType());
                    List<Map<Variable, Symbol>> bindings = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief").argTypes(Predicate.class)).call(List.class,query);
                    for (Map<Variable, Symbol> binding : bindings) {
                        Symbol s = binding.get(new Variable("X"));
                        Pair<String, Symbol> objBindings = new ImmutablePair<>(input.getName(), s);
                        bindingList.add(objBindings);
                    }

                    //Add any constants that match the input type (todo: ideally these should be in typeobject?)
                    query = Factory.createPredicate("constant", "X", input.getSemanticType());
                    bindings = TRADE.getAvailableService(new TRADEServiceConstraints().name("queryBelief").argTypes(Predicate.class)).call(List.class, query);
                    for (Map<Variable, Symbol> binding : bindings) {
                        Symbol s = binding.get(new Variable("X"));
                        Pair<String, Symbol> objBindings = new ImmutablePair<>(input.getName(), s);
                        bindingList.add(objBindings);
                    }

                    //Add all the bindings for a specific input
                    possibleBindings.add(bindingList);
                } catch (TRADEException e) {
                    throw new RuntimeException(e);
                }
            }
        }
        return possibleBindings;
    }


}
