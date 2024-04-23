/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.mcts;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.*;
import edu.tufts.hrilab.action.Observable;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.execution.ActionContext;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.execution.ExecutionType;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import com.google.common.collect.Lists;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.apache.commons.lang3.tuple.Pair;

import java.util.*;

public class AgentModel extends DiarcComponent {
    List<ActionDBEntry> availableActions; //this could be actions... or goals O:
    Symbol actor;
    String type; //todo: probably not a string
    Random random = new Random();

    public AgentModel() {
    }

    public AgentModel(Symbol actor, List<ActionDBEntry> actions) {
        this.actor = actor;
        this.availableActions = actions;
    }

    @Override
    protected List<Option> additionalUsageInfo() {
        List<Option> options = new ArrayList<>();
        options.add(Option.builder("agent").hasArg().argName("name").desc("Name of the simulated agent").build());
        return options;
    }

    @Override
    protected void parseArgs(CommandLine cmdLine) {
        super.parseArgs(cmdLine);
        if (cmdLine.hasOption("agent")) {
            this.actor = Factory.createSymbol(cmdLine.getOptionValue("agent"));
        }
    }

    //todo: add agent models from polycraft
    @TRADEService
    public Symbol getActor() {
        return actor;
    }

    @TRADEService
    public String getAgentType() {
        return type;
    }

    @TRADEService
    public List<ActionDBEntry> getAvailableActions() {
        return availableActions;
    }

    public Context getLikelyBoundAction(Context c) {
        for (ActionDBEntry action : availableActions) {
            List<List<Pair<String, Symbol>>> possibleBindings = MCTSUtils.populateBindings(action.getInputRoles());
            List<List<Pair<String, Symbol>>> permutations = Lists.cartesianProduct(possibleBindings);
            //todo: need to add agents in the system somewhere...
            for (List<Pair<String, Symbol>> permutation : permutations) {
                Map<String, Object> bindings = new HashMap<>();
                bindings.put("?actor", actor);
                for (Pair<String, Symbol> binding : permutation) {
                    bindings.put(binding.getKey(), binding.getValue());
                }
                Context child = new ActionContext(c, new StateMachine(c.getStateMachine(), false),
                        action, bindings, ExecutionType.SIMULATE_ACT, actor);
                if (child.isApproved().getValue()) {
                    return child;
                }
            }
        }

        return null;
    }

    public Context getTestingBoundAction(Context c) {
        Map<String, Object> bindings = new HashMap<>();
        ActionDBEntry action;

        if(actor.toString().equals("rival")) {
            action = Database.getActionDB().getAction("mctsbreak_and_pickup_block_of_platinum");
            bindings.put("?actor", actor);
            bindings.put("?obj", "block_of_platinum");

        } else if(actor.toString().equals("rival2")) {
            action = Database.getActionDB().getAction("mctsbreak_and_pickup");
            bindings.put("?actor", actor);
            bindings.put("?obj", "oak_log");
//            action = Database.getInstance().getAction("mctsbreak_and_pickup_block_of_platinum");
//            bindings.put("?actor", actor);
//            bindings.put("?obj", "block_of_platinum");

        } else {
            log.error("Unknown rival");
            return null;
        }

        Context child = new ActionContext(c, new StateMachine(c.getStateMachine(), false),
                action, bindings, ExecutionType.SIMULATE_ACT, actor);
        if (child.isApproved().getValue()) {
            return child;
        }
        return null;
    }

    public Context executeLikelyAction(Context c, ActionConstraints constraints) {
//        Context approvedContext = getLikelyBoundAction(c);
        Context approvedContext = getTestingBoundAction(c);
        if(approvedContext == null) {
            return c;
        }
        Goal goal = new Goal(actor, new Predicate("rollout", new String[0]),
                Observable.FALSE);
        ActionInterpreter.createInterpreterFromContext(goal, approvedContext).call(); //execute rival action
        return approvedContext;
    }
}
