/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.supermarket;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.Observable;
import edu.tufts.hrilab.action.execution.ChildContexts;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.execution.GoalContext;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.rollout.RolloutActionInterpreter;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Term;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.*;
import java.util.function.ToDoubleFunction;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class SimulationBasedPlanning extends DiarcComponent {

    protected String agentName;
    protected String[] availableActions;
    protected int budget;

    @Override
    protected List<Option> additionalUsageInfo() {
        List<Option> options = new ArrayList<>();
        options.add(Option.builder("planningAgent").hasArg().argName("name").desc("Name of the simulated agent").build());
        options.add(Option.builder("availableActions").hasArgs().argName("actions").desc("names of actions available to the agent").build());
        options.add(Option.builder("budget").hasArgs().type(Integer.class).argName("budget").desc("Computational budget for MCTS").build());
        return options;
    }

    @Override
    protected void init() {
        super.init();
        this.budget = 50;
    }

    @Override
    protected void parseArgs(CommandLine cmdLine) {
        super.parseArgs(cmdLine);
        if(cmdLine.hasOption("planningAgent")) {
            this.agentName = cmdLine.getOptionValue("planningAgent");
        }
        if(cmdLine.hasOption("availableActions")) {
            this.availableActions = cmdLine.getOptionValues("availableActions");
        }
        if(cmdLine.hasOption("budget")) {
            this.budget = Integer.parseInt(cmdLine.getOptionValue("budget"));
        }
    }

    public void doGoal(String goal) {
        try {
            long goalID = TRADE.getAvailableService(new TRADEServiceConstraints().name("submitGoal").argTypes(Predicate.class)).call(Long.class, Factory.createPredicate(goal));
            GoalStatus status = TRADE.getAvailableService(new TRADEServiceConstraints().name("joinOnGoal").argTypes(Long.class)).call(GoalStatus.class, goalID);
        } catch (TRADEException e) {
            log.error("[doGoal]",e);
        }
    }

    protected Context getCurrentContext(Context base) {
        Context curr = base;
        while(true) {
            ChildContexts childContexts = curr.getChildContexts();
            if(childContexts.isEmpty()) {
                break;
            }
            Context next = childContexts.getCurrent();
            if(!next.getStatus().isFailure()) {
                break;
            }
            curr = next;
        }
        return curr;
    }

    public class MCTSStateNode {
        public SupermarketObservation obs;
        public double q = 0.;
        public int n = 0;
        public MCTSActionNode[] children;
        public MCTSActionNode parent;

        public MCTSStateNode(MCTSActionNode parent, SupermarketObservation obs, int numActions) {
            this.parent = parent;
            this.obs = obs;
            this.children = new MCTSActionNode[numActions];
        }

        public boolean needsExpansion() {
            return Arrays.stream(this.children).anyMatch(Objects::isNull);
        }

        public boolean empty() {
            return Arrays.stream(this.children).allMatch(Objects::isNull);
        }

        public List<Integer> unexpandedNodes() {
            return IntStream.range(0, this.children.length).filter(i -> this.children[i] == null)
                    .boxed().collect(Collectors.toList());
        }

        public boolean equals(Object other) {
            return obs.equals(((MCTSStateNode)other).obs);
        }

        public int hashCode() {
            return obs.hashCode();
        }

        public void backup(double q) {
            this.q += q;
            this.n++;
            if(this.parent != null) {
                this.parent.backup(q);
            }
        }
    }

    public class MCTSActionNode {
        public Map<SupermarketObservation, MCTSStateNode> children;
        public MCTSStateNode parent;

        public int action;
        public double q = 0.;
        public int n = 0;

        public MCTSActionNode(MCTSStateNode parent, int action) {
            this.parent = parent;
            this.action = action;
            this.children = new HashMap<>();
        }

        public double valueUCT() {
            return q/((double)n) + Math.sqrt(2.*Math.log(parent.n)/n);
        }

        public double qValue() { return q/((double)n); }

        public void backup(double q) {
            this.q += q;
            this.n++;
            this.parent.backup(q);
        }
    }

    public MCTSStateNode traverse(MCTSStateNode node) {
        return traverse(node, MCTSActionNode::valueUCT,
                new TRADEServiceConstraints().inGroups("agent:" + this.agentName), false);
    }

    public MCTSStateNode traverse(MCTSStateNode node, ToDoubleFunction<MCTSActionNode> actionUtility,
                                  TRADEServiceConstraints constraints, boolean executionMode) {
        if((node.needsExpansion() && !executionMode) || node.empty()) {
            return node;
        }

        MCTSActionNode bestChild = Arrays.stream(node.children)
                .filter(Objects::nonNull)
                .max(Comparator.comparingDouble(actionUtility)).get();

        String nextAction = this.availableActions[bestChild.action];
//        if(executionMode) {
            log.error("MCTS EXECUTING: ACTION " + nextAction);
//        }
        SupermarketObservation nextObs = null;
        try {
            //TODO:brad: does any of this work?
            TRADE.getAvailableService(constraints.name(nextAction)).call(Object.class);
            nextObs = TRADE.getAvailableService(constraints.name("getLastObservation")).call(SupermarketObservation.class);
        } catch(Exception e) {
            log.error("[traverse]",e);
        }
        if(bestChild.children.containsKey(nextObs)) {
            return traverse(bestChild.children.get(nextObs), actionUtility, constraints, executionMode);
        } else {
            MCTSStateNode newNode = new MCTSStateNode(bestChild, nextObs, this.availableActions.length);
            bestChild.children.put(nextObs, newNode);
            return newNode; // newNode is definitely unexpanded.
        }
    }


//    TODO(dkasenberg) replace with an actual planning simulator
    @TRADEService
    public void getLowLevelPlan(GoalContext goalContext) {
        TRADEServiceConstraints constraintsPlanning = new TRADEServiceConstraints().inGroups("agent:" + this.agentName);
        TRADEServiceConstraints constraintsExecution = new TRADEServiceConstraints().inGroups(this.getMyGroups().get(0));

//        The idea should be:
//        (1) keep track of the agent's high-level plan (because it may need to modify bits and pieces of it, and for
//            the sake of preserving it as much as possible
//        (2) roll out the agent's intended low-level actions to a particular horizon
//        (3) figure out if the LLA is predicted to violate any norms
//        (4) if it would, start doing MCTS on the LLAs.
//          - What's the rollout policy?
//            -> if conditions are still met for the HLA, the rollout should be the LLAs corresponding to the remainder
//               of the plan
//            -> we can either write off rollouts that lose HLA conditions entirely, or we can do some form of replanning.
//        Initial decision: assume the conditions will still be met
//        We might need custom code to perform the first N primitives of a particular (non-primitive) action
//        We will probably also need custom instrumentation code to evaluate the rollouts allowing us to look at the observation
//        after each primitive.


        try {
            SupermarketObservation obs = TRADE.getAvailableService(constraintsExecution.name("getLastObservation")).call(SupermarketObservation.class);

            Random random = new Random();

            MCTSStateNode root = new MCTSStateNode(null, obs, this.availableActions.length);
            boolean foundSolution = false;

            int numRollouts = 0;
            for(int i = 0; i < budget; i++) {
//            while(!foundSolution) {
                TRADE.getAvailableService(constraintsPlanning.name("setObservation")).call(void.class,obs);
                log.error("NEXT ROLLOUT");
                MCTSStateNode leaf = traverse(root);

//                Pick an unexpanded action from leaf
                List<Integer> unexpandedActions = leaf.unexpandedNodes();
                int action = unexpandedActions.get(random.nextInt(unexpandedActions.size()));
                //get VALID ACTIONS. Must have preconds valid!
                String nextAction = this.availableActions[action];
                log.error("FINAL ACTION: " + nextAction);
                SupermarketObservation nextObs = null;
                TRADE.getAvailableService(constraintsPlanning.name(nextAction)).call(Object.class);
                MCTSActionNode actionNode = new MCTSActionNode(leaf, action);
                leaf.children[action] = actionNode;

                Context current = getCurrentContext(goalContext);
                Context newCurrent = current.createSimulatedEquivalent(false);
//                Predicate goalPred = getSignatureInPredicateForm();
//                Goal rolloutGoal = new Goal(getActor("?actor"), goalPred, Observable.FALSE);
                Goal rolloutGoal = new Goal(Factory.createSymbol("agent0:agent"), new Predicate("blah", new String[0]),
                        Observable.FALSE);

                //todo: this is our eval function. we need a cost associated w it (currently -1, 0)
                //-1 not plannable, 0 plannable?
                RolloutActionInterpreter rolloutInterpreter =
                        RolloutActionInterpreter.createInterpreterFromContext(rolloutGoal,
                                newCurrent, 15);

                rolloutInterpreter.call();

//                TODO replace with more sophisticated Q-function involving discounting and weights/priorities
                boolean violatesNorms = TRADE.getAvailableService(new TRADEServiceConstraints().name("querySupport").argTypes(Term.class)).call(Boolean.class,Factory.createPredicate("violated_norms(" + this.agentName + ")"));
                foundSolution = !violatesNorms;

                double q = violatesNorms ? -1. : 0.;

                actionNode.backup(q);
                numRollouts++;

            }
            log.error("TOOK " + numRollouts + " ROLLOUTS");

            log.error("Applying rollouts: ");
//            Perform the best action sequence so far until you hit an unexpected state (at least one action)
            traverse(root, MCTSActionNode::qValue, constraintsExecution, true);

            log.error("FINISHED MCTS");

        } catch(Exception e) {
            log.error("[getLowLevelPlan]",e);
        }
    }
}
