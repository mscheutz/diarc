/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.mcts;

import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.execution.ActionContext;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.execution.ExecutionType;
import edu.tufts.hrilab.action.execution.GoalContext;
import edu.tufts.hrilab.fol.Symbol;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;

/**
 * Custom implementation of an MCTS tree node. Each node represents a bound action to be taken,
 * with both arguments and actor.
 */
public class MCTSNode {

    private static final Logger log = LoggerFactory.getLogger(MCTSNode.class);

    MCTSNode parent;
    List<MCTSNode> children;
    Context context; //this contains both state and operator? State might be volatile tho
    Symbol agent; //todo: definitely need a better way to do this. We want each level to be the same agent
    boolean expanded;

    double score;
    int visits;

    ActionDBEntry entry;
    Map<String, Object> bindings;

    /**
     * Create a pre-bound MCTS node.
     * @param entry
     * @param bindings
     * @param parent
     * @param agent
     */
    public MCTSNode(ActionDBEntry entry, Map<String, Object> bindings, MCTSNode parent, Symbol agent) {
        this.parent = parent;
        this.children = new ArrayList<>();
        this.score = 0;
        this.visits = 0;
        this.entry = entry;
        this.bindings = bindings;
        this.agent = agent;
        this.expanded = false;
    }

    /**
     * Create an unbound MCTS node. Only for creation of the root node.
     * @param context
     * @param parent
     * @param agent
     */
    public MCTSNode(Context context, MCTSNode parent, Symbol agent) {
        if(!(context instanceof GoalContext)) {
            log.error("Possible illegal creation of MCTS node from non goal context");
        }
        this.parent = parent;
        this.children = new ArrayList<>();
        this.score = 0;
        this.visits = 0;
        this.context = context;
        this.agent = agent;
        this.expanded = false;
    }

    public void generateContext() {
        this.context = new ActionContext(parent.getContext(), new StateMachine(parent.getContext().getStateMachine(), false),
                entry, bindings, ExecutionType.SIMULATE_ACT, agent);
    }

    public Symbol getAgent() {
        return agent;
    }

    public int getVisits() {
        return visits;
    }

    public void setVisits(int visits) {
        this.visits = visits;
    }

    public double getScore() {
        return score;
    }

    public void setScore(double score) {
        this.score = score;
    }

    /**
     * Calculates the value of the node (check out some MCTS tutorials for more details)
     * @return
     */
    private double valueUCT() {
        if(visits == 0) {
            return 0; //This can be changed for more sophisticated behavior
        }
        return score / ((double) visits) + Math.sqrt(2. * Math.log(parent.visits) / visits);

    }

    /**
     * returns the child with the highest UTC
     * @return
     */
    public MCTSNode getBestChild() {
        if (!expanded) { //also check if any children are unexpanded?
            return this;
        }
        MCTSNode bestChild = children.stream()
                .filter(Objects::nonNull)
                .max(Comparator.comparingDouble(MCTSNode::valueUCT)).get();
        return bestChild.getBestChild();
    }

    /**
     * Propogates the value from this node up to its parent
     * @param value
     */
    public void backup(int value) {
        this.score += value;
        this.visits++;
        if (this.parent != null) {
            this.parent.backup(value);
        }
    }

    public String toActionDesc() {
        return context.getSignatureInPredicateForm().toString();
    }

    public boolean isExpanded() {
        return expanded;
    }

    public List<MCTSNode> getChildren() {
        return children;
    }

    public void addChild(MCTSNode newNode) {
        children.add(newNode);
    }

    public void addChildren(List<MCTSNode> potentialChildren) {
        children.addAll(potentialChildren);
    }

    public Context getContext() {
        return context;
    }

    public void setContext(Context context) {
        this.context = context;
    }

    public MCTSNode getParent() {
        return parent;
    }

    public void setExpanded(boolean expanded) {
        this.expanded = true;
    }
}
