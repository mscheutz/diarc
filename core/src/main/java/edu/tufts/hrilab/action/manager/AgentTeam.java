/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.manager;

import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.learning.ActionLearning;
import edu.tufts.hrilab.action.learning.ActionLearningStatus;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.Future;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class AgentTeam {

    private static final Logger log = LoggerFactory.getLogger(AgentTeam.class);

    private Symbol name;
    private AgentTeam parentTeam;
    private Map<Symbol,AgentTeam> members = new HashMap<>();
    private final Map<Goal, Future> activeGoals = new HashMap();
    private final Lock goalsLock = new ReentrantLock();
    private ActionLearning actionLearning;
    private boolean frozen = false;
    //TODO: Add constructor with resources when actually have resource definition
    //      Also replace Resource with (or at least enforce locking in line with) actual resource lock impl
    //Currently enforcing a temporary agent wide resource which all goals use in the QueueEM case
    //Adding a "learning" resource to enable executeWhileLearning behavior while not allowing other goals to interfere with learning
    private final Map<Symbol,Resource> resources = new HashMap<>();

    public AgentTeam(Symbol name, ExecutionManager em) {
        this.name = name;
        resources.put(name, new Resource(this, name));
        Symbol learningResource = Factory.createSymbol("learning");
        resources.put(learningResource, new Resource(this,learningResource));
        actionLearning = new ActionLearning(em, false);
    }

    public Symbol getName() {
        return name;
    }

    //// Hierarchy ////
    protected void addMember(AgentTeam member) {
        members.put(member.getName(), member);
    }

    protected void addParentTeam(AgentTeam parentTeam) {
        this.parentTeam = parentTeam;
    }

    public Map<Symbol, AgentTeam> getMembers() {
        return members;
    }

    public int getNumberOfMembers() {
        return members.size();
    }

    public Set<Symbol> getMemberNames() {
        return getMembers().keySet();
    }

    public AgentTeam getMember(Symbol memberName) {
        return members.get(memberName);
    }

    public AgentTeam getParentTeam() {
        return parentTeam;
    }

    public boolean isLeafAgent() {
        return members.isEmpty();
    }

    public boolean isRoot() {
        return parentTeam == null;
    }

    //// Goals ////
    protected void addGoal(Goal g, Future future) {
        try {
            goalsLock.lock();
            activeGoals.put(g, future);
        } finally {
            goalsLock.unlock();
        }
    }

    protected Future removeGoal(Goal g) {
        try {
            goalsLock.lock();
            return activeGoals.remove(g);
        } finally {
            goalsLock.unlock();
        }
    }

    protected Future getGoalFuture(Goal g) {
        try {
            goalsLock.lock();
            return activeGoals.get(g);
        } finally {
            goalsLock.unlock();
        }
    }

    //TODO: make sure locking in EM is safe
    protected void acquireGoalsLock() {
        goalsLock.lock();
    }

    protected void releaseGoalsLock() {
        goalsLock.unlock();
    }

    public Set<Goal> getActiveGoals() {
        try {
            goalsLock.lock();
            return activeGoals.keySet();
        } finally {
            goalsLock.unlock();
        }
    }

    public Goal getActiveGoal(long gid) {
        try {
            goalsLock.lock();
            for (Goal g : activeGoals.keySet()) {
                if (g.getId() == gid) {
                    return g;
                }
            }
        } finally {
            goalsLock.unlock();
        }
        return null;
    }

    //// Action Learning ////
    protected boolean learnAction(Predicate newAction) {
        return actionLearning.learnAction(newAction);
    }

    protected boolean endActionLearning(Predicate newAction) {
        return actionLearning.endActionLearning(newAction);
    }

    protected boolean pauseActionLearning(Predicate newAction) {
        return actionLearning.pauseActionLearning(newAction);
    }

    protected boolean resumeActionLearning(Predicate newAction) {
        return actionLearning.resumeActionLearning(newAction);
    }

    protected boolean cancelActionLearning(Predicate newAction) {
        return actionLearning.cancelActionLearning(newAction);
    }

    protected ActionLearningStatus getLearningStatus() {
        return actionLearning.getLearningStatus();
    }

    protected void waitForActionLearningStart(Predicate newAction) {
        actionLearning.waitForActionLearningStart(newAction);
    }

    protected void changeLearningExecution(Symbol status) {
        actionLearning.changeLearningExecution(status);
    }

    protected void modifyAction(Predicate action, Predicate modification, Predicate location) {
        actionLearning.modifyAction(action, modification, location);
    }

    protected boolean shouldIgnore(Goal g) {
        return actionLearning.shouldIgnore(g);
    }

    protected void addLearningGoal(Goal g) {
        actionLearning.addGoal(g);
    }

    protected boolean shouldExecute() {
        return actionLearning.shouldExecute();
    }

    //// Resources ////
    public Resource getResource(Symbol name) {
        return resources.get(name);
    }

    public Set<Symbol> getResourceNames() {
        return resources.keySet();
    }

    public Set<Resource> getResources() {
        return new HashSet<>(resources.values());
    }

    //// Freeze ////
    //TODO: can look to get rid of these - not reasoning at this level currently
    protected void freeze() {
        if (frozen) {
            log.warn("[freeze] agent {} is already frozen", name);
            return;
        }
        this.frozen = true;
        for (AgentTeam member: members.values()) {
            member.freeze();
        }
    }

    protected void unfreeze() {
        if (!frozen) {
            log.warn("[unfreeze] agent {} is not frozen", name);
            return;
        }
        this.frozen = false;
        for (AgentTeam member: members.values()) {
            member.unfreeze();
        }
    }

    public boolean isFrozen() {
        return frozen;
    }
}
