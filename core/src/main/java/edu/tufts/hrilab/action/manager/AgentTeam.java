/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.manager;

import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.fol.Factory;
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
    private boolean frozen = false;
    //TODO: Add constructor with resources when actually have resource definition
    //      Also replace Resource with (or at least enforce locking in line with) actual resource lock impl
    //Currently enforcing a temporary agent wide resource which all goals use in the QueueEM case
    private final Map<Symbol,Resource> resources = new HashMap<>();

    public AgentTeam(Symbol name) {
        this.name = name;
        resources.put(name, new Resource(this, name));
    }

    public AgentTeam(Symbol name, HashMap<Symbol, AgentTeam> members) {
        this.name = name;
        this.members = members;
        resources.put(name, new Resource(this, name));
    }

    public AgentTeam(Symbol name, AgentTeam parentTeam) {
        this.name = name;
        this.parentTeam = parentTeam;
        resources.put(name, new Resource(this, name));
    }

    public void addMember(AgentTeam member) {
        members.put(member.getName(), member);
    }

    public void addParentTeam(AgentTeam parentTeam) {
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

    public void addGoal(Goal g, Future future) {
        try {
            goalsLock.lock();
            activeGoals.put(g, future);
        } finally {
            goalsLock.unlock();
        }
    }

    public Future removeGoal(Goal g) {
        try {
            goalsLock.lock();
            return activeGoals.remove(g);
        } finally {
            goalsLock.unlock();
        }
    }

    public Future getGoalFuture(Goal g) {
        try {
            goalsLock.lock();
            return activeGoals.get(g);
        } finally {
            goalsLock.unlock();
        }
    }

    //TODO: make sure locking in EM is safe
    public void acquireGoalsLock() {
        goalsLock.lock();
    }

    public void releaseGoalsLock() {
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

    public Symbol getName() {
        return name;
    }

    //TODO: can look to get rid of these
    public void freeze() {
        if (frozen) {
            log.warn("[freeze] agent {} is already frozen", name);
            return;
        }
        this.frozen = true;
        for (AgentTeam member: members.values()) {
            member.freeze();
        }
    }

    public void unfreeze() {
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

    public Resource getResource(Symbol name) {
        return resources.get(name);
    }

    public Set<Symbol> getResourceNames() {
        return resources.keySet();
    }

    public Set<Resource> getResources() {
        return new HashSet<>(resources.values());
    }
}
