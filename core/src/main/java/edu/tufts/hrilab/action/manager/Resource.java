/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.manager;

import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.fol.Symbol;

import java.util.HashSet;
import java.util.Set;

public class Resource {
    private final AgentTeam provider;
    private final Symbol name;
    private boolean available = true;
    private Goal holder = null;
    private final Set<Goal> users = new HashSet<>();

    public Resource(AgentTeam provider, Symbol name) {
        this.provider = provider;
        this.name = name;
    }

    public synchronized boolean isAvailable() {
        return available;
    }

    public synchronized Goal getHolder() {
        return holder;
    }

    protected synchronized void setHolder(Goal holder) {
        this.holder = holder;
        available = false;
    }

    protected synchronized void releaseHolder() {
        holder = null;
        available = true;
    }

    AgentTeam getProvider() {
        return provider;
    }

    public Symbol getName() {
        return name;
    }

    public synchronized boolean addUser(Goal g) {
        return users.add(g);
    }

    public synchronized boolean removeUser(Goal g) {
        return users.remove(g);
    }

    public synchronized Set<Goal> getUsers() {
        return Set.copyOf(users);
    }

    @Override
    public String toString() {
        return name.toString();
    }
}
