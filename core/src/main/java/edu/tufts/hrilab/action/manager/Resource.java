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

    public boolean isAvailable() {
        return available;
    }

    public Goal getHolder() {
        return holder;
    }

    protected void setHolder(Goal holder) {
        this.holder = holder;
        available = false;
    }

    protected void releaseHolder() {
        holder = null;
        available = true;
    }

    private AgentTeam getProvider() {
        return provider;
    }

    public Symbol getName() {
        return name;
    }

    public boolean addUser(Goal g) {
        return users.add(g);
    }

    public boolean removeUser(Goal g) {
        return users.remove(g);
    }

    public Set<Goal> getUsers() {
        return users;
    }

    @Override
    public String toString() {
        return name.toString();
    }
}
