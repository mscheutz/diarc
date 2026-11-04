/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.manager;

import edu.tufts.hrilab.action.goal.PriorityTier;

public class PriorityInfo {
    private long priority;
    private PriorityTier priorityTier;

    public PriorityInfo(long priority, PriorityTier priorityTier) {
        this.priority = priority;
        this.priorityTier = priorityTier;
    }

    public long getPriority() {
        return priority;
    }

    public void setPriority(long priority) {
        this.priority = priority;
    }

    public PriorityTier getPriorityTier() {
        return priorityTier;
    }

    public void setPriorityTier(PriorityTier priorityTier) {
        this.priorityTier = priorityTier;
    }
}
