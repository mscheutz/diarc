/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.goal;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public enum PriorityTier {
    LOW,
    NORMAL,
    HIGH,
    URGENT,
    SKIPPENDING; //Placeholder for functionality hopefully resolved by implemented lower level resource constraints

    private static Logger log = LoggerFactory.getLogger(PriorityTier.class);

    public static PriorityTier fromString(String typeStr) {
        if (typeStr != null) {
            for (PriorityTier t : PriorityTier.values()) {
                if (typeStr.equalsIgnoreCase(t.name())) {
                    return t;
                }
            }
        }
        log.error("Unknown priority tier: " + typeStr);
        return null;
    }
}
