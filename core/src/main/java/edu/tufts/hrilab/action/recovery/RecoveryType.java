/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.recovery;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public enum RecoveryType {
    ACTION,
    GOAL;

    private static Logger log = LoggerFactory.getLogger(RecoveryType.class);

    public static RecoveryType fromString(String typeStr) {
        if (typeStr != null) {
            for (RecoveryType t : RecoveryType.values()) {
                if (typeStr.equalsIgnoreCase(t.name())) {
                    return t;
                }
            }
        }
        log.error("Unknown recovery type: " + typeStr);
        return null;
    }
}