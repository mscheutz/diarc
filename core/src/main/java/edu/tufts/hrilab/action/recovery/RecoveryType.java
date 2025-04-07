/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.recovery;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public enum RecoveryType {
    ACTION(true,false),
    GOAL(false,true),
    EITHER(true,true);

    private boolean isAction;
    private boolean isGoal;

    RecoveryType(boolean isAction, boolean isGoal) {
        this.isAction = isAction;
        this.isGoal = isGoal;
    }
    public boolean isAction() {
        return isAction;
    }

    public boolean isGoal() {
        return isGoal;
    }

    public boolean matches(RecoveryType other) {
        return isAction && other.isAction() || isGoal && other.isGoal();
    }

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