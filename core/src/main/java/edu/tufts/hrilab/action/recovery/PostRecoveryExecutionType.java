/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.recovery;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Defines the desired execution behavior to be performed after successful execution of an action recovery policy
 * PARENT_RETRY: Re-execute the parent context in which the failure is caught.
 *  Example Use Case: Failure causes success effects of previous actions in a script to be undone. Recovery cleans up
 *  any relevant side effects and places in a state where the encompassing action can be retried.
 * RETRY: Re-execute the failed context which triggered recovery.
 *  Example Use Case: Obstacle prevents action success but causes no side effects. Recovery removes obstacle and
 *  thereafter only the failed action needs to be re-executed.
 * SUCCESS: Continue execution as if the failed context triggering recovery has succeeded.
 *  Example Use Case: Recovery directly achieves goal state of failed action, planning around novel information which
 *  caused initial failure.
 * PARENT_SUCCESS: Continue execution as if the parent context in which the failure was caught has succeeded.
 *  Example use Case: Recovery achieves the goal state of the parent script, likely through human interaction.
 */
public enum PostRecoveryExecutionType {
    PARENT_RETRY,
    RETRY,
    SUCCESS,
    PARENT_SUCCESS;

    private static Logger log = LoggerFactory.getLogger(PostRecoveryExecutionType.class);

    public static PostRecoveryExecutionType fromString(String typeStr) {
        if (typeStr != null) {
            if (typeStr.startsWith("postrecovery_")) {
                typeStr = typeStr.substring(13);
            }
            for (PostRecoveryExecutionType t : PostRecoveryExecutionType.values()) {
                if (typeStr.equalsIgnoreCase(t.name())) {
                    return t;
                }
            }
        }
        log.error("Unknown post recovery execution type: " + typeStr);
        return null;
    }
}
