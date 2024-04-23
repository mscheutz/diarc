/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.fetch;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


public enum FetchBreakerStates {
    STATE_DISABLED, STATE_ENABLED, STATE_ERROR;

    protected static Logger log = LoggerFactory.getLogger(FetchBreakerStates.class);

    public static FetchBreakerStates valToFetchBreakerStates(int val) {
        for (FetchBreakerStates breakerState: values()) {
            if (breakerState.ordinal() == val) {
                return breakerState;
            }
        }
        log.warn("[valToFetchBreakerStates] value " + val + " is not a valid ordinal value.");
        return null;
    }
}