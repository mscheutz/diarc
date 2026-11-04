/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.interfaces;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.ActionStatus;

/**
 * Interface for complex/stateful motion commands (e.g., move to a location,
 * turn a given amount).
 */
public interface OrientationInterface {
    /** 
     * Turn to a global heading.
     * @param tdest the global heading (in radians) to which to turn
     * @return an identifying timestamp for the turn action
     */
    @TRADEService
    long turnTo(double tdest);

    /** 
     * Turn a specified distance.
     * @param dist the distance (in radians) to turn
     * @return an identifying timestamp for the turn action
     */
    @TRADEService
    long turnDist(double dist);

    /** 
     * Check status of current Motion.  Soon this will be obsoleted, as the
     * motion commands will notify Action of their completions.
     * @param aid the identifying timestamp of the action to check
     * @return the status found for the indicated action
     */
    @TRADEService
    ActionStatus checkMotion(long aid);

    /**
     * Cancel current Motion.
     * @param aid the identifying timestamp of the action to cancel
     * @return true if action was canceled, false otherwise (i.e., if that
     * action ID was not active)
     */
    @TRADEService
    boolean cancelMotion(long aid);
}
