/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.interfaces;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.annotations.Action;

/**
 * Interface for complex/stateful motion commands (e.g., move to a location,
 * turn a given amount).
 */
public interface NavigationInterface {

  /**
   * Move to a global location.
   *
   * @param xdest the x-coordinate of the destination
   * @param ydest the y-coordinate of the destination
   * @return an identifying timestamp for the move action
   */
    @TRADEService
    @Action
    long moveTo(double xdest, double ydest);

  /**
   * Move forward a specified distance.
   *
   * @param dist the distance (in meters) to move
   * @return an identifying timestamp for the move action
   */
    @TRADEService
    @Action
    long moveDist(double dist);

  /**
   * Check status of current Motion. Soon this will be obsoleted, as the
   * motion commands will notify Action of their completions.
   *
   * @param aid the identifying timestamp of the action to check
   * @return the status found for the indicated action
   */
    @TRADEService
    @Action
    ActionStatus checkMotion(long aid);

  /**
   * Cancel current Motion.
   *
   * @param aid the identifying timestamp of the action to cancel
   * @return true if action was canceled, false otherwise (i.e., if that
   * action ID was not active)
   */
    @TRADEService
    @Action
    boolean cancelMotion(long aid);
}
