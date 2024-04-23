/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.interfaces;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;

/**
 * Minimal interface for setting and getting velocities.
 */
public interface VelocityInterface {

  /**
   * Get translational velocity.
   * @return the most recent TV reading (m/sec).
   */
  @TRADEService
  @Action
  double getTV();

  /**
   * Get rotational velocity.
   * @return the most recent RV reading (rad/sec).
   */
  @TRADEService
  @Action
  double getRV();

  /**
   * Get translational and rotational velocity.
   * @return the most recent velocity readings (m/sec and rad/sec).
   */
  @TRADEService
  @Action
  double[] getVels();

  /**
   * Get the default velocities used by VelocityComponent functions.
   * @return the default velocities (m/sec and rad/sec).
   */
  @TRADEService
  @Action
  double[] getDefaultVels();

  /**
   * Stop.
   */
  @TRADEService
  @Action
  void stop();

  /**
   * Set translational velocity.
   * @param tv the new TV (m/sec)
   * @return true if there's nothing in front of the robot, false
   * otherwise.
   */
  @TRADEService
  @Action
  boolean setTV(double tv);

  /**
   * Set rotational velocity.
   * @param rv the new RV (rad/sec)
   * @return true if there's nothing on that side, false otherwise.
   */
  @TRADEService
  @Action
  boolean setRV(double rv);

  /**
   * Set both velocities.
   * @param tv the new TV (m/sec)
   * @param rv the new RV (rad/sec)
   * @return true if there's nothing in front of the robot, false
   * otherwise.
   */
  @TRADEService
  @Action
  boolean setVels(double tv, double rv);
  /**
   *Returns left and right wheel velocities.
   *
   double[] getVelocity();
   */

  /**
   * Return a three-element array of (x, y, theta) position.
   * @return A three-element {@code double} array (x,y,t)
  double[] getPoseEgo();
   */
}
