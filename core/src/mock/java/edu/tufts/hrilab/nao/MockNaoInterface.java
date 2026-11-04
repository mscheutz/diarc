/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.nao;

import ai.thinkingrobots.trade.TRADEService;

/**
 *
 * @author Evan Krause
 */
public interface MockNaoInterface extends NaoInterface {

  /**
   * To set if there's floor support. Allows mocking the sensor checks done from
   * the real robot.
   *
   * @param value
   */
  @TRADEService
  void setFloorSupport(boolean value);

  /**
   * To set if there's an obstacle. Allows mocking the sensor checks done from
   * the real robot.
   *
   * @param value
   */
  @TRADEService
  void setObstacle(boolean value);

}
