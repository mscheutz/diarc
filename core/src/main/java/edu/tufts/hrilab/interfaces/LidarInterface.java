/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.interfaces;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;


public interface LidarInterface {

  @TRADEService
  @Action
  public float[] getLaserScan();

  @TRADEService
  @Action
  public float getLaserAngleIncrement();
}
