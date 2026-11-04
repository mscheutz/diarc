/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision;

import ai.thinkingrobots.trade.TRADEService;

public interface MockVisionInterface extends VisionInterface {
  @TRADEService
  boolean setSceneIndex(int index);
  @TRADEService
  void setObsevations(boolean makeObservations);
}
