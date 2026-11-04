/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.scale;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Symbol;

public class MockScaleComponent extends DiarcComponent {

  boolean lastReading = true;

  public MockScaleComponent() {
  }

  @TRADEService
  @Action
  public boolean itemWeightGreaterThan(Symbol value) {
    boolean toReturn = lastReading;
    log.info("[itemWeightGreaterThan] checking if item weight is greater than " + value.getName() + " and returning " + toReturn);
    lastReading = !lastReading;
    return toReturn;
  }
}
