/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.arm;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.util.Util;

public class MockUR5eComponent extends MockArmComponent {

  @TRADEService
  @Action
  public boolean startPouring(double ang) {
    log.info("Start Pouring, angle: " + ang);
    Util.Sleep(sleepDuration);

    return true;
  }

  @TRADEService
  @Action
  public boolean startPouring(Symbol obj) {
    log.info("Start Pouring, object: " + obj);
    Util.Sleep(sleepDuration);

    return true;
  }

  @TRADEService
  @Action
  public boolean stopPouring(Symbol obj) {
    log.info("Stop Pouring: " + obj);
    return true;
  }

  @TRADEService
  @Action
  public boolean stopPouring() {
    log.info("Stop Pouring");
    return true;
  }

  @TRADEService
  @Action
  public boolean moveAbove(Symbol s, Symbol s2) {
    log.info("Moving " + s + " above " + s2);
    Util.Sleep(sleepDuration);

    return true;
  }

}
