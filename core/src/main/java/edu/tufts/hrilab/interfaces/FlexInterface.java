/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.interfaces;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.fol.Symbol;

/**
 * Interface to be implemented by a Python module, not a Java DIARC component.
 */
public interface FlexInterface {

  @TRADEService
  @Action
  Justification openDoor(Symbol refId);

  @TRADEService
  @Action
  Justification grabObject(Symbol refId);

  @TRADEService
  @Action
  Justification pushButton(Symbol refId);
}
