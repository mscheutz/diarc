/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.interfaces;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.justification.Justification;

/**
 * Interface to be implemented by a Python module, not a Java DIARC component.
 */
public interface FlexPythonInterface {

  @TRADEService
  @Action
  Justification open_door_primitive(int x_pix, int y_pix);
}
