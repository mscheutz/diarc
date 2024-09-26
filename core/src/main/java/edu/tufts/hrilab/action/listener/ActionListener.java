/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.listener;

import edu.tufts.hrilab.action.execution.Context;

/**
 * An interface that can be used by the ActionInterpreter to notify other objects
 * of progress during action execution. In particular, when a Context has started/ended.
 */
public interface ActionListener {
  /**
   * Method to be called when a Context has started execution.
   * @param step
   */
  void stepStarted (Context step);

  /**
   * Method to be called when a Context has ended execution.
   * @param step
   */
  void stepComplete (Context step);
}
