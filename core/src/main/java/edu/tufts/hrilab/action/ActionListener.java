/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action;

import edu.tufts.hrilab.action.execution.Context;

/**
 * An interface that can be used by the ActionInterpreter to notify other objects
 * (i.e. Goal Manager) that the action has completed.
 *
 * EW: Extended to notify objects on action start and on progress of action steps.
 * Currently used only for external GUI purposes, we can make separate listeners if we
 * don't want this in the GM.
 * 
 * This is an implementation of the
 * {@see <a href="https://en.wikipedia.org/wiki/Observer_pattern">Observer Pattern</a>}.
 * 
 * @author willie
 */
public interface ActionListener {

  void actionStarted (ActionInterpreter ai);

  void actionComplete (ActionInterpreter ai);

  void stepStarted (Context step);

  void stepComplete (Context step);
}
