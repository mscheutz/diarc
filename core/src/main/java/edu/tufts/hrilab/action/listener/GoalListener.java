/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.listener;

import edu.tufts.hrilab.action.goal.GoalInfo;

/**
 * An interface that can be used to send notifications about Goals.
 */
public interface GoalListener {

  void goalStatusChanged (GoalInfo info);
}
