/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.learning;

public enum ActionLearningStatus {
  START,
  //STOP, brad:temporarily changed to prevent bug from nullary predicate interaction with belief
  END,
  ACTIVE,
  CANCEL,
  PAUSE,
  RESUME,
  NONE;

  public static ActionLearningStatus getEnumVal(String value) {
    return valueOf(value.toUpperCase());
  }
}
