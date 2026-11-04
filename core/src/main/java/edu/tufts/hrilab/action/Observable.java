/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action;

/**
 * Indicates whether a state should be
 * - observed (TRUE): look for an observer than can verify it
 * - inferred (FALSE): check in StateMachine/Belief
 * - both (DEFAULT): try to observe first, if it fails (no observer or not found), check in StateMachine/Belief.
 */
public enum Observable {
  TRUE,
  FALSE,
  DEFAULT;

  public boolean shouldObserve() {
    return this != Observable.FALSE;
  }

  public static Observable get(boolean observable, boolean inferable) {
    if(observable) {
      return TRUE;
    } else if(inferable) {
      return FALSE;
    } else {
      return DEFAULT;
    }
  }

  public static Observable get(Boolean observable) {
    if(observable == null) {
      return DEFAULT;
    } else if (observable) {
      return TRUE;
    } else {
      return FALSE;
    }
  }
}
