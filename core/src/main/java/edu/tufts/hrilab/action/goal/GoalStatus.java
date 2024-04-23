/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.goal;

import edu.tufts.hrilab.action.util.Utilities;

public enum GoalStatus {

  UNKNOWN(false, false),
  PENDING(false, false),
  ACTIVE(false, false),
  SUSPENDED(false, false),
  CANCELED(true, false),
  SUCCEEDED(true, false),
  FAILED(true, true);

  /**
   * If this enum instance represents a terminal status.
   */
  private final boolean terminated;

  /**
   * If this enum instance represents a failure status.
   */
  private final boolean failure;

  /**
   * Enum instance constructor.
   * @param isTerminated
   * @param isFailure
   */
  GoalStatus(boolean isTerminated, boolean isFailure) {
    terminated = isTerminated;
    failure = isFailure;
  }

  /**
   * Getter method for whether this enum instance represents a terminal status.
   * @return
   */
  public boolean isTerminated() {
    return terminated;
  }

  /**
   * Getter method for whether or not this enum instance represents a failure status.
   * @return
   */
  public boolean isFailure() {
    return failure;
  }

  /**
   * Case-insensitive version of valueOf.
   * @param string
   * @return
   */
  public static GoalStatus fromString(String string) {
    return Utilities.strToEnum(GoalStatus.class, string);
  }
}
