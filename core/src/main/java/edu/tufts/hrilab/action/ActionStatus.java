/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action;

import edu.tufts.hrilab.action.util.Utilities;

/**
 * Enumeration for the status of actions.
 */
public enum ActionStatus {
  UNKNOWN(false, false),
  INITIALIZED(false, false),
  CHECKINGAPPROVAL(false, false),
  APPROVED(false, false),
  PROGRESS(false, false),
  VERIFYING_RETURNVALUE(false, false),
  VERIFYING_EFFECTS(false, false),
  VERIFYING_NORMS(false, false),
  ACTIVE_CHILD(false, false),
  SUCCESS(true, false),
  RECOVERY(false, false),
  FAIL(true, true),
  FAIL_FORBIDDEN(true, true),
  FAIL_PRECONDITIONS(true, true),
  FAIL_OVERALLCONDITIONS(true, true),
  FAIL_POSTCONDITIONS(true, true),
  FAIL_OBLIGATIONS(true,true),
  FAIL_CONSTRAINTS(true, true),
  FAIL_ARGUMENTS(true, true),
  FAIL_ARGUMENTUNBOUND(true, true),
  FAIL_RETURNVALUE(true, true),
  FAIL_NOTFOUND(true, true),
  FAIL_ANCESTOR(true, true),
  FAIL_CHILD(true, true),
  FAIL_SYNTAX(true, true),
  FAIL_RECOVERY(true, true),
  FAIL_NORMS(true, true),
  SATISFYING_PRECONDITIONS(false, false),
  SATISFYING_OBLIGATIONS(false,false),
  PARENT_CANCELED(true,false),
  CANCEL(true, false),
  EXIT_RECOVERY(true,true),
  SUSPEND(false, false),
  RESUME(false, false),
  RETURN(false, false);

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
  ActionStatus(boolean isTerminated, boolean isFailure) {
    terminated = isTerminated;
    failure = isFailure;
  }

  /**
   * Getter method for whether or not this enum instance represents a terminal status.
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
  public static ActionStatus fromString(String string) {
    return Utilities.strToEnum(ActionStatus.class, string);
  }
}


