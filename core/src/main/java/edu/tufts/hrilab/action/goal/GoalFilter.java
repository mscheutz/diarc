package edu.tufts.hrilab.action.goal;

/**
 * Enum to define different Goal categories. This is mainly used to filter the GoalManager's goals
 * during goal queries.
 */
public enum GoalFilter {
  ACTIVE, // being executed
  CURRENT, // active and pending
  PENDING, // waiting to execute
  NEXT, // PENDING and next to be executed
  PAST, // permanently terminated
  SYSTEM, // ACTIVE goal that does not skip the pending queue. TODO: rename this. maybe USER_ACTIVE ?
  ALL //
}
