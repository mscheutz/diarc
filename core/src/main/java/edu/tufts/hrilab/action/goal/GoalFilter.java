package edu.tufts.hrilab.action.goal;

public enum GoalFilter {
  ACTIVE, // being executed
  CURRENT, // active and pending
  PENDING, // waiting to execute
  NEXT, // PENDING and next to be executed
  PAST, // permanently terminated
  SYSTEM, // ACTIVE goal that does not skip the pending queue. TODO: rename this. maybe USER_ACTIVE ?
  ALL //
}
