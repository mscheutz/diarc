package edu.tufts.hrilab.action.goal;

import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;

import java.io.Serializable;

public class GoalInfo implements Serializable {
  /**
   * Goal predicate. This predicate can toggle between bound and unbound forms, depending on if the goal execution
   * has been started. If the initial goal predicate has unbound variables, this predicate will contain unbound
   * variables until the GoalContext binds them during the action selection stage of goal execution. If the GoalContext
   * is reset and executed again, this predicate can be rebound to different variable bindings.
   */
  public Predicate goal;
  /**
   * Goal predicate that can have unbound variables. This goal predicate never changes, and is used
   * to unbind the goal predicate so that a Goal can be reused inside a GoalContext when the context is reset.
   */
  public Symbol unboundGoal;
  /**
   * Unique goal ID.
   */
  public final long gid;
  /**
   * Justification containing failure reasons.
   */
  public Justification failConditions;
  /**
   * System time that goal execution started.
   */
  public long start = -1;
  /**
   * System time that goal execution terminated.
   */
  public long end = -1;
  /**
   * GoalStatus.
   */
  public GoalStatus status = GoalStatus.PENDING;
  /**
   * Actor responsible for this goal.
   */
  public Symbol actor;
  /**
   * If this goal is a state-base goal (isAction==false) or action-based (isAction==true). Action goals map directly
   * onto an existing action, where no top-level task planning is needed.
   */
  public boolean isAction = false;
  /**
   * Optional metric for task planning.
   */
  public Predicate metric;
  /**
   * If true, this goal is wrapped as a persistent goal (i.e., while loop) and executed until failure or cancellation/suspension.
   */
  public boolean persistent = false;
  /**
   * Goal priority. Used by the ExecutionManager to determine what goals to execute.
   */
  public long priority = 1;

  public GoalInfo(long gid) {
    this.gid = gid;
  }

  public GoalInfo(GoalInfo other) {
    this.goal = other.goal;
    this.unboundGoal = other.unboundGoal;
    this.gid = other.gid;
    this.failConditions = other.failConditions;
    this.start = other.start;
    this.end = other.end;
    this.status = other.status;
    this.actor = other.actor;
    this.isAction = other.isAction;
    this.metric = other.metric;
    this.persistent = other.persistent;
    this.priority = other.priority;
  }
}
