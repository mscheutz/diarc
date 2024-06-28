/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.goal;

import edu.tufts.hrilab.action.ActionInterpreter;
import edu.tufts.hrilab.action.Observable;
import edu.tufts.hrilab.action.description.ContextDescription;
import edu.tufts.hrilab.action.execution.ArgumentBasedContext;
import edu.tufts.hrilab.action.manager.Resource;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.util.IdGenerator;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.justification.Justification;

import java.io.Serializable;
import java.util.Set;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * <code>Goal</code> encapsulates a goal Predicate, the ActionInterpreter in charge of
 * executing it and some additional information.
 */
public class Goal implements Serializable {

  private static final Logger log = LoggerFactory.getLogger(Goal.class);

  static final long serialVersionUID =-6787920823195882966L;

  /**
   * Goal predicate. This predicate can toggle between bound and unbound forms, depending on if the goal execution
   * has been started. If the initial goal predicate has unbound variables, this predicate will contain unbound
   * variables until the GoalContext binds them during the action selection stage of goal execution. If the GoalContext
   * is reset and executed again, this predicate can be rebound to different variable bindings.
   */
  private Predicate goal;
  /**
   * Goal predicate that can have unbound variables. This goal predicate never changes, and is used
   * to unbind the goal predicate so that a Goal can be reused inside a GoalContext when the context is reset.
   */
  private Symbol unboundGoal;
  private final long gid;
  private Justification failConditions;
  private long start = -1;
  private long end = -1;
  private boolean terminated = false;
  private GoalStatus status = GoalStatus.PENDING;
  private Symbol actor;
  private Observable observable = Observable.FALSE;
  private boolean persistent = false;
  private boolean isAction = false;
  private static final IdGenerator idGenerator = new IdGenerator();
  private Predicate metric;
  private Set<Resource> requiredResources = null;

  /**
   * The ActionInterpreter currently executing this Goal. Can be null if this Goal isn't currently being executed.
   */
  private ActionInterpreter ai = null;
  /**
   * The current node in the context tree. This can be null.
   */
  private Context currentContext;
  /**
   * The root node of the context tree for executing this Goal (not the root node of the entire context tree).
   */
  private Context rootContext;
  private Lock contextLock = new ReentrantLock();

  private PriorityTier priorityTier = PriorityTier.NORMAL;
  private long priority = 1;

  /**
   * Get the goal predicate. This can contain free variables, and it can also become bound
   * depending on where in the goal execution process this method is called.
   *
   * @return
   */
  public Predicate getPredicate() {
    if (goal == null) {
      // if bindPredicate has not been called yet
      if (unboundGoal.isPredicate()) {
        return (Predicate) unboundGoal;
      } else if (unboundGoal.isVariable()) {
        return Factory.createPredicate("unbound", unboundGoal);
      } else {
        log.error("Goal predicate has invalid form: " + unboundGoal);
        return null;
      }
    } else {
      return goal;
    }
  }

  /**
   * Binds goal predicate to values in context.
   *
   * @param context context providing bindings
   * @return true if predicate was bound using variables in context
   */
  public boolean bindPredicate(Context context) {
    if (context instanceof ArgumentBasedContext) {
      Predicate boundGoal = ((ArgumentBasedContext) context).bindPredicate(unboundGoal);
      if (boundGoal == null) {
        return false;
      }
      goal = boundGoal;
      return true;
    }

    return false;
  }

  public long getId() {
    return gid;
  }

  public boolean isAction() {
    return isAction;
  }

  //TODO:brad: see comments in problem solving action selector, this can be removed when those are fixed.
  public void setIsAction(boolean value) {
    isAction= value;
  }

  public boolean isObservation() {
    return observable != Observable.FALSE;
  }

  public Observable getObservableEnum() {
    return observable;
  }

  public boolean isPersistent() {
    return persistent;
  }

  public Symbol getActor() {
    return actor;
  }

  public Justification getFailConditions() {
    return failConditions;
  }

  public GoalStatus getStatus() {
    return status;
  }

  public long getStartTime() {
    return start;
  }

  public long getEndTime() {
    return end;
  }

  /**
   * Set the root context of this goal (not the root of the entire tree).
   *
   * @param context
   */
  public void setRootContext(Context context) {
    contextLock.lock();
    try {
      if (rootContext != null) {
        log.error("[setRootContext] rootContext already set for goal: " + goal);
        return;
      }
      rootContext = context;
    } finally {
      contextLock.unlock();
    }
  }

  /**
   * Set the current context being executed by an ActionInterpreter.
   *
   * @param context
   */
  public void setCurrentContext(Context context) {
    contextLock.lock();
    try {
      currentContext = context;
    } finally {
      contextLock.unlock();
    }

  }

  /**
   * Get the root context of this goal (not the root of the entire tree).
   *
   * @return
   */
  public Context getRootContext() {
    contextLock.lock();
    try {
      return rootContext;
    } finally {
      contextLock.unlock();
    }
  }

  /**
   * Get the current context being executed by an ActionInterpreter.
   *
   * @return
   */
  public Context getCurrentContext() {
    contextLock.lock();
    try {
      return currentContext;
    } finally {
      contextLock.unlock();
    }
  }

  public ContextDescription getContextDescription(Predicate location) {
    return getRootContext().getContextDescription(location);
  }

  /**
   * Set goal status to ACTIVE and set start time.
   * This method does not actually start goal execution.
   *
   * @return
   */
  public boolean setAsStarted() {
    if (!terminated) {
      start = System.currentTimeMillis();
      status = GoalStatus.ACTIVE;
      return true;
    }
    return false;
  }

  /**
   * Set the status of this goal. Does not directly change the
   * actual execution of this goal.
   *
   * @param status
   */
  //TODO:brad: this shouldn't be public, but we need to change it in action learning
  public void setStatus(GoalStatus status) {
    this.status = status;
  }

  /**
   * Set goal status to passed in status, set the end time, and
   * set the internal terminated flag.
   * This method does not actually terminate the goal execution.
   *
   * @param status
   */
  public void setAsTerminated(GoalStatus status) {
    if (terminated) {
      log.error("[setAsTerminated] goal already terminated: " + this);
    } else {
      terminated = true;
      this.status = status;
      end = System.currentTimeMillis();
    }
  }

  //TODO: Have computePriority() method, compute priority externally and pass in single value here, or have separate
  // class variables for all relevant features and use in compareTo()?
  public void setPriority(long priority) {
    this.priority = priority;
  }

  public long getPriority() {
    return priority;
  }

  public void setPriorityTier(PriorityTier priorityTier) {
    this.priorityTier = priorityTier;
  }

  public PriorityTier getPriorityTier() {
    return priorityTier;
  }

  /**
   * Get the set of resources necessary to be available in order to execute this goal.
   * Note: This set is specific to the subclass of ExecutionManager being used in the system and requires
   *       setRequiredResources to be called first (by the ExecutionManager). Will return null if the set
   *       of resources has not yet been computed by an ExecutionManager.
   */
  public Set<Resource> getRequiredResources() {
    return requiredResources;
  }

  /**
   * Set the set of resources which will be held by this goal during execution. This method should only
   * be called by the Execution Manager on submission.
   */
  public void setRequiredResources(Set<Resource> requiredResources) {
    this.requiredResources = requiredResources;
  }

  /**
   * Set the goal status to CANCEL and also attempt to cancel the
   * goal execution.
   */
  public void cancel() {
    // set status to cancel which can be overwritten by AI
    status = GoalStatus.CANCELED;

    if (ai != null) {
      ai.cancel();
    }
  }

  /**
   * TODO: this is a half-baked attempt at pausing a goal, but only
   * pauses the current AI, and not any child AIs. Something closer to
   * how cancel works will need to be implemented. For instance, suspend could cancel the goal execution
   * but keep the goal as a current goal in the BGM.
   */
  public void suspend() {
    // set status to cancel which can be overwritten by AI
    status = GoalStatus.SUSPENDED;

    if (ai != null) {
      ai.suspend();
    }
  }

  public void resume() {
    // set status to active which can be overwritten by AI
    status = GoalStatus.ACTIVE;

    if (ai != null) {
      ai.resume();
    }
  }

  public void setFailConditions(Justification cond) {
    log.debug("[setFailCondition] actor: " + actor + " failcond: " + cond);
    failConditions = cond;
  }

  public ActionInterpreter getActionInterpreter() {
    return ai;
  }

  public boolean setActionInterpreter(ActionInterpreter interpreter) {
    if (ai == null) {
      ai = interpreter;
      return true;
    } else {
      log.error("[setActionInterpreter] ActionInterpreter already set for goal: " + this);
      return false;
    }
  }

  /**
   * Construct goal that defines the actor to achieve the goal, and the desired goal/action. This is the main method
   * used when using the GoalManager submitGoal method.
   *
   * @param goalState
   */
  public Goal(Predicate goalState) {
    gid = idGenerator.getNext();

    // check persistent first bc other goal predicate forms could be nested inside
    if (goalState.getName().equals("persistent")) {             // persistent goal
      if (goalState.size() != 2) {  // illegal structure
        log.error("[Goal] illegal predicate structure: " + goalState);
        return;
      }
      goalState = (Predicate) goalState.get(1);
      persistent = true;
    }

    // handle special goal predicate forms
    switch (goalState.getName()) {
      case "sti":
      case "did":
        log.error("Goal form is no longer supported: " + goalState);
        break;
      case "obs":
      case "observe":
        this.observable = Observable.TRUE;
        // fall through
      case "goal":
        this.unboundGoal = goalState.get(1);
        break;
      default: // default case is an explicit action to execute
        isAction = true;
        this.unboundGoal = goalState;
    }

    // actor is always first arg
    this.actor = goalState.get(0);
  }

  /**
   * Construct goal with explicit actor. This is primarily used for creating goal instance from ASL
   *
   * @param actor
   * @param goalState
   */
  public Goal(Symbol actor, Symbol goalState) {
    this(actor, goalState, Observable.FALSE);

    // handle special cases
    switch (goalState.getName()) {
      case "obs":
      case "observe":
        this.unboundGoal = ((Predicate)goalState).get(0);
        this.observable = Observable.TRUE;
    }
  }

  /**
   * Goal constructor used to explicitly mark goal as an observation. It's possible to
   * have an observation goal without the OBS(actor, state) predicate form. This is almost
   * always the case for observations from conditions and effects checks.
   *
   * @param actor      the actor doing the observing (can be null, if it can be inferred from predicate)
   * @param goalState  the goal predicate
   * @param observable enum specifying if goal is observable, and if so what type (true/false/default)
   */
  public Goal(Symbol actor, Symbol goalState, Observable observable) {
    gid = idGenerator.getNext();
    this.observable = observable;
    this.actor = actor;

    if (actor == null) {
      log.error("Invalid null actor for goal: " + goalState);
      return;
    }

    if (goalState.isVariable()) {
      this.unboundGoal = goalState;
    } else if (goalState.isPredicate()) {
      // should be a predicate
      Predicate predicate = (Predicate) goalState;

      // check persistent first bc other goal predicate forms could be nested inside
      if (predicate.getName().equals("persistent")) {             // persistent goal
        if (predicate.size() != 1) {  // illegal structure
          log.error("[Goal] illegal predicate structure: " + predicate);
          return;
        }
        predicate = (Predicate) predicate.get(0);
        persistent = true;
      }


      // error messages for deprecated special predicate forms
      switch (goalState.getName()) {
        case "sti":
        case "did":
          log.error("Goal form is no longer supported: " + predicate);
      }

      this.unboundGoal = predicate;
    } else {
      log.error("Goal state is not a Variable or Predicate: " + goalState);
    }
  }

  @Override
  public String toString() {
    if (goal == null) {
      return unboundGoal.toString();
    } else {
      return goal.toString();
    }
  }

  public void setMetric(Predicate metric) {
    this.metric = metric;
  }

  public Predicate getMetric() {
    return metric;
  }
}
