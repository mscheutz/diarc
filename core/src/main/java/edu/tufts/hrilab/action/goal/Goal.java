/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.goal;

import edu.tufts.hrilab.action.ActionInterpreter;
import edu.tufts.hrilab.action.Observable;
import edu.tufts.hrilab.action.description.ContextDescription;
import edu.tufts.hrilab.action.execution.ArgumentBasedContext;
import edu.tufts.hrilab.action.listener.GoalListener;
import edu.tufts.hrilab.action.manager.Resource;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.util.IdGenerator;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.justification.Justification;

import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * <code>Goal</code> encapsulates a goal Predicate, the ActionInterpreter in charge of
 * executing it and some additional information.
 */
public class Goal {

  private static final Logger log = LoggerFactory.getLogger(Goal.class);

  /**
   * Goal predicate. This predicate can toggle between bound and unbound forms, depending on if the goal execution
   * has been started. If the initial goal predicate has unbound variables, this predicate will contain unbound
   * variables until the GoalContext binds them during the action selection stage of goal execution. If the GoalContext
   * is reset and executed again, this predicate can be rebound to different variable bindings.
   */

  private Set<GoalListener> listeners = new HashSet<>();
  private GoalInfo info;
  private Observable observable = Observable.FALSE;
  private static final IdGenerator idGenerator = new IdGenerator();
  private Set<Resource> requiredResources = null;
  private Set<Resource> heldResources = null;
  private boolean isLearningGoal = false;

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


  /**
   * Construct goal that defines the actor to achieve the goal, and the desired goal/action. This is the main method
   * used when using the GoalManager submitGoal method.
   *
   * @param goalState
   */
  public Goal(Predicate goalState) {
    info = new GoalInfo(idGenerator.getNext());

    // check persistent first bc other goal predicate forms could be nested inside
    if (goalState.getName().equals("persistent")) {             // persistent goal
      if (goalState.size() != 2) {  // illegal structure
        log.error("[Goal] illegal predicate structure: " + goalState);
        return;
      }
      goalState = (Predicate) goalState.get(1);
      info.persistent = true;
    }

    // handle special goal predicate forms
    switch (goalState.getName()) {
      case "sti":
      case "did":
        log.error("Goal form is no longer supported: " + goalState);
        break;
      case "obs":
      case "observe":
        observable = Observable.TRUE;
        // fall through
      case "goal":
        info.unboundGoal = goalState.get(1);
        break;
      default: // default case is an explicit action to execute
        info.isAction = true;
        info.unboundGoal = goalState;
    }

    // actor is always first arg
    info.actor = goalState.get(0);
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
        info.unboundGoal = ((Predicate) goalState).get(0);
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
    info = new GoalInfo(idGenerator.getNext());
    this.observable = observable;
    info.actor = actor;

    if (actor == null) {
      log.error("Invalid null actor for goal: " + goalState);
      return;
    }

    if (goalState.isVariable()) {
      info.unboundGoal = goalState;
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
        info.persistent = true;
      }


      // error messages for deprecated special predicate forms
      switch (goalState.getName()) {
        case "sti":
        case "did":
          log.error("Goal form is no longer supported: " + predicate);
      }

      this.info.unboundGoal = predicate;
    } else {
      log.error("Goal state is not a Variable or Predicate: " + goalState);
    }
  }

  /**
   * Get the goal predicate. This can contain free variables, and it can also become bound
   * depending on where in the goal execution process this method is called.
   *
   * @return
   */
  public Predicate getPredicate() {
    if (info.goal == null) {
      // if bindPredicate has not been called yet
      if (info.unboundGoal.isPredicate()) {
        return (Predicate) info.unboundGoal;
      } else if (info.unboundGoal.isVariable()) {
        return Factory.createPredicate("unbound", info.unboundGoal);
      } else {
        log.error("Goal predicate has invalid form: " + info.unboundGoal);
        return null;
      }
    } else {
      return info.goal;
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
      Predicate boundGoal = ((ArgumentBasedContext) context).bindPredicate(info.unboundGoal);
      if (boundGoal == null) {
        return false;
      }
      info.goal = boundGoal;
      return true;
    }

    return false;
  }

  public long getId() {
    return info.gid;
  }

  public boolean isAction() {
    return info.isAction;
  }

  //TODO:brad: see comments in problem solving action selector, this can be removed when those are fixed.
  public void setIsAction(boolean value) {
    info.isAction = value;
  }

  public boolean isObservation() {
    return observable != Observable.FALSE;
  }

  public Observable getObservableEnum() {
    return observable;
  }

  public boolean isPersistent() {
    return info.persistent;
  }

  public Symbol getActor() {
    return info.actor;
  }

  public Justification getFailConditions() {
    return info.failConditions;
  }

  public GoalStatus getStatus() {
    return info.status;
  }

  public long getStartTime() {
    return info.start;
  }

  public long getEndTime() {
    return info.end;
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
        log.error("[setRootContext] rootContext already set for goal: " + info.goal);
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
    if (!info.status.isTerminated()) {
      info.start = System.currentTimeMillis();
      setStatus(GoalStatus.ACTIVE);
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
  public void setStatus(GoalStatus status) {
    if (this.info.status.isTerminated()) {
      log.error("Goal already has terminal status. Goal: {} Status: {}", getPredicate(), status);
    } else {
      this.info.status = status;

      if (this.info.status.isTerminated()) {
        info.end = System.currentTimeMillis();
      }

      // notify listeners
      listeners.forEach(l -> l.goalStatusChanged(info));
    }
  }

  /**
   * Alternate method to setStatus, to set failure condition before sending goal status notifications.
   *
   * @param status
   * @param failureJustification
   */
  public void setFailureStatus(GoalStatus status, Justification failureJustification) {
    if (!status.isFailure()) {
      log.error("[setFailureStatus] status is not a failure type: {}", status);
      setStatus(status);
    }

    info.failConditions = failureJustification;
    setStatus(status);
  }

  //TODO: Have computePriority() method, compute priority externally and pass in single value here, or have separate
  // class variables for all relevant features and use in compareTo()?
  public void setPriority(long priority) {
    info.priority = priority;
  }

  public long getPriority() {
    return info.priority;
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
   * setRequiredResources to be called first (by the ExecutionManager). Will return null if the set
   * of resources has not yet been computed by an ExecutionManager.
   */
  public Set<Resource> getRequiredResources() {
    return requiredResources;
  }

  /**
   * Set the set of resources which must be available for this goal during execution. This method should only
   * be called by the Execution Manager on submission.
   */
  public void setRequiredResources(Set<Resource> requiredResources) {
    this.requiredResources = requiredResources;
  }

  /**
   * Get the set of resources that will be held by this goal during execution.
   * Note: This set is specific to the subclass of ExecutionManager being used in the system and requires
   * setRequiredResources to be called first (by the ExecutionManager). Will return null if the set
   * of resources has not yet been computed by an ExecutionManager.
   */
  public Set<Resource> getHeldResources() {
    return heldResources;
  }

  /**
   * Set the set of resources which will be held by this goal during execution. This method should only
   * be called by the Execution Manager on submission.
   */
  public void setHeldResources(Set<Resource> heldResources) {
    this.heldResources = heldResources;
  }

  /**
   * Set indication whether this goal is being used as a part of action learning. This is important for reasoning in the
   * ExecutionManager about whether the goal conflicts with active learning.
   */
  public void setIsLearningGoal(boolean isLearningGoal) {
    this.isLearningGoal = isLearningGoal;
  }

  /**
   * Returns true if this goal was submitted as part of an action learning process
   */
  public boolean isLearningGoal() {
    return isLearningGoal;
  }

  /**
   * Cancels goal execution (if currently being executed). If being executed, calls cancel
   * on the ActionInterpreter which will set the status of this goal once execution has terminated.
   */
  public void cancel() {
    if (ai != null) {
      // the AI will set the CANCELED status once it has terminated
      ai.cancel();
    } else {
      setStatus(GoalStatus.CANCELED);
    }
  }

  /**
   * Suspends goal execution (if currently being executed). If being executed, calls suspend
   * on the ActionInterpreter which will set the status of this goal once execution has fully suspended.
   */
  public void suspend() {
    if (ai != null) {
      // the AI will set the SUSPENDED status once it has been suspended
      ai.suspend();
    } else {
      setStatus(GoalStatus.SUSPENDED);
    }
  }

  /**
   * Resumes goal execution (if currently suspended). Calls suspend
   * on the ActionInterpreter which will set the status of this goal once execution has resumed.
   */
  public void resume() {
    if (ai != null) {
      // the AI will set the ACTIVE status once it has resumed
      ai.resume();
    }
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

  @Override
  public String toString() {
    if (info.goal == null) {
      return info.unboundGoal.toString();
    } else {
      return info.goal.toString();
    }
  }

  public void setMetric(Predicate metric) {
    info.metric = metric;
  }

  public Predicate getMetric() {
    return info.metric;
  }

  /**
   * Get copy of GoalInfo.
   *
   * @return
   */
  public GoalInfo getInfo() {
    return new GoalInfo(info);
  }

  public void addListener(GoalListener listener) {
    listeners.add(listener);
  }

  public void removeListener(GoalListener listener) {
    listeners.remove(listener);
  }

}
