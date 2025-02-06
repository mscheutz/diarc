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
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.Locale;
import java.util.Set;
import java.util.concurrent.TimeUnit;
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
  private GoalStatus status = GoalStatus.PENDING;
  private Symbol actor;
  private Observable observable = Observable.FALSE;
  private boolean isAction = false;
  private static final IdGenerator idGenerator = new IdGenerator();
  private Predicate metric;
  private Set<Resource> requiredResources = null;
  private Set<Resource> heldResources = null;
  private boolean isLearningGoal = false;
  private boolean persistent = false;
  private boolean scheduled = false;
  private Long scheduledDelay = null;
  private TimeUnit scheduledDelayTimeUnit = null;
  private Date scheduledDate = null;

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

  public boolean isScheduled() {
    return scheduled;
  }

  public long getScheduledDelay() {
    return scheduledDelay;
  }

  public TimeUnit getScheduledDelayTimeUnit() {
    return scheduledDelayTimeUnit;
  }

  public Date getScheduledDate() {
    return scheduledDate;
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
    if (!status.isTerminated()) {
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
  public void setStatus(GoalStatus status) {
    if (this.status.isTerminated()) {
      log.error("Goal already has terminal status. Goal: {} Status: {}", getPredicate(), status);
    } else {
      this.status = status;
      if (this.status.isTerminated()) {
        end = System.currentTimeMillis();
      }
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
   * Set the set of resources which must be available for this goal during execution. This method should only
   * be called by the Execution Manager on submission.
   */
  public void setRequiredResources(Set<Resource> requiredResources) {
    this.requiredResources = requiredResources;
  }

  /**
   * Get the set of resources that will be held by this goal during execution.
   * Note: This set is specific to the subclass of ExecutionManager being used in the system and requires
   *       setRequiredResources to be called first (by the ExecutionManager). Will return null if the set
   *       of resources has not yet been computed by an ExecutionManager.
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
  public boolean cancel() {
    if (ai != null) {
      // the AI will set the CANCELED status once it has terminated
      return ai.cancel();
    } else {
      setStatus(GoalStatus.CANCELED);
      return true;
    }
  }

  /**
   * Suspends goal execution (if currently being executed). If being executed, calls suspend
   * on the ActionInterpreter which will set the status of this goal once execution has fully suspended.
   */
  public boolean suspend() {
    if (ai != null) {
      // the AI will set the SUSPENDED status once it has been suspended
      return ai.suspend();
    } else {
      setStatus(GoalStatus.SUSPENDED);
      return true;
    }
  }

  /**
   * Resumes goal execution (if currently suspended). Calls suspend
   * on the ActionInterpreter which will set the status of this goal once execution has resumed.
   */
  public boolean resume() {
    // set status to active which can be overwritten by AI
    status = GoalStatus.ACTIVE;

    if (ai != null) {
      // the AI will set the ACTIVE status once it has resumed
      return ai.resume();
    } else {
      return true;
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

    // check for certain special forms first bc other goal predicate forms could be nested inside
    Predicate unwrappedGoalState = unwrapGoalPredicate(goalState, true);
    if (unwrappedGoalState != null) {
      goalState = unwrappedGoalState;
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

      predicate = unwrapGoalPredicate(predicate, false);
      if (predicate == null) {
        return;
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

  /**
   * Check for special goal forms on submission (Currently 'scheduled' and 'persistent'). Unwraps the predicate to
   * submit while populating class variables with values based on wrapper names and arguments.
   */
  private Predicate unwrapGoalPredicate(Predicate goalState, boolean actorIncluded) {
    if (goalState.getName().equals("persistent")) {
      if (actorIncluded) {
        if (goalState.size() != 2) {  // illegal structure
          log.error("[Goal] illegal predicate structure for persistent goal: " + goalState);
          return null;
        }
        goalState = (Predicate) goalState.get(1);
      } else {
        if (goalState.size() != 1) {
          log.error("[Goal] illegal predicate structure for persistent goal: " + goalState);
          return null;
        }
        goalState = (Predicate) goalState.get(0);
      }
      persistent = true;

      return unwrapGoalPredicate(goalState, false);
    } else if (goalState.getName().equals("scheduled")) {
      int formatType = 0;
      String dateString = "";
      String delayString = "";
      String timeUnitString = "";
      if (actorIncluded) {
        if (goalState.size() == 3) {
          dateString = goalState.get(2).toString();
          formatType = 2;
        } else if (goalState.size() == 4) {
          delayString = goalState.get(2).toString();
          timeUnitString = goalState.get(3).toString();
          formatType = 1;
        }
        goalState = (Predicate) goalState.get(1);
      } else {
        if (goalState.size() == 2) {
          dateString = goalState.get(1).toString();
          formatType = 2;
        } else if (goalState.size() == 3) {
          delayString = goalState.get(1).toString();
          timeUnitString = goalState.get(2).toString();
          formatType = 1;
        }
        goalState = (Predicate) goalState.get(0);
      }
      try {
        if (formatType == 0) {
          log.error("[Goal] illegal predicate structure for scheduled goal: " + goalState);
          return null;
        } else if (formatType == 1) {
          scheduledDelay = Long.parseLong(delayString);
          scheduledDelayTimeUnit = TimeUnit.valueOf(timeUnitString);
        } else if (formatType == 2) {
          SimpleDateFormat formatter = new SimpleDateFormat("HH-mm-ss", Locale.ENGLISH);
          Date date = null;
          //TODO: what date format do we expect? Is only the LLM able to target this flexibly?
          try {
            date = formatter.parse(dateString);
            Calendar calendar = Calendar.getInstance();
            calendar.set(Calendar.HOUR_OF_DAY,date.getHours());
            calendar.set(Calendar.MINUTE,date.getMinutes());
            calendar.set(Calendar.SECOND,date.getSeconds());
            date = calendar.getTime();
          } catch (ParseException ignored) {}
          if (date == null) {
            formatter = new SimpleDateFormat("MM-dd-yyyyTHH-mm-ss", Locale.ENGLISH);
            date = formatter.parse(dateString);
          }
          scheduledDate = date;
        }
      } catch (Exception e) {
        log.error("[Goal] exception while parsing arguments for scheduled goal: " + goalState, e);
      }
      scheduled = true;

      //Modified Impl:
      //With current implementation of disjoint goal submission due to needing to adhere to resource constraints
      // of inner goal while allowing this goal to be active, need to cut off here to allow for 'scheduled persistent goals'
      // or 'scheduled persistent scheduled  to be possible.
      //return unwrapGoalPredicate(goalState, false);
      return goalState;
    }

    return goalState;
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
