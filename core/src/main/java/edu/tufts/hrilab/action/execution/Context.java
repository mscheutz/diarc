/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author willie
 * @author luca
 * @author tyler frasca
 */
package edu.tufts.hrilab.action.execution;

import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.ActionConstraints;
import edu.tufts.hrilab.action.ScriptParser;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.description.ContextDescription;
import edu.tufts.hrilab.action.ActionInterpreter;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.Observable;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.db.OperatorDBEntry;
import edu.tufts.hrilab.action.execution.control.ControlFactory;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.util.Utilities;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import static java.lang.String.format;

public abstract class Context {

  protected final Logger log;
  /**
   * Used to assign unique IDs so each Context instance.
   */
  private static AtomicLong nextId = new AtomicLong(0);

  /**
   * Each Context has a parent context from which the context is generated.
   * The only exception is that there is one root context with no parent.
   * An action being executed as the result of a submitted goal will derive
   * from the root context. An event within an action that invokes will have
   * its own context that is a child of the context for the action script.
   */
  protected final Context caller;
  /**
   * The list of childContexts is used to track the history of the execution.
   */
  protected final ChildContexts childContexts = new ChildContexts();
  /**
   * Execution type: act, simulate but act or full simulation
   */
  private ExecutionType execType;
  /**
   * The system time at which this action started.
   */
  protected long startTime;
  /**
   * The system time at which this action ended.
   */
  protected long endTime;
  /**
   * The system time at which this action must stop. See: {
   *
   * @see ActionContext#setTimeout}
   */
  protected long maxTime;
  /**
   * Unique id of this context.
   */
  private final Long id;
  /**
   * Command.
   */
  protected final String cmd;
  /**
   * Status of this Context.
   */
  private ActionStatus actionStatus;
  /**
   * A boolean value for this context.
   */
  private boolean logicalValue;
  /**
   * Justification for the status of this context.
   */
  private Justification justification;
  /**
   * StateMachine for context.
   */
  protected StateMachine stateMachine;
  /**
   * ActionStatus lock.
   */
  private Lock statusLock = new ReentrantLock();
  /**
   * ActionStatus lock condition that is signalled when the status changes.
   */
  private Condition terminalCondition = statusLock.newCondition();

  /**
   * The constraints for selecting an action in this context (and child contexts).
   * The constraints include forbidden actions and forbidden contexts. If an
   * action or context is to be temporarily retracted as being forbidden, create a
   * new ActionConstraints without the retracted constraint.
   *
   * For example, if permission has been given to do a forbidden action (e.g.
   * push an obstacle) then the constraint will not be present in the new
   * ActionConstraints and child context will also not have this constraint.
   * However, once context returns to the parent of the context, the constraint is
   * back in place. This allows for constraints to be restricted to a certain
   * context.
   */
  protected ActionConstraints constraints;

  /**
   * Context constructor.
   *
   * @param parent parent context
   * @param sm     state machine for this context
   * @param cmd    command of this context
   */
  public Context(Context parent, StateMachine sm, String cmd) {
    this.log = LoggerFactory.getLogger(this.getClass());
    this.id = nextId.getAndIncrement();
    this.caller = parent;
    this.cmd = cmd;
    this.stateMachine = sm;
    if (parent != null) {
      this.constraints = parent.constraints;
      this.execType = parent.execType;
    } else {
      this.execType = ExecutionType.ACT;
    }
    this.logicalValue = true;
    initializeStatus();
  }

  /**
   * Context constructor that explicitly sets the execution type.
   *
   * @param c        parent context
   * @param sm       state machine for this context
   * @param cmd      command of this context
   * @param execType execution type
   */
  protected Context(Context c, StateMachine sm, String cmd, ExecutionType execType) {
    this(c, sm, cmd);
    this.execType = execType;
  }

  /**
   * Initialize context's action status and default justification.
   */
  private void initializeStatus() {
    setStatus(ActionStatus.INITIALIZED);
    justification = new ConditionJustification(true);
  }

  /**
   * Get context's unique id.
   *
   * @return
   */
  public Long getId() {
    return id;
  }

  /**
   * Acquire all locks for this context
   *
   * @param actionInt The AI that needs the lock
   * @param greedy    if not greedy, then release the locks if all can't be acquired
   * @return true is all locks acquired, false otherwise
   */
  public boolean acquireLocks(ActionInterpreter actionInt, boolean greedy) {
    return true;
  }

  /**
   * Release all locks for this context
   *
   * @param actionInt The AI that needs the lock
   */
  public void releaseLocks(ActionInterpreter actionInt) {
  }

  public ActionStatus getStatus() {
    try {
      statusLock.lock();
      return actionStatus;
    } finally {
      statusLock.unlock();
    }
  }

  /**
   * Set the ActionStatus and Justification for the status.
   *
   * NOTE: Classes overriding this method must call super.setStatus(status, justification)
   * inside their overriding method to actually set the status and justification. Using the convenience
   * method setStatus(status) will cause an infinite loop, and also abandon the Justification.
   *
   * @param status
   * @param justification
   */
  public void setStatus(ActionStatus status, Justification justification) {
    log.debug("Setting context " + getSignatureInPredicateForm() + " to status " + status);

    try {
      statusLock.lock();

      if (status == ActionStatus.INITIALIZED) {
        // allow context to be (re)set to INITIALIZED, which is required for a context to be reset for re-use
        actionStatus = status;
        this.justification = justification;
      } else if (actionStatus != null && actionStatus.isTerminated()) {
        // duplicate setting of FAIL_ANCESTOR can happen when failure initially propagates down context tree
        // (e.g., overall condition failure), and then propagates back up context tree (normal failure propagation)
        if (status != ActionStatus.FAIL_ANCESTOR) {
          log.warn("Can not set context " + cmd + " to status " + status + ". Already has terminal status: " + actionStatus);
        }
        return;
      } else if (status == ActionStatus.SUCCESS || status == ActionStatus.APPROVED || status == ActionStatus.CANCEL || status.isFailure()) {
        actionStatus = status;
        this.justification = justification;
      } else {
        actionStatus = status;
        this.justification = justification;
      }

      if (isTerminated()) {
        // make sure non-async children don't continue to execute (e.g., when a parent fails overall conditions)
        if (isFailure()) {
          childContexts.forEach(child -> {
            if (!child.isAsynchronous() && child.getStatus() != ActionStatus.INITIALIZED && !child.isTerminated()) {
              child.setStatus(ActionStatus.FAIL_ANCESTOR);
            }
          });
        }

        // make sure all children have finished execution (in the case of asynchronous children)
        if (childContexts.anyMatch(child -> child.isAsynchronous() && !child.isTerminated())) {
          log.debug("[setStatus] not setting status to " + actionStatus + " because there's still an active asynchronous child. Context: " + this);
          actionStatus = ActionStatus.ACTIVE_CHILD;
          // don't send terminalCondition signal in this case
        } else if (caller.getStatus() == ActionStatus.ACTIVE_CHILD) {
          // if parent is waiting on child(ren), try to propagate status up context tree
          caller.setStatus(actionStatus);
          terminalCondition.signalAll();
        } else {
          terminalCondition.signalAll();
        }
      }

    } finally {
      statusLock.unlock();
    }
  }

  /**
   * Convenience method to set the ActionStatus without needing to provide a justification. This is
   * useful for ActionStatus cases where a justification doesn't make sense (e.g., PROGRESS, SUCCESS, etc).
   *
   * Callers of setStatus should prefer to use the setStatus that takes a Justification whenever possible.
   *
   * @param status
   */
  public final void setStatus(ActionStatus status) {
    log.debug("Setting context " + getSignatureInPredicateForm() + " to status " + status + " with justification: " + justification);
    setStatus(status, this.justification);
  }

  /**
   * Goes through the call tree, checking that all ancestors are currently running. If an ancestor
   * has terminated, that ancestor (i.e., Context) is returned. This is used to determine if
   * asynchronous contexts can/should run (async context or overall condition). This is necessary
   * since parent contexts that are not currently executed might suddenly fail due to overall conditions,
   * or overall conditions might be in the middle of an execution step when a parent finishes (with fail or success).
   *
   * @return Context that has terminated status
   */
  protected Context getNearestTerminatedAncestor() {
    if (this.isTerminated()) {
      return this;
    } else if (caller == null) {
      return null;
    } else {
      return caller.getNearestTerminatedAncestor();
    }
  }

  /**
   * Reset Context instance to enable it to be used by an ActionInterpreter.
   * This is useful for cases when an ActionInterpreter needs to be used repeatedly
   * without creating new Contexts every run of the ActionInterpreter.
   */
  public final void resetContext() {
    resetConcreteContext();
    resetCoreContext();
  }

  /**
   * This resets fields specific to concrete sub-classes and should
   * be overridden by inheriting classes if additional resetting
   * needs to happen.
   */
  protected void resetConcreteContext() {
    // intentionally left empty -- this method is only
    // for inheriting classes that need to reset local fields
  }

  /**
   * This resets the base Context class's fields and children.
   */
  protected final void resetCoreContext() {
    logicalValue = true;
    startTime = 0;
    endTime = 0;
    initializeStatus();
    childContexts.reset();
  }

  /**
   * Set the logical value for this event.
   *
   * @param value the logical value of the event
   */
  protected void setLogicalValue(boolean value) {
    logicalValue = value;
  }

  /**
   * Gets the logical value for this event. Typically used in controls such as if, while, not...
   *
   * @return the logical value of this execution
   */
  public boolean getLogicalValue() {
    // The logical value of an Context is false if the execution did not succeed
    // If the execution succeeded, use the logical value field.
    return (this.getStatus() == ActionStatus.SUCCESS && logicalValue);
  }

  protected boolean getActualLogicalValue() {
    return logicalValue;
  }

  /**
   * Returns the justification (reason) for the context's execution status
   *
   * @return jusitifcation
   */
  public Justification getJustification() {
    return justification;
  }

  /**
   * If Context has a concrete execution step to perform, it should happen in this method.
   * This should not include setting up next steps (i.e., children to perform), which should
   * happen in the setupNextStep method.
   *
   * Primitive actions and operators (i.e., leaf nodes in the context tree) are examples
   * where this method should be implemented.
   */
  public void doStep() {
    if (log.isDebugEnabled()) {
      log.debug("[doStep] " + this);
    }
  }

  /**
   * Get next step in the execution of this context. This context must have
   * ActionStatus of PROGRESS or RECOVERY (i.e., currently being executed) to return
   * a non-null context.
   *
   * @return
   */
  public final Context getNextStep() {
    Context next = null;
    if (getStatus() == ActionStatus.PROGRESS || getStatus() == ActionStatus.RECOVERY) {
      setupNextStep();
      next = getNextStepForType();
    }
    return next;
  }

  /**
   * If Context can have next steps (i.e., children), implement this method to add the next child context that should
   * be executed.
   */
  protected void setupNextStep() {
    // intentionally empty
  }

  /**
   * Get the next step (i.e., context) to be executed. Returns null if there are no more steps.
   *
   * TODO: remove this method
   *
   * @return
   */
  protected Context getNextStepForType() {
    return childContexts.getNextAndIncrement();
  }

  /**
   * Verify that the effects (if any) of a context are valid.
   *
   * @return
   */
  protected Justification verifyEffects() {
    return new ConditionJustification(true);
  }

  /**
   * Verify that the return value (if any) of a context is valid.
   *
   * @return
   */
  public Justification verifyReturnValue() {
    return new ConditionJustification(true);
  }

  /**
   * Get the signature of the Context in Predicate form.
   * Most Context implementations should override this method.
   *
   * @return
   */
  public Predicate getSignatureInPredicateForm() {
    return Factory.createPredicate(this.cmd, new ArrayList<>());
  }

  /**
   * Get the ContextDescription for this context tree based on the location predicate.
   *
   * This searches down the context tree until some context
   * matches the location predicate and returns a ContextDescription.
   *
   * @return
   */
  @Action
  public ContextDescription getContextDescription(Predicate location) {
    ContextDescription contextDescription = null;
    Context currentChild = childContexts.getCurrent();
    if (currentChild != null) {
      contextDescription = currentChild.getContextDescription(location);
    } else {
      log.warn("[getContextDescription(location)] no context description available." +
              "this: " + this +
              "location: " + location +
              "numChildren: " + childContexts.size());
    }
    return contextDescription;
  }

  /**
   * Get the ContextDescription for this Context.
   *
   * TODO: this currently searches down the context tree until some context (e.g., ActionContext)
   * returns a ContextDescription.
   * Eventually, this should probably be abstract, to force every context class to return
   * a ContextDescription.
   *
   * @return
   */
  public ContextDescription getContextDescription() {
    ContextDescription contextDescription = null;
    Context currentChild = childContexts.getCurrent();
    if (currentChild != null) {
      contextDescription = currentChild.getContextDescription();
    } else {
      log.warn("[getContextDescription] no context description available." +
              "this: " + this.toString() +
              "numChildren: " + childContexts.size());
    }
    return contextDescription;
  }

  /**
   * Retrieve argument value. Generic implementation just recurses up the context tree.
   * Even though some context instances don't have arguments themselves, it's important for calls to
   * pass through non-argument contexts to search for arguments in other parts of the context tree.
   */
  public Object getArgumentValue(String bname) {
    return (caller != null) ? caller.getArgumentValue(bname) : null;
  }

  /**
   * Retrieve argument (ActionBinding). Generic implementation just recurses up the context tree.
   * Even though some context instances don't have arguments themselves, it's important for calls to
   * pass through non-argument contexts to search for arguments in other parts of the context tree.
   */
  protected ActionBinding getEvaluatedArgument(String bname) {
    return (caller != null) ? caller.getEvaluatedArgument(bname) : null;
  }

  /**
   * Get list of cloned arguments.
   * Even though some context instances don't have arguments themselves, it's important for calls to
   * pass through non-argument contexts to search for arguments in other parts of the context tree.
   *
   * @return
   */
  public List<ActionBinding> getArguments() {
    return (caller != null) ? caller.getArguments() : null;
  }

  /**
   * Retrieve argument type. Generic implementation just recurses up the context tree.
   * Even though some context instances don't have arguments themselves, it's important for calls to
   * pass through non-argument contexts to search for arguments in other parts of the context tree.
   */
  protected Class<?> getArgumentType(String bname) {
    return (caller != null) ? caller.getArgumentType(bname) : null;
  }

  /**
   * Retrieve binding. Generic implementation just recurses up the context tree.
   * Even though some context instances don't have arguments themselves, it's important for calls to
   * pass through non-argument contexts to search for arguments in other parts of the context tree.
   */
  protected ActionBinding getArgument(String bname) {
    return (caller != null) ? caller.getArgument(bname) : null;
  }

  /**
   * Same as getArgument method but does not generate a log statement in the
   * case the argument is not found. This is useful for method like hasArgumentInScope(bname)
   * so that confusing log error statements aren't made.
   *
   * @param bname
   * @return
   */
  protected ActionBinding getArgumentSilent(String bname) {
    return (caller != null) ? caller.getArgumentSilent(bname) : null;
  }

  /**
   * Get a local argument without recursing up the context tree. Only classes
   * that extend ArgumentBasedContext will have local arguments, so this
   * return null by default.
   *
   * @param bname
   * @return
   */
  protected ActionBinding getLocalArgument(String bname) {
    return null;
  }

  /**
   * Helper method to find actor value in the context tree.
   *
   * @return
   */
  protected Symbol getActor(String actorVarName) {
    Object actor = this.getArgumentValue(actorVarName);
    if (actor == null) {
      if (Utilities.isScriptVariable(actorVarName)) {
        // actor is not yet bound
        return Factory.createVariable(actorVarName);
      } else {
        log.error(actorVarName + " value does not exist. Defaulting to \"self\".");
        return Factory.createSymbol("self");
      }
    } else if (!(actor instanceof Symbol)) {
      log.error(actorVarName + " is not of type Symbol. Attempting to covert actor to Symbol: " + actorVarName);
      return Factory.createSymbol(actor.toString());
    } else {
      return (Symbol) actor;
    }
  }

  /**
   * Get the start time
   *
   * @return the action's start time
   */
  public long getStartTime() {
    return startTime;
  }

  /**
   * Set the start time
   *
   * @param stime start time
   */
  protected void setStartTime(long stime) {
    startTime = stime;
  }

  protected void setStartTime() {
    if (startTime == 0) {
      startTime = System.currentTimeMillis();
      log.debug("setting start time for " + getSignatureInPredicateForm() + " " + startTime);
    } else {
      log.debug("resuming execution of " + getSignatureInPredicateForm() + " " + startTime);
    }
  }

  protected void setEndTime(long etime) {
    endTime = etime;
    log.debug("setting end time for " + getSignatureInPredicateForm() + " " + getStatus() + " duration " + getDuration());
  }

  public long getEndTime() {
    if (endTime == 0) {
      // probably not what should happen
      return getElapsedTime();
    }
    return endTime;
  }

  public long getDuration() {
    if (endTime == 0 && startTime != 0) {
      return System.currentTimeMillis() - startTime;
    }
    return endTime - startTime;
  }

  /**
   * Get the elapsed time
   *
   * @return the action's elapsed time
   */
  protected long getElapsedTime() {
    return System.currentTimeMillis() - startTime;
  }

  /**
   * Gets the maximum time this action should run until
   *
   * @return The latest time this action should run
   */
  public long getMaxTime() {
    return maxTime;
  }

  protected void setTimeout() {
    this.maxTime = caller.maxTime;
    this.startTime = caller.startTime;
  }

  public boolean isAction() {
    return false;
  }

  public boolean isAsynchronous() {
    return false;
  }

  public boolean isSimulation() {
    return (execType.isSimulation());
  }

  public ActionConstraints getConstraints() {
    return constraints;
  }

  public void setConstraints(ActionConstraints ac) {
    this.constraints = ac;
  }


  public Justification isApproved() {
    return new ConditionJustification(true);
  }

  protected void startOverAllMonitor() {
  }

  @Override
  public String toString() {
    return "(context " + cmd + ")";
  }

  public String getCommand() {
    return cmd;
  }

  protected void setExecType(ExecutionType execType) {
    this.execType = execType;
  }

  public ExecutionType getExecType() {
    return execType;
  }

  public boolean isFailure() {
    try {
      statusLock.lock();
      return actionStatus.isFailure();
    } finally {
      statusLock.unlock();
    }
  }

  public boolean isSuccess() {
    try {
      statusLock.lock();
      return actionStatus == ActionStatus.SUCCESS;
    } finally {
      statusLock.unlock();
    }
  }

  public boolean isTerminated() {
    try {
      statusLock.lock();
      return actionStatus != null && actionStatus.isTerminated();
    } finally {
      statusLock.unlock();
    }
  }

  /**
   * Get parent context.
   *
   * @return
   */
  public Context getParentContext() {
    return caller;
  }

  /**
   * Get all children contexts in a copied ChildContexts instance. This is a shallow copy of
   * the underlying child Contexts.
   *
   * @return
   */
  public ChildContexts getChildContexts() {
    return childContexts.getCopy();
  }

  /**
   * A method to perform additional steps during ActionStatus updates (i.e., setStatus). If a particular Context sub-class
   * needs this to happen, that class should override this method.
   *
   * NOTE: This method is only called when a select set of ActionStatuses are successfully set.
   */
  protected void performAdditionalStatusUpdates() {
  }

  protected void updatePerformanceModels() {
  }

  /**
   * Helper method to check if this context is the root of the failure (return true),
   * * or if a child context is the cause of the failure (return false).
   *
   * @return
   */
  public boolean causedFailure() {
    if (getStatus().isFailure()) {
      return !getChildContexts().anyMatch(child -> child.isFailure());
    }
    return false;
  }

  /**
   * Get state updates for this context based on the current status (i.e., ActionStatus).
   *
   * @return
   */
  public List<Predicate> getStateUpdates() {
    return new ArrayList<>();
  }

  /**
   * This method blocks until this context is terminated.
   */
  public void waitForTermination() {
    try {
      statusLock.lock();
      if (!isTerminated()) {
        try {
          log.debug("[waitForTermination] waiting ...");
          terminalCondition.await();
          log.debug("[waitForTermination] ... done waiting.");
        } catch (InterruptedException e) {
          log.error("[waitForTermination] interrupted while waiting for termination.", e);
        }
      }
    } finally {
      statusLock.unlock();
    }
  }

  /**
   * This method blocks up to maxTime for this context to reach a terminal ActionStatus.
   *
   * @param timeout max milliseconds to wait
   */
  public void waitForTermination(long timeout) {
    try {
      statusLock.lock();
      if (!isTerminated()) {
        try {
          log.info("[waitForTermination(timeout)] waiting ...");
          terminalCondition.await(timeout, TimeUnit.MILLISECONDS);
          log.info("[waitForTermination(timeout)] ... done waiting.");
        } catch (InterruptedException e) {
          log.info("[waitForTermination(timeout)] interrupted while waiting for termination.", e);
        }
      }
    } finally {
      statusLock.unlock();
    }
  }

  /**
   * TODO: remove this method. only here as a temporary fix to observation
   * contexts being added as child contexts (see Condition's observeCondition method).
   *
   * @return
   */
  public Context getRootContext() {
    if (caller != null) {
      return caller.getRootContext();
    }
    return this;
  }

  public final Context setupNextStep(ScriptParser parser) {
    if (parser.isEmpty()) {
      log.warn("[setupNextStep] script is empty.");
      return null;
    }

    if (parser.getType() == EventSpec.EventType.CONTROL) {
      // Controls are not created in the same way as actions/operators
      // as they need to access subsequent script elements to control execution flow.
      Context control = ControlFactory.build(this, this.stateMachine, parser);
      if (control == null) {
        control = new FailureContext(this, this.stateMachine, ActionStatus.FAIL_SYNTAX, null);
      }
      childContexts.add(control);
      return control;
    } else {
      return addEvent(parser.getEventSpec());
    }
  }

  protected boolean hasNormCheckingAncestor() {
    if (this.caller == null) {
      return false;
    } else if (this instanceof NormCheckingActionContext) {
      return true;
    }
    return this.caller.hasNormCheckingAncestor();
  }

  /**
   * Instantiate the event specified in the event specification
   *
   * @param eSpec the event to be executed
   */
  public Context addEvent(EventSpec eSpec) {
    if (log.isDebugEnabled()) {
      log.debug("Adding event according to specification: " + eSpec.toString() + " exec: " + this.execType);
    }

    String command = eSpec.getCommand();
    String eventSpecActor = eSpec.getActor();

    // get actor
    Symbol actor;
    if (Utilities.isScriptVariable(eventSpecActor)) {
      actor = getActor(eventSpecActor);
    } else {
      // actor is an actor name (e.g., dempster), create Symbol
      // TODO: should this set semantic type (i.e., AGENT)?
      actor = Factory.createSymbol(eventSpecActor);
    }

    // Get types of arguments, whenever possible. If argument not found, defaulting to string representation.
    List<Class<?>> inputArgTypes = new ArrayList<>();
    List<String> inputArgs = eSpec.getInputArgs();
    for (String arg : inputArgs) {
      Class cls = getArgumentType(arg);
      if (cls == null) {
        log.warn("[addEvent] null role type returned for argument: " + arg + " Defaulting to String type.");
        cls = String.class;
      }
      inputArgTypes.add(cls);
    }

    Context c = null;

    // look up event spec in respective database based on type -> create child context
    switch (eSpec.getType()) {
      // action DB or postcondition DB
      case ACTION:
        ActionDBEntry action = Database.getActionDB().getAction(command, actor, inputArgTypes);

        if (action != null) {
          log.debug("making action context in Context.addEvent");
          // DMK: This is a hack for the norm-following supermarket stuff; will need to find another way to do this.
          if (this.hasNormCheckingAncestor()) {
            c = new NormCheckingActionContext(this, this.stateMachine, action, inputArgs, actor);
          } else {
            c = new ActionContext(this, this.stateMachine, action, inputArgs, eSpec.getReturnArgs(), actor);
          }
        } else {
          log.error("No action found for event " + eSpec);
          c = generateFailureContext(actor, command, inputArgs);
        }
        break;

      case GOAL:
        // create new context for sub-goal
        Symbol goalState;
        if (Utilities.isScriptVariable(command)) {
          goalState = Factory.createVariable(command);
        } else {
          goalState = Factory.createPredicate(command, inputArgs.toArray(new String[inputArgs.size()]));
        }
        Goal goal = new Goal(actor, goalState);
        c = new GoalContext(this, this.stateMachine, goal, this.execType);
        break;

      // operator DB
      case OPERATOR:
        OperatorDBEntry operator = Database.getOperatorDB().getOperator(command, inputArgTypes);
        if (operator != null) {
          c = new OperatorContext(this, this.stateMachine, operator, eSpec.getInputArgs(), eSpec.getReturnArgs());
        } else {
          log.error("No operator in DB for event " + eSpec);
          c = generateFailureContext(actor, command, inputArgs);
        }
        break;

      case TSC:
        // create new context for trade service call
        c = new TradeServiceContext(this, this.stateMachine, command, inputArgs, eSpec.getReturnArgs(), actor);
        break;

      // observation DB
      case OBSERVATION:
        // create new context for observation
        c = new ObservationContext(this, this.stateMachine, command, inputArgs, actor, Observable.TRUE);
        break;

      case CONTROL:
        log.error("Error: addEvent is not compatible with controls");
        break;
    }

    //add new child context to childContexts
    childContexts.add(c);
    return c;
  }

  public Context addEvent(Goal g, StateMachine sm) {
    return addEvent(g, sm, ExecutionType.ACT);
  }

  /**
   * Instantiate the event specified in the goal
   *
   * @param goal          the goal to be pursued
   * @param sm            the state machine in charge of tracking states for this goal
   * @param executionType
   * @return the newly generated action context / child.
   */
  public Context addEvent(Goal goal, StateMachine sm, ExecutionType executionType) {
    if (log.isDebugEnabled()) {
      log.debug(format("Adding event according to goal: %s", goal));
    }

    Context child = new GoalContext(this, sm, goal, executionType);

    childContexts.add(child);
    return child;
  }

  /**
   * Helper method to create FailureContext with a FAIL_NOTFOUND ActionStatus.
   *
   * @param actor
   * @param command
   * @param inputArgs
   * @return
   */
  private FailureContext generateFailureContext(Symbol actor, String command, List<String> inputArgs) {
    List<Symbol> args = new ArrayList<>();
    args.add(actor); // add actor as first arg (not in arguments from eventSpec)
    for (String arg : inputArgs) {
      if (Utilities.isScriptVariable(arg)) {
        Symbol boundArg = Utilities.createFOL(getEvaluatedArgument(arg));
        if (boundArg == null) {
          args.add(Factory.createVariable(arg));
        } else {
          args.add(boundArg);
        }
      } else {
        args.add(Factory.createSymbol(arg));
      }
    }

    Predicate p = Factory.createPredicate(command, args);
    Justification just = new ConditionJustification(false, Factory.createPredicate("found", actor, p));
    // if no action is found during simulate capability we want it to still execute
    return new FailureContext(this, this.stateMachine, ActionStatus.FAIL_NOTFOUND, just, ExecutionType.ACT);
  }

  public StateMachine getStateMachine() {
    return stateMachine;
  }

  /**
   * deep copy of the context, its status, and its children
   *
   * @param newParent parent to attach the copied context too
   * @return the copied context
   */
  public abstract Context copy(Context newParent);

  protected void copyInternal(Context newContext) {
    newContext.actionStatus = getStatus();
    newContext.justification = getJustification();
    newContext.logicalValue = logicalValue;
    newContext.startTime = startTime;
    newContext.endTime = endTime;
    newContext.childContexts.setNextIndex(childContexts.getNextIndex());
    for (Context child : childContexts.getChildrenContexts()) {
      newContext.childContexts.add(child.copy(newContext));
    }
  }

  //This might be unsafe! But I need it for setting a copied tree to a simulated state machine
  protected void setStateMachine(StateMachine sm) {
    this.stateMachine = sm;
    for (Context child : childContexts.getChildrenContexts()) {
      child.setStateMachine(sm);
    }
  }

  protected void setChildContexts(ChildContexts childContexts) {
    this.childContexts.clear();
    this.childContexts.setChildren(childContexts);
  }

  /**
   * Creates a copy of this context tree to be used for simulation.
   * Used when you don't want to modify a context tree, but need to do some safe operations on it.
   *
   * @param belief If believe should be used in the new tree
   * @return The root of the new simulated context tree
   */
  public Context createSimulatedEquivalent(boolean belief) {
    //todo: set state machine to sim state machine from supermarketcomponenntimpl
    Context newRoot;
    getStateMachine().mergeBelief();
    StateMachine simStateMachine = new StateMachine(getStateMachine(), belief);
    Context oldRoot = getRootContext();
    newRoot = ((RootContext) getRootContext()).createSimulationRoot(simStateMachine, ExecutionType.SIMULATE_ACT);
    oldRoot.copyInternal(newRoot);
    newRoot.setStateMachine(simStateMachine);

    //this is super fragile - requires identical context tree toplogies (fine here, because we just made newRoot)
    return getEquivalentContext(this, newRoot);
  }

  /**
   * Given a context, find its location in its context tree. Then use that map topology to find an identical context in a different tree.
   * Assumes that the oldContext's tree and the newRoot's tree have identical topologies!
   * This is typically used for finding a context from a real tree in an equivalent simulated tree
   * //todo: This can maybe be done differently if context IDs are copied as well. That way, map topology doesn't matter.
   *
   * @param oldContext A reference context, to be found in a different tree
   * @param newRoot    New tree to find the copy of oldContext in
   * @return A context that is a different instance of, but otherwise identical to, the oldContext
   */
  protected static Context getEquivalentContext(Context oldContext, Context newRoot) {
    List<Integer> path = new ArrayList<>();
    Context context = oldContext;
    while (!(context.caller instanceof RootContext)) { //todo: this is a temporary fix for diff roots
      path.add(context.caller.childContexts.getChildrenContexts().indexOf(context));
      context = context.caller;
    }
    path.add(context.caller.childContexts.getChildrenContexts().indexOf(context)); //add simroot
    context = newRoot;
//        Context should be root now.
    for (int i = path.size() - 1; i >= 0; i--) {
      context = context.getChildContexts().get(path.get(i));
    }
    return context;
  }


}
