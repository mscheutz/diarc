/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.action;

import edu.tufts.hrilab.action.execution.*;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.lock.ActionResourceLock;

import edu.tufts.hrilab.action.priority.PriorityCalculator;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.fol.Symbol;
import org.slf4j.LoggerFactory;
import org.slf4j.Logger;

import java.util.*;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * <code>ActionInterpreter</code> is the primary step execution module for the
 * robot.
 */
public class ActionInterpreter implements Callable<ActionStatus> {

  private static Logger log = LoggerFactory.getLogger(ActionInterpreter.class);

  private Lock isRunningLock = new ReentrantLock();
  private volatile boolean isRunning = false;

  /**
   * Root of execution (action)
   */
  private final Context rootStep;

  private Lock currentStepLock = new ReentrantLock();

  /**
   * Context that is currently being executed by this ActionInterpreter.
   */
  protected Context currentStep;

  /**
   * Goal that is executed by this ActionInterpreter
   */
  private final Goal goal;

  private double priority = 0;
  private double cost;
  private double benefit;
  private double maxUrgency = 1.0;
  private double minUrgency = 0.0;

  private AtomicBoolean shouldUpdate = new AtomicBoolean(true);

  /**
   * To suspend execution of an ActionInterpreter.
   */
  private Lock suspendLock = new ReentrantLock();
  private Condition suspendCondition = suspendLock.newCondition();
  private AtomicBoolean suspended = new AtomicBoolean(false);
  private AtomicBoolean canceled = new AtomicBoolean(false);

  /**
   * Flag for canceling this ActionInterpreter.
   */
  private AtomicBoolean shouldCancel = new AtomicBoolean(false);

  /**
   * Class for handling action execution.
   */
  private StepExecution stepExecution;

  /**
   * Listeners. Need to be notified when execution status changes.
   */
  private List<ActionListener> listeners = new ArrayList<>();

  /**
   * The action call stack.
   */
  protected ActionStack callStack;

  /**
   * Handles thread(s) that execute ActionInterpreters.
   */
  private final ExecutorService asyncExecutor = Executors.newCachedThreadPool();
  private final Map<Long, Future<ActionStatus>> asyncFutures = new HashMap<>();
  private final Map<Long, ActionInterpreter> asyncAIs = new HashMap<>();

  /**
   * Create a new ActionInterpreter instance to execute a goal.
   *
   * @param goal         Goal
   * @param caller       Caller context (e.g. root context)
   * @param stateMachine state machine to use for execution
   */
  public static ActionInterpreter createInterpreterFromGoal(Goal goal, Context caller, StateMachine stateMachine) {
    return new ActionInterpreter(goal, caller.addEvent(goal, stateMachine));
  }

  /**
   * Create a new ActionInterpreter instance to execute a goal (with specified executionType).
   *
   * @param goal          Goal
   * @param caller        Caller context (e.g. root context)
   * @param stateMachine  state machine to use for execution
   * @param executionType action execution type
   * @return
   */
  public static ActionInterpreter createInterpreterFromGoal(Goal goal, Context caller, StateMachine stateMachine, ExecutionType executionType) {
    return new ActionInterpreter(goal, caller.addEvent(goal, stateMachine, executionType));
  }

  /**
   * Create new ActionInterpreter instance from an event spec.
   *
   * @param goal      Goal
   * @param caller    Caller context (e.g. root context)
   * @param eventSpec The EventSpec describing the initial action
   */

  public static ActionInterpreter createInterpreterFromEventSpec(Goal goal, Context caller, EventSpec eventSpec) {
    Context stepContext = caller.addEvent(eventSpec);

    // Bind predicate arguments to context variables (necessary when instantiating through EventSpec)
    goal.bindPredicate(stepContext);

    return new ActionInterpreter(goal, stepContext);
  }

  /**
   * Create new ActionInterpreter instance from an action context.
   *
   * @param step context to be executed (e.g. root context)
   */
  public static ActionInterpreter createInterpreterFromContext(Goal goal, Context step) {
    return new ActionInterpreter(goal, step);
  }

  /**
   * Create a new ActionInterpreter instance from a context that may have been partially executed
   *
   * @param goal     the goal which the context tree is working toward
   * @param goalRoot the root context for the goal
   * @return the action new interpreter
   */
  public static ActionInterpreter createInterpreterFromExecutionTree(Goal goal, Context goalRoot) {
    ActionInterpreter ai = new ActionInterpreter(goal, goalRoot);
    Context startStep = ai.createExecutionStack(goalRoot);
    ai.setCurrentStep(startStep);
    goal.setCurrentContext(startStep);
    return ai;
  }

  /**
   * Only ActionInterpreter constructor. All AIs should be constructed through
   * a factory method (e.g., createInterpreterFrom___(...)).
   *
   * @param goal
   * @param rootContext
   */
  protected ActionInterpreter(Goal goal, Context rootContext) {
    this.goal = goal;
    this.goal.setActionInterpreter(this);
    this.rootStep = rootContext;
    this.currentStep = rootStep;
    this.finishInitialization();
  }

  /**
   * Finishes the AI initialization. Should be called at the end of every constructor.
   */
  private void finishInitialization() {
    goal.setRootContext(rootStep);
    goal.setCurrentContext(rootStep);

    //TODO: figure out better way to simulate failed goals
    if (!rootStep.getStatus().isFailure() || !goal.getStatus().isFailure()) {
      callStack = new ActionStack(this);
      if (rootStep instanceof ActionContext) {
        ActionContext action = (ActionContext) rootStep;
        cost = action.getCost();
        benefit = action.getBenefit();
        maxUrgency = action.getMaxUrgency();
        minUrgency = action.getMinUrgency();
      }
      stepExecution = new StepExecution(rootStep, callStack);
    } else {
      goal.setStatus(GoalStatus.FAILED);
      goal.setFailConditions(rootStep.getJustification());
      log.info("Action for goal {} failed with GoalStatus: {}", goal.getPredicate(), goal.getStatus());
    }
  }

  /**
   * Create the execution stack from a context tree which may be partially executed
   *
   * @param context a partially executed context tree used to create execution stack
   * @return the starting context step
   */
  protected Context createExecutionStack(Context context) {
    StateMachine sm = context.getStateMachine();
    ActionStatus status = context.getStatus();
    if (context.isSuccess()) {
      return null;
    } else if (status == ActionStatus.APPROVED || status == ActionStatus.CHECKINGAPPROVAL ||
            status == ActionStatus.INITIALIZED || status == ActionStatus.SATISFYING_PRECONDITIONS ||
            status == ActionStatus.SATISFYING_OBLIGATIONS) {
      context.resetContext();
      return context;
    } else {
      if (context.getChildContexts().isEmpty()) { // primitive action -- in progress or failed
        context.resetContext();
        return context;
      } else {
        if (status == ActionStatus.PROGRESS) { // action script in progress
          callStack.before(context);
          for (Context child : context.getChildContexts().getChildrenContexts()) {
            Context step = createExecutionStack(child);
            if (step != null) {
              return step;
            }
          }
          // action script hasn't been executed (hasn't generated action steps)
          context.setStatus(ActionStatus.APPROVED);
          return context;
        } else if (status.isFailure()) {
          if (context.causedFailure()) {
            context.resetContext();
          } else {
            context.setStatus(ActionStatus.PROGRESS); // this probably won't work, but uncertain
            callStack.before(context);
            for (Context child : context.getChildContexts().getChildrenContexts()) {
              Context step = createExecutionStack(child);
              if (step != null) {
                return step;
              }
            }
          }
          return context;
        }
      }
    }
    // this will be reached if status is cancelled / paused / verifying ... /
    context.resetContext();
    return context;
  }

  /**
   * Execute one cycle of the step interpreter. This is currently equivalent
   * to executing one primitive (i.e., progressing through the script until a
   * primitive is found).
   */
  protected Context runCycle(Context step) {
    log.trace("[runCycle] current step: {}", step.getSignatureInPredicateForm());
    Context nextStep;

    //  push context step onto action stack
    callStack.before(step);                 // Prepare step execution

    notifyStepExecution(step);
    nextStep = stepExecution.execute(step);
    notifyStepComplete(step);
    if (nextStep == null) {
      log.debug("Next step is null. Current step was {}. Exiting...", step.getSignatureInPredicateForm());
      halt();
    }
    return nextStep;
  }

  /**
   * Execute one cycle of the step interpreter. This is currently equivalent
   * to executing one primitive (i.e., progressing through the script until a
   * primitive is found).
   */
  protected Context runAsyncCycle(Context step) {
    log.trace("[runAsyncCycle] current step: {}", step.getSignatureInPredicateForm());

    callStack.before(step);

    if (step.getStatus() == ActionStatus.RESUME) {
      // call resume on the existing asyncAI
      asyncAIs.get(step.getId()).resume();
    } else {
      // TODO: it's a bit awkward to create a Goal here, but AIs currently need a goal instance.
      //  Consider getting rid of this requirement?
      Symbol actor = (Symbol) currentStep.getArgumentValue("?actor");
      Goal goal = new Goal(actor, currentStep.getSignatureInPredicateForm());
      ActionInterpreter ai = ActionInterpreter.createInterpreterFromContext(goal, currentStep);
      Future<ActionStatus> asyncFuture = asyncExecutor.submit(ai);
      asyncFutures.put(currentStep.getId(), asyncFuture);
      asyncAIs.put(currentStep.getId(), ai);
    }

    Context nextStep = callStack.next();
    while (nextStep != null) {
      Context tmpStep = stepExecution.getNextStep(nextStep);
      if (tmpStep != null) {
        nextStep = tmpStep;
        break;
      }
      stepExecution.finishStep(nextStep);
      nextStep = callStack.next();
    }

    if (nextStep == null) {
      if (log.isDebugEnabled()) {
        log.debug("Next step is null. Current step was {}. Exiting...", step.getSignatureInPredicateForm());
      }
      halt();
    }
    return nextStep;
  }

  /**
   * Get the root context executed by this ActionIntepreter.
   *
   * @return root context/step
   */
  public Context getRoot() {
    return rootStep;
  }

  /**
   * Get the cost of the step.
   *
   * @return the cost
   */
  public double getActionCost() {
    return cost;
  }

  /**
   * Get the benefit of the step.
   *
   * @return the benefit
   */
  public double getActionBenefit() {
    return benefit;
  }

  /**
   * Get the maximum time allowed the step.
   *
   * @return the max time
   */
  public long getActionMaxTime() {
    currentStepLock.lock();
    try {
      return currentStep.getMaxTime();
    } finally {
      currentStepLock.unlock();
    }
  }

  /**
   * Get the start time for the step.
   *
   * @return the start time
   */
  public long getActionStartTime() {
    currentStepLock.lock();
    try {
      return currentStep.getStartTime();
    } finally {
      currentStepLock.unlock();
    }
  }

  /**
   * Get the maximum urgency allowed the step.
   *
   * @return the max urgency
   */
  public double getActionMaxUrg() {
    return maxUrgency;
  }

  /**
   * Get the minimum urgency allowed the step.
   *
   * @return the min urgency
   */
  public double getActionMinUrg() {
    return minUrgency;
  }

  /**
   * Get the step's current priority.
   *
   * @return the priority
   */
  public double getActionPriority() {
    return priority;
  }

  /**
   * Set the step's current priority.
   *
   * @param newpri the new priority
   */
  protected void setActionPriority(double newpri) {
    priority = newpri;
  }

  public Goal getGoal() {
    return goal;
  }

  /**
   * Whether this ActionInterpreter instance is valid (i.e. can run) or not.
   * It all comes down the the status of the current step.
   *
   * @return true if ActionInterpreter is valid and can execute the action.
   */
  // TODO: should this be the root step, if the action is over, current step will
  //       be null,
  protected boolean isValid() {
    currentStepLock.lock();
    try {
      if (currentStep == null) {
        return false;
      } else {
        return !currentStep.getStatus().isFailure();
      }
    } finally {
      currentStepLock.unlock();
    }
  }

  /**
   * The main ActionInterpreter run method. After running some checks (locks
   * init and such), it will loop while <code>shouldUpdate</code>. Each loop
   * makes a call to <code>runCycle</code> to execute the behavior for a single
   * loop/cycle.
   */
  @Override
  public ActionStatus call() {
    try {
      // Make sure Action Interpreter is valid and not already running
      isRunningLock.lock();
      try {
        if (this.isValid() && !isRunning) {
          // (Try to) Start goal execution with this interpreter
          if (goal.setAsStarted()) {
            log.debug("Started Action Interpreter for goal: {}. Context id: {}", goal.getPredicate(), rootStep.getId());
          } else {
            log.error("Could not start Action Interpreter (goal refused to start): {}", goal.getPredicate());
            return rootStep.getStatus();
          }
        } else {
          log.error("Could not start Action Interpreter (not valid or already running): {}", goal.getPredicate());
          return rootStep.getStatus();
        }

        isRunning = true;
      } finally {
        isRunningLock.unlock();
      }
      // done ensuring single thread is executing an AI at a time

      // send notifications
      notifyStart();

      // main execution loop (wrapped in suspend/resume logic)
      while (!this.goal.getStatus().isTerminated()) {
        if (suspended.get()) {
          // suspend logic
          suspendLock.lock();
          try {
            suspendCondition.await();
          } catch (InterruptedException e) {
            log.warn("Interrupted while waiting for suspend condition.", e);
          } finally {
            suspendLock.unlock();
          }
        } else {
          mainExecutionLoop();
        }
      }

      // goal termination
      ActionResourceLock.deepReleaseAll(this);
      notifyCompletion();
      isRunning = false;
      return rootStep.getStatus();
    } catch (Exception e) {
      log.error("Exception caught during ActionInterpreter execution.", e);
      return ActionStatus.FAIL;
    }
  }

  private void mainExecutionLoop() {
    log.debug("Entering main execution loop. Goal: {} Context: {}", goal, ((currentStep == null) ? "null" : currentStep));

    while (shouldUpdate.get()) {
      goal.setCurrentContext(currentStep);
      Context nextStep;
      if (currentStep.isAsynchronous() && currentStep != rootStep) {
        log.trace("about to call runAsyncCycle: {} ...", currentStep.getSignatureInPredicateForm());
        nextStep = runAsyncCycle(currentStep);
        log.trace("done with runAsyncCycle.");
      } else {
        log.trace("about to call runCycle: {} ...", currentStep.getSignatureInPredicateForm());
        nextStep = runCycle(currentStep);
        log.trace("done with runCycle.");
      }

      // because the cancel/suspend hooks happens asynchronously to the main execution loop, it's possible the currentStep
      // already has a terminal status in the calls to cancel/suspend (and can't be cancelled/suspended). This ensures
      // the cancel/suspend is set on the nextStep and propagated up/down the context tree
      if (suspended.get() && nextStep != null && nextStep.getStatus() != ActionStatus.SUSPEND) {
        nextStep.setStatus(ActionStatus.SUSPEND);
      }  else if (canceled.get() && nextStep != null && nextStep.getStatus() != ActionStatus.CANCEL) {
        nextStep.setStatus(ActionStatus.CANCEL);
      }

      // update current step
      setCurrentStep(nextStep);

      // give other threads a chance to execute
      Thread.yield();
    }
  }

  protected void setCurrentStep(Context step) {
    currentStepLock.lock();
    try {
      currentStep = step;
    } finally {
      currentStepLock.unlock();
    }
  }

  /**
   * Suspend the Action Interpreter's script.
   */
  public void suspend() {
    // lock on the currentStep so cancelling doesn't accidentally cancel an old step
    currentStepLock.lock();
    suspendLock.lock();

    try {
      if (currentStep == null) {
        log.warn("[suspend] AI for goal has already terminated: {}", goal.getPredicate());
      } else if (currentStep.isTerminated()) {
        // setting suspended flag will set nextStep to suspended in the mainLoop before it's executed
        suspended.set(true);
      } else {
        suspended.set(true);
        currentStep.setStatus(ActionStatus.SUSPEND);
      }
    } finally {
      currentStepLock.unlock();
      suspendLock.unlock();
    }
  }

  /**
   * Resume the Action Interpreter's script after suspending.
   */
  public void resume() {
    // lock on the currentStep in case suspend has not fully completed
    currentStepLock.lock();
    suspendLock.lock();

    try {
      if (suspended.getAndSet(false)) {
        if (currentStep == null) {
          // goal has been fully suspended
          rootStep.setStatus(ActionStatus.RESUME);
          setCurrentStep(rootStep);
          shouldUpdate.set(true); // re-enable mainExecutionLoop
          suspendCondition.signal();  // wake up main AI thread
        } else {
          // goal not full suspended -- resume from current step
          log.debug("Resuming from partially suspended goal: {}", goal);
          currentStep.setStatus(ActionStatus.RESUME);
        }
      } else {
        log.warn("[resume] trying to resume an ActionInterpreter that wasn't paused.");
      }
    } finally {
      currentStepLock.unlock();
      suspendLock.unlock();
    }
  }

  /**
   * Sets the shouldCancel flag to true, which will cause the main AI loop to
   * set the next step's ActionStatus to CANCEL. This does not explicitly
   * halt the ActionInterpreter, but instead relies on the handling of the
   * CANCEL status by the currently running action. This allows for graceful
   * handling of canceled actions (e.g., via try/catch/finally). This is the
   * only option for external classes.
   */
  public void cancel() {
    // lock on the currentStep so cancelling doesn't accidentally cancel an old step
    currentStepLock.lock();

    try {
      // cancel logic
      if (currentStep == null) {
        if (!suspended.get()) {
          log.warn("[cancel] goal has already terminated: {}", goal.getPredicate());
        }
      } else if (currentStep.isTerminated()) {
        // setting cancel flag will set nextStep to cancel in the mainLoop before it's executed
        canceled.set(true);
      } else {
        // in normal operation
        canceled.set(true);
        currentStep.setStatus(ActionStatus.CANCEL);
      }
    } finally {
      currentStepLock.unlock();
    }

    // if canceling from a suspended state, we need to unblock the suspend
    if (suspended.get()) {
      suspendLock.lock();
      try {
        suspendCondition.signal();
      } finally {
        suspendLock.unlock();
      }
    }
  }

  /**
   * Halt the Action Interpreter. This is called when the script has no more
   * events.
   */
  private void halt() {
    if (!shouldUpdate.get()) {
      log.error("[halt] halt has already been called: {}", goal.getPredicate());
      return;
    }

    // TODO: can suspended be replaced with (rootStep.getStatus() == ActionStatus.SUSPEND)
    if (rootStep.getStatus() == ActionStatus.SUSPEND) {
      suspended.set(true);
    }

    // Stop execution mainExecutionLoop()
    shouldUpdate.set(false);

    // wait for sub-AIs to finish
    // sub-AIs stay alive during suspend, so they can be resumed
    if (rootStep.getStatus() != ActionStatus.SUSPEND) {
      for (Long contextId : asyncFutures.keySet()) {
        Future<ActionStatus> currFuture = asyncFutures.get(contextId);
        if (!currFuture.isDone()) {
          log.warn("[halt] waiting for asynchronous context to finish. Goal: {} Context id: {}", goal.getPredicate(), contextId);
          try {
            currFuture.get();
            log.warn("[halt] asynchronous context finished. Goal: {} Context id: {}", goal.getPredicate(), contextId);
          } catch (InterruptedException | ExecutionException e) {
            log.error("[halt] exception waiting for asynchronous context to finish. Goal: {} Context id: {}", goal.getPredicate(), contextId);
          }
        }
      }
    }

    // Update goal end status
    ActionStatus rootActionStatus = rootStep.getStatus();
    if (rootActionStatus == ActionStatus.SUCCESS) {
      goal.setStatus(GoalStatus.SUCCEEDED);
    } else if (rootActionStatus == ActionStatus.CANCEL) {
      goal.setStatus(GoalStatus.CANCELED);
    } else if (rootActionStatus == ActionStatus.SUSPEND) {
      goal.setStatus(GoalStatus.SUSPENDED);
    } else if (rootActionStatus.isFailure()) {
      goal.setStatus(GoalStatus.FAILED);
      goal.setFailConditions(rootStep.getJustification());
    } else {
      log.error("[halt] Could not convert ActionStatus to GoalStatus: {} for action: {}", rootActionStatus, goal.toString());
    }

    if (goal.getStatus().isFailure()) {
      log.info("[halt] goal terminated: {} GoalStatus: {} {}: {}."
              , goal.getPredicate(), goal.getStatus(), goal.getId(), goal.getFailConditions());
    } else if (log.isDebugEnabled()) {
      log.debug("[halt] goal terminated: {} GoalStatus: {} {}: {}."
              , goal.getPredicate(), goal.getStatus(), goal.getId(), goal.getFailConditions());
    }
  }

  public void updatePriority(PriorityCalculator calc) {
    priority = calc.calculate(this);
  }

  public void addListener(ActionListener al) {
    listeners.add(al);
  }

  private void notifyStart() {
    for (ActionListener al : listeners) {
      al.actionStarted(this);
    }
  }

  private void notifyCompletion() {
    for (ActionListener al : listeners) {
      al.actionComplete(this);
    }
  }

  private void notifyStepExecution(Context step) {
    for (ActionListener al : listeners) {
      al.stepStarted(step);
    }
  }

  private void notifyStepComplete(Context step) {
    for (ActionListener al : listeners) {
      al.stepComplete(step);
    }
  }

  public Context getCurrentStep() {
    return currentStep;
  }

}
