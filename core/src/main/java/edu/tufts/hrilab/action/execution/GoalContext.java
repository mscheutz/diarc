/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.execution;

import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.Observable;
import edu.tufts.hrilab.action.ParameterizedAction;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.execution.util.ContextUtils;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.recovery.*;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;

import java.util.List;

public class GoalContext extends ArgumentBasedContext {
  protected Goal goal;
  protected ExecutionType executionType;
  protected GoalRecovery goalRecovery;

  public GoalContext(Context caller, StateMachine sm, Goal goal, ExecutionType executionType, GoalRecovery recovery) {
    super(caller, sm, "goal(" + goal.toString() + ")");
    this.goal = goal;
    this.executionType = executionType;
    setupArguments(null, null);
    goalRecovery = recovery;
  }

  public GoalContext(Context caller, StateMachine sm, Goal goal, ExecutionType executionType) {
    this(caller, sm, goal, executionType, new ExplorationGoalRecovery());
  }

  @Override
  protected void setupArguments(List<? extends Object> inputArgs, List<? extends Object> returnArgs) {
    if (!hasLocalArgument("?actor")) {
      addArgument("?actor", Symbol.class, goal.getActor(), false, false);
    } else {
      setArgument("?actor", goal.getActor());
    }
  }

  /**
   * Get this GoalContext's Goal.
   *
   * @return
   */
  public Goal getGoal() {
    return goal;
  }

  @Override
  public void setupNextStep() {
    if (this.getStatus() == ActionStatus.RECOVERY) {
      log.debug("In RECOVERY mode " + goalRecovery.getStatus() + " for goal: " + goal);
      // update recovery status with info about what has happened since the last goalRecovery check-in
      // e.g., it's unlikely that more explorations should be done if an exploration was successful
      goalRecovery.updateRecoveryStatus(this);

      if (goalRecovery.getStatus() == RecoveryStatus.PLAN) {
        // make plan to achieve goal state
        selectAction();

        // exit RECOVERY mode. If the selected action fails, RECOVERY mode can be re-entered based on shouldAttemptRecovery
        setStatus(ActionStatus.PROGRESS);
      } else if (goalRecovery.getStatus() == RecoveryStatus.RECOVER) {
        // previous attempt/child failed -- attempt to execute recovery policy
        ParameterizedAction recoveryPolicy = goalRecovery.selectRecoveryPolicy(this);

        // add recovery policy
        if (recoveryPolicy != null) {
          RecoveryGoalContext recoveryContext = new RecoveryGoalContext(this, this.stateMachine, recoveryPolicy, this.executionType, new DefaultGoalRecovery());
          childContexts.add(recoveryContext);
        } else {
          // no recovery policy --> exit recovery mode
          setStatus(ActionStatus.FAIL_RECOVERY);
        }
      } else if (goalRecovery.getStatus() == RecoveryStatus.EXIT) {
        // exit RECOVERY mode. Set to FAIL_RECOVERY so StepExection can properly finishStep
        setStatus(ActionStatus.FAIL_RECOVERY);
      }
    } else if (this.getChildContexts().isEmpty()) {
      // normal non-recovery action selection (i.e., PROGRESS)
      log.debug("Selecting action for goal: " + goal);
      selectAction();
    }
  }

  private void selectAction() {

    // necessary for simulation where steps are not executed because some roles will not be bound
    if (getExecType().shouldExecute()) {
      // bind goal predicate (which can be the case for asl sub-goals. e.g., goal:?state, goal:holding(?actor,?obj)
      if (!goal.bindPredicate(this)) {
        Justification just = new ConditionJustification(false, Factory.createPredicate("valid", goal.getPredicate()));
        Context child = new FailureContext(this, stateMachine, ActionStatus.FAIL_ARGUMENTS, just);
        this.childContexts.add(child);
        return;
      }
    }

    // check if goal state has already been achieved (in the case that some action has already been executed in an attempt to achieve goal)
    if (!this.getChildContexts().isEmpty()) {
      // two cases: (1) if goal state is desired world state or (2) goal state is a particular action to perform
      if (this.stateMachine.holds(goal.getPredicate()).getValue()) {
        // case (1): goal state is already true -- no need to do anything in this case
        // NOTE: this will need to be generalized for goals that start recovery for a failed action, and then re-select
        // an action that has already been executed in the past as part of some other goal
        return;
      } else {
        // case (2): we're adopting the notion that an action "completed" if all steps in an action succeed, regardless of the post-condition outcome.
        // NOTE: selectionAction will not be called again when an action finishes with SUCCESS, so FAIL_POSTCONDITIONS is
        // the only special case to check for
        Context actionContext = ContextUtils.getMatchingContext(this, this.goal.getPredicate(), ActionStatus.FAIL_POSTCONDITIONS);
        if (actionContext != null) {
          log.info("Goal (specified as an action) failed post-condition checks. Assuming goal state holds: " + goal.getPredicate());
          return;
        }
      }
    }

    Context child;
    if (goal.isObservation()) {
      child = new ObservationContext(this, stateMachine, goal.getPredicate(), goal.getActor(), Observable.TRUE);
    } else {
      // perform action selection
      ParameterizedAction action = goalRecovery.selectNextAction(goal, constraints, stateMachine);

      if (action == null) {
        Justification just = new ConditionJustification(false, new Predicate("found", goal.getActor(), goal.getPredicate()));
        child = new FailureContext(this, stateMachine, ActionStatus.FAIL_NOTFOUND, just);
      } else if (!action.getJustification().getValue()) { // Verify action has support and no critics
        log.error("No permissible action found for goal " + goal + ". Critics: " + action.getJustification().getFailureReason());
        child = new FailureContext(this, stateMachine, ActionStatus.FAIL_FORBIDDEN, action.getJustification());
      } else {
        Symbol actor = (Symbol) getArgumentValue("?actor");
//        child = new NormCheckingActionContext(this, stateMachine, action.getEntry(), action.getBindings(), executionType, actor);
        child = new ActionContext(this, stateMachine, action.getEntry(), action.getBindings(), executionType, actor);
      }
    }

    if (goal.isPersistent() && !child.isFailure()) {
      child = new PersistentContext(this, stateMachine, child);
    }

    childContexts.add(child);
  }

  @Override
  protected void resetConcreteContext() {
    if (getExecType() != ExecutionType.SIMULATE_PERFORMANCE) {
      childContexts.clear();
    }
  }

  @Override
  protected Justification verifyEffects() {
    Justification justification;

    // no particular post-condition needs to hold for observation goals or explicit action goals
    if (goal.isObservation() || goal.isAction()) {
      justification = new ConditionJustification(true);
    } else {
      // else make sure goal state holds
      justification = stateMachine.holds(goal.getPredicate());
    }

    return justification;
  }

  /**
   * Overriding setStatus to "catch" failures when a recovery operation should take place,
   * instead of propagating failures up context tree.
   *
   * TODO: This method is synchronized to handle canceling. Is there a better way to handle this?
   *
   * @param eStatus       the execution status of the currently running event
   * @param justification the reason for the context's execution status
   */
  @Override
  public synchronized void setStatus(ActionStatus eStatus, Justification justification) {

    if (eStatus == ActionStatus.INITIALIZED) {
      // allow context to be (re)set to INITIALIZED, which is required for a context to be reset for re-use
      super.setStatus(eStatus, justification);
    } else if (this.isTerminated()) {
      log.warn("Trying to set status of previously terminated context: " + this.getSignatureInPredicateForm()
              + ". Current status is: " + this.getStatus() + ", tried to set to: " + eStatus);
    } else if (!eStatus.isTerminated() || eStatus == ActionStatus.CANCEL || eStatus == ActionStatus.SUCCESS) {
      log.debug("Can't recover from action status: " + eStatus);
      super.setStatus(eStatus, justification);
    } else if (goalRecovery.shouldAttemptRecovery(eStatus, justification, this)) {
      log.debug("[setStatus] not propagating failure: " + eStatus);
      super.setStatus(ActionStatus.RECOVERY, justification);
    } else {
      log.debug("[setStatus] passing exit handling to super class: " + eStatus);
      super.setStatus(eStatus, justification);
    }
  }

  public Predicate getSignatureInPredicateForm() {
    if (goal != null) {
      if (goal.isAction()) {
        return goal.getPredicate();
      } else if (goal.isObservation()) {
        return Factory.createPredicate("obs", goal.getActor(), goal.getPredicate());
      } else {
        return Factory.createPredicate("goal", goal.getActor(), goal.getPredicate());
      }
    }
    return Factory.createPredicate(cmd);
  }

  /**
   * Make a copy of this node in the execution tree for simulation purposes
   *
   * @param newParent new parent context
   * @return
   */
  @Override
  public GoalContext copy(Context newParent) {
    Goal newGoal = new Goal(goal.getActor(), goal.getPredicate().clone());
    // TODO: probably change the following so that it has the proper execution type
    GoalContext newGoalContext = new GoalContext(newParent, newParent.stateMachine, newGoal, newParent.getExecType());
    copyInternal(newGoalContext);
    return newGoalContext;
  }

}
