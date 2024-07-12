/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.action.execution;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.ActionStack;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.justification.ConditionJustification;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * <code>StepExecution</code>  the execution process for executing
 * a context step: canDoStep, doStep, getNextStep, finishStep.
 */
public class StepExecution {
  protected final static Logger log = LoggerFactory.getLogger(StepExecution.class);
  private ActionStack callStack;
  private Context rootContext;

  public StepExecution(Context rootContext, ActionStack callStack) {
    this.rootContext = rootContext;
    this.callStack = callStack;
  }

  /**
   * <code>execute</code> executes action: check pre conditions, argument constraints
   * do step
   * get next step
   * finish
   *
   * @param step context to execute
   * @return anypoint return next action or null
   */
  public Context execute(Context step) {
    Context nextStep = null;
    step.setStartTime();
    log.debug("[execute] step: " + step.getSignatureInPredicateForm() + " start time: " + step.getStartTime() + "(ms)");
    Justification stepJustification = canDoStep(step);
    if (stepJustification.getValue()) {
      doStep(step); // execute step
      nextStep = getNextStep(step); // Get next child step
      // if there is a next child then return it
      if (nextStep != null) {
        return nextStep;
      }
    }

    // if in RECOVERY or interrupted step due to SUSPEND, return same step to be executed again
    if (step.getStatus() == ActionStatus.RECOVERY || step.getStatus() == ActionStatus.SUSPEND) {
      return step;
    }

    // if there are no other child contexts then finish that step and select the next action
    finishStep(step);

    // step has no more children
    // TODO: re-work this section. it's super ugly and confusing
    nextStep = callStack.next();
    while (nextStep != null) {
      Context tmpStep = getNextStep(nextStep);
      if (tmpStep != null) {
        nextStep = tmpStep;
        break;
      }

      // if in RECOVERY, return same step to be executed again
      if (nextStep.getStatus() == ActionStatus.RECOVERY) {
        callStack.next(); // remove "nextStep" from call stack as it will be added again when it's re-executed
        return nextStep;
      }

      finishStep(nextStep);
      nextStep = callStack.next();
    }
    return nextStep;
  }

  /**
   * <code>canDoStep</code> initializes the context. then checks to see
   * if the action is approved to execute: permissible, pre-conditions,
   * argument-constraints. then it will return the justification for
   * whether the context can execute.
   *
   * @param context the context step which will be executed
   * @return justification for why/not approved to execute.
   */
  public Justification canDoStep(Context context) {
    // Check if current state allows step to be done.
    ActionStatus actionStatus = context.getStatus();
    Justification justification;
    switch (actionStatus) {
      case INITIALIZED:
        actionStatus = ActionStatus.CHECKINGAPPROVAL;
        context.setStatus(actionStatus);
        // do not break -- continue on to CHECKINGAPPROVAL
      case CHECKINGAPPROVAL:
        // Context is initialized. Ask for approval (check if forbidden action ||
        //                                           will lead to forbiddin state ||
        //                                           pre-conditions not met)
        justification = context.isApproved();
        if (justification.getValue()) {
          log.trace("Approved, Context can be executed.");
          context.setStatus(ActionStatus.APPROVED, justification);
        } else {
          log.debug("Step " + context + " is not approved for execution");
        }

        context.performAdditionalStatusUpdates();
        return justification;
      case PROGRESS:
        log.warn("PROGRESS in canDoStep. This shouldn't happen.");
        return new ConditionJustification(true);
      case RECOVERY:
        // this is the case when a context is being re-executed (e.g., GoalContext running a second action)
        return new ConditionJustification(true);
      case SUCCESS:
        log.warn("Step " + context.getSignatureInPredicateForm() + " not being executed because it has already been executed successfully.");
        return new ConditionJustification(false);
      default: // All other states: do not execute
        log.debug("Step " + context.cmd + "not executed. Reason:" + actionStatus);
        return new ConditionJustification(false);
    }
  }

  /**
   * <code>doStep</code> execute the context step: start overall condition monitors,
   * update the state machine, if the step is an action, execute context.
   *
   * @param context step to be executed
   */
  public void doStep(Context context) {
    ActionStatus actionStatus = context.getStatus();
    switch (actionStatus) {
      case APPROVED:
        log.trace("Execution in progress...");
        context.setStatus(ActionStatus.PROGRESS, new ConditionJustification(true));
        log.trace("Starting overall conditions monitor...");
        context.startOverAllMonitor();
        // Do not break, move on to PROGRESS case.
      case PROGRESS:
      case RECOVERY:
        context.doStep();
        break;
      default:
        log.debug("Step not executed. Reason: " + actionStatus);
    }
  }

  /**
   * <code>getNextAction</code> selects the next sub step to be executed
   *
   * @param context the step that is being executed, used to get sub steps
   * @return a sub step for to execute, may return null to depict no more steps
   * or exitcontext
   */
  public Context getNextStep(Context context) {
    return context.getNextStep(); // Get next(child) step of lastStep
  }

  /**
   * <code>finishStep</code> finish the context step: verify the effects,
   * update state machine, if step is an action. return a justifcation for
   * success/failure.
   *
   * @param context step to be completes
   * @return justification for success/failure
   */
  public Justification finishStep(Context context) {
    // Check if current state allows the step to succeed.
    if (context.isFailure()) {
      // if this context is the root of the failure, and failed with pre- or post-condtion failure, don't check effects,
      // otherwise check effects. also, check effects of all ancestor contexts as we move up the tree
      if ((context.getStatus() != ActionStatus.FAIL_PRECONDITIONS && context.getStatus() != ActionStatus.FAIL_POSTCONDITIONS) || !context.causedFailure()) {
        // still want to observe which effects hold, except when FAIL_POSTCONDITONS because they've already been checked
        context.verifyEffects();
      }
      context.caller.setStatus(context.getStatus(), context.getJustification());
      context.performAdditionalStatusUpdates();
      return context.getJustification();
    } else {
      Justification verification;
      switch (context.getStatus()) {
        case SUCCESS:
          context.performAdditionalStatusUpdates();
          return context.getJustification();
        case PROGRESS:
          // set next step to verify return value
          context.setStatus(ActionStatus.VERIFYING_RETURNVALUE, new ConditionJustification(true));
          break;
        case RETURN:
          if (!context.isAction() && !context.isAsynchronous()) {
            // propagate "return" up context tree
            context.caller.setStatus(ActionStatus.RETURN);
          }
          // still want to verify return value as we move up context tree
          context.setStatus(ActionStatus.VERIFYING_RETURNVALUE);
          break;
        case VERIFYING_RETURNVALUE:
          log.trace("Verifying return values...");
          verification = context.verifyReturnValue();
          if (verification.getValue()) {
            log.trace("Return value verified.");
            context.setStatus(ActionStatus.VERIFYING_EFFECTS, verification);
          } else {
            log.debug("Return value verification failed for " + context.getSignatureInPredicateForm() + " FAIL_RETURNVALUE");
            context.setStatus(ActionStatus.FAIL_RETURNVALUE, verification);
          }
          break;
        case VERIFYING_EFFECTS:
          log.trace("Verifying effects...");
          // TODO: this is potentially problematic -- shouldn't all children need to finish before effects are verified
          verification = context.verifyEffects();
          if (verification.getValue()) {
            log.trace("Effects verified. Success!");
            try {
              TRADE.getAvailableService(new TRADEServiceConstraints().name("getViolations"));
              context.setStatus(ActionStatus.VERIFYING_NORMS, verification);
            } catch (TRADEException e) {
                context.setStatus(ActionStatus.SUCCESS, verification);
            }
          } else if (!context.isFailure()) { // FIXME: this checks if failure because verify effects sets failure during simulation
            log.debug("Effect verification failed. FAIL_POSTCONDITIONS");
            context.setStatus(ActionStatus.FAIL_POSTCONDITIONS, verification);
          }
          break;
        case VERIFYING_NORMS:
          //todo: Check norms
//          try {
//            verification = (Justification) TRADE.callThe("getViolations", context.getActor("?actor"), context.stateMachine);
//            if (verification.getValue()) {
//              context.setStatus(ActionStatus.SUCCESS, verification);
//            } else {
//              context.setStatus(ActionStatus.FAIL_NORMS, verification);
//            }
//          } catch (TRADEException e) {
//            throw new RuntimeException(e);
//          }
          break;

        case ACTIVE_CHILD:
          // TODO: this is potentially problematic -- shouldn't all children need to finish before effects are verified
          // TODO: this should wait for all children to finish
          // for now, do nothing, just return as if execution finished successfully
          return context.getJustification();
        case CANCEL:
          context.caller.setStatus(context.getStatus(), context.getJustification());
          context.performAdditionalStatusUpdates();
          return context.getJustification();
        case PARENT_CANCELED:
          context.caller.setStatus(ActionStatus.CANCEL);
          return context.getJustification();
        default: // No state change for other cases.
          log.error("[finishStep] current status in action \"" + context.getCommand() + "\" not handled: " + context.getStatus());
          return new ConditionJustification(false);
      }
    }

    // resursively call finishStep until it bottoms out with SUCCESS, isFailure(), or some unhandled case (i.e., default switch)
    // TODO: make this tail-recursive
    return finishStep(context);
  }
}
