/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.execution;

import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class ContextTreeModification {
  private static final Logger log = LoggerFactory.getLogger(ContextTreeModification.class);

  /**
   * Modify the context tree by inserting, removing, or replacing an action step
   * @param context the context tree to be modified -- currently assumes context is GoalContext with 1 step
   * @param modification the type and value of the modification modType(modificationInformation)
   *                     insert(step)
   *                     delete(step)
   *                     replace(ogStep, newStep)
   * @return the index of modified step
   */
  // TODO: need to include reference locations to insert action in different locations throughout script
  // TODO: consider sub-actions
  public static int modifyContext(Context context, Predicate modification) {
    int resetIndex = 0; // also used to return
    String modType = modification.getName();
    // assumes context is of type goal and only one child
    ChildContexts childContexts = context.getChildContexts();
    Context actionContext = childContexts.get(0);
    StateMachine stateMachine = context.getStateMachine();
    if (modType.equals("insert")) {
      Predicate goalPred = Factory.createPredicate(modification.getArgs().get(0).toString());
      Context newGoalContext = new GoalContext(actionContext, stateMachine, new Goal(goalPred), context.getExecType());
      //TODO: need better way to insert action prior to failure for post analysis
      if (actionContext.isFailure()) {
        resetIndex = actionContext.childContexts.findFirstIndexOf(Context::isFailure);
        actionContext.childContexts.insert(newGoalContext, resetIndex);
        overrideStatus(actionContext, ActionStatus.PROGRESS);
      } else {
        resetIndex = Math.max(0, actionContext.childContexts.getNextIndex()-1);
        actionContext.childContexts.insert(newGoalContext, resetIndex);
      }
    } else if (modType.equals("delete") || modType.equals("replace")) {
      Predicate goalPred = Factory.createPredicate(modification.getArgs().get(0).toString());
      Predicate stepPred;
      if (!Database.getActionDB().getActionsBySignature(goalPred).isEmpty()) {
        stepPred = Factory.createPredicate("succeeded", goalPred);
      } else {
        stepPred = goalPred;
      }
      if (actionContext.isFailure()) {
        resetIndex = actionContext.childContexts.findFirstIndexOf(child ->
          (child.isAction() &&
            ((ActionContext) child).getEffects().stream().anyMatch(effect ->
              (effect.getPredicate().equals(stepPred)))
          )
        );
        overrideStatus(actionContext, ActionStatus.PROGRESS);
      } else {
        resetIndex = actionContext.childContexts.findFirstIndexOf(child ->
          (actionContext.childContexts.indexOf(child) >= childContexts.getNextIndex() &&
            child.isAction() &&
            ((ActionContext) child).getEffects().stream().anyMatch(effect ->
              (effect.getPredicate().equals(stepPred)))
          )
        );
      }
      Context child = actionContext.childContexts.get(resetIndex);
      actionContext.childContexts.remove(child);
      if (modType.equals("replace")) {
        Predicate newGoalPred = (Predicate) modification.getArgs().get(1);
        Context newGoalContext = new GoalContext(actionContext, stateMachine, new Goal(newGoalPred), context.getExecType());
        actionContext.childContexts.insert(newGoalContext, resetIndex);
      }
    } else {
      log.warn("No modification of type: " + modType);
    }

    //TODO: need to ensure the remaining child contexts are reset if they were already executed.
    actionContext.childContexts.resetRemaining(resetIndex);
    //}
    context.setChildContexts(childContexts);
    return resetIndex;
  }

  /**
   * go to the step following referenceStep
   * get copy of child contexts, modify child contexts, then set original context with modified children
   * @param referenceStep the step before the step to execute
   * @return found reference step
   */
  public static boolean goToNextStep(Context context, Context referenceStep) {
    ChildContexts childContexts = context.getChildContexts(); // get copy of child contexts
    if (context.isFailure()) { //if context is failure, the status will need to be updated
      overrideStatus(context, ActionStatus.PROGRESS, new ConditionJustification(true)); // set this to progress
    }
    if (childContexts.indexOf(referenceStep) < 0) { // if reference step is not immediate child of context
      for (Context child : childContexts.getChildrenContexts()) {
        // check each child's children
        if (goToNextStep(child, referenceStep)) {
          return true;
        }
      }
    } else {
      overrideStatus(context, ActionStatus.PROGRESS);
      for (Context child : context.getChildContexts().getChildrenContexts()) {
        if (!child.isTerminated()) {
          overrideStatus(child, ActionStatus.SUCCESS);
          if (child.equals(referenceStep)) {
            break;
          }
        }
      }
      // plus 2, because when interpreter is added, the next step is used as start step, so the next step would be the following
      childContexts.setNextIndex(childContexts.indexOf(referenceStep)+2);
      context.setChildContexts(childContexts);
      return true;
    }
    return false;
  }

  /**
   * locate and reset the failed context step. if the context did not cause the failure, set it to progress and check children
   * @param context failed context
   */
  public static void resetFailedContext(Context context) {
    int index = 0;
    if (!context.isFailure()) {
      log.warn("Trying to reset failure context " + context + " but context didn't fail");
      return;
    }
    overrideStatus(context, ActionStatus.PROGRESS);
    ChildContexts childContexts = context.getChildContexts();
    boolean foundFailure = false;
    for (Context child : childContexts.getChildrenContexts()) {
      if (foundFailure) {
        break;
      }
      if (child.isFailure()) {
        foundFailure = true;
        if (child.causedFailure()) { // if child caused failure, then reset it and update the child index
          child.resetCoreContext();
          childContexts.setNextIndex(index);
          break;
        } else {
          //child.resetFailedStep();
          resetFailedContext(child);
          childContexts.setNextIndex(index);
        }
      }
      index ++;
    }
    context.setChildContexts(childContexts);
  }

  private static void overrideStatus(Context context, ActionStatus status) {
    overrideStatus(context,status, null);
  }

  private static void overrideStatus(Context context, ActionStatus status, Justification justification) {
    if (!context.getStatus().equals(status)) {
      context.setStatus(ActionStatus.INITIALIZED);
      if (justification != null) {
        context.setStatus(status, justification);
      } else {
        context.setStatus(status);
      }
    }
  }

}
