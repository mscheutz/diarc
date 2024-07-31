/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.recovery;

import edu.tufts.hrilab.action.*;
import edu.tufts.hrilab.action.execution.*;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.execution.util.ContextUtils;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.selector.ActionSelector;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;

public class Recovery {

    protected Logger log = LoggerFactory.getLogger(this.getClass());

    protected RecoveryStatus recoveryStatus = RecoveryStatus.PLAN;

    /**
     * Failure information of the original failure which caused recovery to start in the first place
     */
    private FailureInformation failureInfo = null;

    /**
     * Last failure action status received during shouldAttemptRecovery call.
     */
    private ActionStatus lastFailureStatus = null;
    /**
     * Last justification received during shouldAttemptRecovery call. This is the justification for the lastFailureStatus.
     */
    private Justification lastFailureJustification = null;
    /**
     * Failure status which triggered the most recent recovery policy execution. Caching so that we can continue to
     * propagate the original ActionStatus up the context tree in case recovery fails.
     */
    private ActionStatus originalFailureStatus = null;
    /**
     * Justification associated with originalFailureStatus.
     */
    private Justification originalFailureJustification = null;
    /**
     * (ActionRecovery) The direct child context whose failure triggered recovery (whether it was this context directly
     *  or a descendent of this context). Stored for RETRY post recovery execution.
     */
    private ActionContext failedChildContext = null;
    /**
     * (ActionRecovery) Indication of whether the leaf action which failed is a direct child of this catching context or not.
     *  Used in action recovery where a specific catchingContext signature is not defined, to prevent every subsequent
     *  node from entering recovery.
     */
    private boolean failedActionIsChild = false;

    /**
     * The last selected recovery policy action. It is cached here, so a new selection call
     * isn't needed for selectRecoveryPolicy, since the initial selection is made during
     * shouldAttemptRecovery.
     */
    private ActionDBEntry selectedRecoveryAction = null;

    /**
     * (ActionRecovery) Indicates desired behavior after successful execution of a recovery policy
     */
    private PostRecoveryExecutionType postRecoveryExecutionType = null;

    private boolean retryingContextExecution = false;

    public Recovery() {
        super();
    }

    /**
     * Get current recovery status/mode.
     * @return
     */
    public final RecoveryStatus getStatus() {
        return recoveryStatus;
    }

    /**
     * Get the ActionStatus of the failure which originally caused recovery to begin.
     * @return the ActionStatus, null if not currently in recovery
     */
    public final ActionStatus getOriginalFailureStatus() {
        return originalFailureStatus;
    }

    /**
     * Get the Justification of the failure which originally caused recovery to begin.
     * @return the Justification, null if not currently in recovery
     */
    public final Justification getOriginalFailureJustification() {
        return originalFailureJustification;
    }

    /**
     * Get the current PostRecoveryExecutionType
     */
    public final PostRecoveryExecutionType getPostRecoveryExecutionType() {
        return postRecoveryExecutionType;
    }

    /**
     * Resets recovery state - clears cached details from last failure and chosen recovery policy to allow for new recoveries
     */
    public void resetState() {
        recoveryStatus = RecoveryStatus.PLAN;
        lastFailureJustification = null;
        lastFailureStatus = null;
        failureInfo = null;
        originalFailureStatus = null;
        originalFailureJustification = null;
        failedChildContext = null;
        failedActionIsChild = false;
        selectedRecoveryAction = null;
        postRecoveryExecutionType = null;
    }

    public boolean shouldAttemptRecovery(ActionStatus actionStatus, Justification justification, Context context) {
        lastFailureJustification = justification;
        lastFailureStatus = actionStatus;

        //Can't recover from leaf context itself. Need to at least be one step up in order to be in the correct place to
        //  handle the tree/execution after recovery completes.
        if (context.getChildContexts().isEmpty()) {
            return false;
        }

        // ActionStatus.FAIL is only the case if an action script explicitly exits during execution (i.e., exit(FAIL))
        // NOTE: ActionStatus.FAIL happens during the cancelGoal goal when no matching goal is found, and we don't want to recover in that case
        if (actionStatus.isFailure() && actionStatus != ActionStatus.FAIL) {
            switch (actionStatus) {
                case EXIT_RECOVERY:
                case FAIL_RECOVERY:
                    recoveryStatus = RecoveryStatus.EXIT;
                    return false;
                case FAIL_POSTCONDITIONS:
                    // bring agent belief in line with world state (i.e., discrepancies)
                    Set<Predicate> failureReasons = new HashSet<>(justification.getFailureReason());
                    Utilities.correctBeliefState(failureReasons);
                case FAIL_OVERALLCONDITIONS:
                case FAIL_PRECONDITIONS:
                case FAIL_RETURNVALUE:
                case FAIL_NOTFOUND:
                    if (hasRecoveryPolicy(context)) {
                        // If recovery is causing its own recovery by making execution fail repeatedly in the same way, prevent infinite loop
                        if (duplicateRecoveryAttempt(context)) {
                            log.warn("[shouldAttemptRecovery] detected recovery loop, returning false");
                            return false;
                        }
                        recoveryStatus = RecoveryStatus.RECOVER;
                        return true;
                    } else {
                        recoveryStatus = RecoveryStatus.EXIT;
                        return false;
                    }
            }
        }

        log.debug("Not recovering from context: " + context.getCommand() + " with action status: " + actionStatus);
        recoveryStatus = RecoveryStatus.EXIT;
        return false;
    }

    public ParameterizedAction selectNextAction(Goal goal, ActionConstraints constraints, StateMachine stateMachine) {
        return ActionSelector.getInstance().selectActionForGoal(goal, constraints, stateMachine);
    }

    public ParameterizedAction selectRecoveryPolicy(Context context) {
        // populate arguments for ParameterizedAction
        Map<String, Object> arguments = new HashMap<>();
        List<ActionBinding> inputArgs = selectedRecoveryAction.getInputRoles();
        arguments.put(inputArgs.get(0).getName(), failureInfo.actor);
        arguments.put(inputArgs.get(1).getName(), failureInfo.failedActions);
        arguments.put(inputArgs.get(2).getName(), failureInfo.failureReasons);
        arguments.put(inputArgs.get(3).getName(), failureInfo.goal);


        return new ParameterizedAction(selectedRecoveryAction, arguments);
    }

    //TODO: have way to override postRecoveryBehavior from within execution of the recovery policy
    public void updateRecoveryStatus(Context context) {
        if (recoveryStatus == RecoveryStatus.RECOVER && context.getChildContexts().getCurrent().isSuccess()) {
            if (failureInfo.contextType == RecoveryType.GOAL) {
                // (GoalRecovery) if in RECOVER mode, and last goal (i.e., recover goal) succeeded, go back to PLAN mode
                recoveryStatus = RecoveryStatus.PLAN;
                // Similarly want to prevent infinite recovery loops if recovery policy didn't actually fix anything and
                //  execution ends up failing in the same exact way.
                // Same logic used for checking in the action recovery case works. Can separate this reasoning if we
                //  want different behavior
                retryingContextExecution = true;
            } else {
                // (ActionRecovery) if in RECOVER mode, and last goal (i.e., recover goal) succeeded, set status to
                //  SUCCESS and allow ActionContext to handle post recovery execution based on postRecoveryExecutionType
                if (postRecoveryExecutionType == PostRecoveryExecutionType.RETRY || postRecoveryExecutionType == PostRecoveryExecutionType.PARENT_RETRY) {
                    retryingContextExecution = true;
                }
                recoveryStatus = RecoveryStatus.SUCCESS;
            }
        }
    }

    public ActionContext getContextToRetry() {
        return failedChildContext;
    }

    /**
     * Checks if there are any valid recovery policies for the current failure. If there is at least one
     * policy available, selects one, and caches it for later use in selectRecoveryPolicy.
     * @param context
     * @return
     */
    private boolean hasRecoveryPolicy(Context context) {
        // don't attempt to recover if no recovery policy exists
        selectedRecoveryAction = selectRecoveryAction(context);
        if (selectedRecoveryAction == null) {
            log.debug("Not recovering. No recovery policy could be found. Not recovering from context: " + context.getCommand());
            return false;
        }

        return true;
    }

    /**
     * Gets all valid recovery policy actions from the Database.
     * @param context
     * @return
     */
    private ActionDBEntry selectRecoveryAction(Context context) {
        // populate policy constraints based on current context
        GoalContext goalContext;
        failureInfo = new FailureInformation();
        if (context instanceof GoalContext) {
            goalContext = (GoalContext)context;
            // TODO: Is this desired?
            //// Don't allow goal recovery for a goal which is simply an action, should use action recovery which would have been applied on the child already
            //if (goalContext.getGoal().isAction()) {
            //    return null;
            //}
            failureInfo.contextType = RecoveryType.GOAL;
            failureInfo.actor = goalContext.getGoal().getActor();
        } else if (context instanceof ActionContext) {
            failureInfo.contextType = RecoveryType.ACTION;
            goalContext = ContextUtils.getGoalContext(context);
            failureInfo.catchingContext = context.getSignatureInPredicateForm();
            failureInfo.actor = (Symbol)context.getArgumentValue("?actor");
        } else {
            log.error("[selectRecoveryAction] called with incompatible context type {}", context.getClass());
            return null;
        }
        failureInfo.goal = goalContext == null ? null : goalContext.getGoal().getPredicate();
        failureInfo.actionStatus = lastFailureStatus;
        failureInfo.failedActions = getFailedActions(context);
        failureInfo.isCatchingContextParent = failedActionIsChild;
        failureInfo.failureReasons = lastFailureJustification.getFailedConditions();

        // look up recovery policy options
        List<ActionDBEntry> recoveryPolicies = Database.getRecoveryPolicyDB().getRecoveryPolicies(failureInfo);

        if (recoveryPolicies.isEmpty()) {
            return null;
        } else if (recoveryPolicies.size() > 1) {
            log.warn("More than one valid recovery policy found. Using first one in list.");
        }
        ActionDBEntry recoveryPolicy = recoveryPolicies.get(0);

        postRecoveryExecutionType = recoveryPolicy.getPolicyConstraints().postRecoveryExecutionType;
        originalFailureStatus = lastFailureStatus;
        originalFailureJustification = lastFailureJustification;

        return recoveryPolicy;
    }

    /**
     * Helper method to find the failed action in the context tree, and return
     * the Predicate form of the failed action.
     *
     * @param context
     * @return
     */
    private List<Predicate> getFailedActions(Context context) {
        List<Predicate> failedActionPredicates = new ArrayList<>();
        Context failedContext = null;

        //Step 1: Find the failed leaf context
        if (lastFailureStatus != ActionStatus.FAIL_NOTFOUND) {
            //Find the failed action context, the below method currently works for cases other than FAIL_NOTFOUND
            ActionContext failedAction = ContextUtils.getActionContext(context.getChildContexts().getCurrent(), lastFailureJustification.getStep(), lastFailureStatus);
            if (failedAction != null) {
                failedContext = failedAction;
            }
        }
        //If failure reason is FAIL_NOTFOUND, either didn't find an action or couldn't plan. Regardless, navigate down the
        // current context branch down to the failed context.
        if (failedContext == null) {
            failedContext = context.getChildContexts().getCurrent();
            while (!failedContext.getChildContexts().isEmpty()) {
                failedContext = failedContext.getChildContexts().getCurrent();
            }
        }

        failedActionIsChild = failedContext == context.getChildContexts().getCurrent();

        //Step 2: Traverse back up the tree from the failed context to gather the trace of failed actions
        while (failedContext != context) {
            if (failedContext instanceof ActionContext) {
                failedActionPredicates.add(failedContext.getSignatureInPredicateForm());
                failedChildContext = (ActionContext) failedContext;
            }
            failedContext = failedContext.getParentContext();
        }

        return failedActionPredicates;
    }

    /**
     * Helper method to determine whether the supplied context is a RecoveryGoalContext matching the currently selected recovery policy
     */
    private boolean isSelectedRecovery(Context context) {
        return context instanceof RecoveryGoalContext &&
               ((ActionContext) context.getChildContexts().get(0)).getDBE() == selectedRecoveryAction;
    }

    //TODO: In general, I'm sure other scenarios exist where this can occur. How much of this is developer
    // misuse/bad policy implementation vs unsafe recovery implementation? 
    /**
     * Helper method to determine whether the supplied context is entering a repeated recovery loop
     */
    private boolean duplicateRecoveryAttempt(Context context) {
        if (lastFailureStatus == null || lastFailureJustification == null) {
            return false;
        }

        //Case 1: Prevent infinite recovery when using a poorly defined policy alongside 'postrecovery_retry' or 'postrecovery_parent_retry' execution
        //TODO: when to reset retryingContextExecution?
        if (retryingContextExecution) {
            //If we are currently undergoing re-execution of a context after recovery, make sure identical recovery isn't
            //  triggered during that re-execution. The selected policy will be cached by the time this method is hit.
            PostRecoveryExecutionType retryType = selectedRecoveryAction.getPolicyConstraints().postRecoveryExecutionType;

            int count = 1;
            int childrenSize = context.getChildContexts().size();
            //Search children backwards for the same recovery policy as the one currently selected (while counting)
            while (count < childrenSize) {
                Context childContext = context.getChildContexts().get(childrenSize - count);
                if (isSelectedRecovery(childContext)) {
                    if (retryType == PostRecoveryExecutionType.RETRY) {
                        //If RETRY, just need to check if the previous child is the same recovery policy
                        return count == 2;
                    } else if (retryType == PostRecoveryExecutionType.PARENT_RETRY) {
                        //TODO: this isn't necessarily true with conditional branches - to do this 100% safely would
                        //      essentially need to check if all steps are the same
                        //If PARENT_RETRY, check if the previous execution trace was the same length as the current one
                        if (childrenSize == count * 2 - 1) {
                            //Single previous trace
                            return true;
                        } else if (childrenSize > count * 2 - 1) {
                            //Unlikely, but possible we have multiple recovery traces of mismatched length
                            childContext = context.getChildContexts().get(childrenSize - count*2);
                            return isSelectedRecovery(childContext);
                        }
                    }
                }
                count += 1;
            }
        }

        //Rewrite this section if recovery interface is changed/moved
        //Case 2: Prevent application of a recovery policy from within execution of that same recovery policy
        context = context.getParentContext();
        while (context != null) {
            Recovery recovery;
            if (context instanceof ActionContext) {
                recovery = ((ActionContext)context).getRecovery();
            } else if (context instanceof GoalContext) {
                recovery = ((GoalContext)context).getRecovery();
            } else {
                context = context.getParentContext();
                continue;
            }
            if (recovery.getStatus() == RecoveryStatus.RECOVER) {
                return lastFailureStatus.compareTo(recovery.originalFailureStatus) == 0 && lastFailureJustification.equals(recovery.originalFailureJustification);
            }
            context = context.getParentContext();
        }
        return false;
    }
}
