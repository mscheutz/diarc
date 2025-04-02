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
     * Failure information of the most recent failure, used to select a recovery policy
     */
    private FailureInformation failureInfo = null;

    //Q: For Goal recovery, should there be a distinction between consecutively failing in the same
    // way vs making some progress/failing differently before failing again in the same way? (similar to action
    // parent_retry) Where would this max count be defined? What is defined as progress?
    // (Implementation would be in hasExhaustedRecoveryAttempts next to checkForExecutionProgress)
    /**
     * Record of recovery policy application counts distinguished by failure information.
     * Maximum recovery attempts are defined on recovery policies now. Practically this means:
     *   Action Recovery RETRY: The same child context can be reattempted and fail up to N times the same way while
     *     using this policy to recover. Count is reset when the reattempted context succeeds.
     *   Action Recovery PARENT_RETRY: This policy can be applied up to N times for the same reason without making
     *      progress through the script.
     *   Goal Recovery: This policy can be applied up to N times for the same reason total while executing this context.
     */
    private Map<FailureInformation, Integer> policyApplications = new HashMap<>();
    private PolicyConstraints lastMatchingConstraints = null;
    /**
     * Record of policy application counts for this context's current action step. Important for action recovery RETRY
     * cases in case of cyclical failures. Whenever successfully completing a step, all recovery applications for that
     * step should be cleared so that if the same action is present later on in the same script, counts don't overlap
     * and cause recovery to be aborted early.
     */
    private Map<FailureInformation, Integer> currentStepPolicyApplications = new HashMap<>();
    //Q: Should there be separate attempt thresholds for failing in the same way consecutively vs failing in the same
    //      way overall? The below is not used for anything right now, could be used for that. (If so, include notion of
    //      progress to inform when to increment/reset)
    /**
     * Number of times the associated context has failed in the exact same way consecutively.
     */
    private int repeatedIdenticalFailures = 1;
    /**
     * The original failure status which triggered recovery policy application. Caching so that we can continue to
     * propagate the original ActionStatus up the context tree in case recovery fails (rather than the recovery failure)
     */
    private ActionStatus originalFailureStatus = null;
    /**
     * Justification associated with originalFailureStatus.
     */
    private Justification originalFailureJustification = null;
    /**
     * (ActionRecovery) The direct child context whose failure triggered recovery (whether it was initially this context
     * directly or a descendant of this context). Stored for RETRY post recovery execution.
     */
    private ActionContext failedChildContext = null;
    /**
     * (ActionRecovery) Indication of whether the leaf action which failed is a direct child of this catching context or not.
     *  Used in action recovery where a specific catchingContext signature is not defined, to ensure only the parent
     *  attempts recovery and to prevent every subsequent ancestor from entering recovery in case of failure.
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

    /**
     * An indication of whether this Recovery instance's associated context is currently undergoing execution which is
     * part of a 'repeated attempt' after a recovery policy has been successfully executed. This is possible any time in
     * goal recovery when replanning, and in action recovery when postRecoveryExecutionType is RETRY or PARENT_RETRY.
     * In these situations we want to perform extra checks to prevent easy-to-detect cases of a recovery policy not
     * sufficiently affecting the world and causing infinitely looping recovery.
     * 0 == Goal Recovery (Never gets reset because the goal either succeeds or recovery is re-entered)
     * 1 == RETRY (Must be reset after the reattempted child context is complete)
     * 2 == PARENT_RETRY (Never gets reset because the context either completes or recovery is re-entered)
     */
    private int retryingContextExecution = -1;

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
        failedChildContext = null;
        failedActionIsChild = false;
        selectedRecoveryAction = null;
        postRecoveryExecutionType = null;
        lastMatchingConstraints = null;
    }

    public void resetConcreteState() {
        resetState();
        failureInfo = null;
        repeatedIdenticalFailures = 1;
        originalFailureJustification = null;
        originalFailureStatus = null;
        retryingContextExecution = -1;
        policyApplications = new HashMap<>();
        currentStepPolicyApplications = new HashMap<>();
    }

    public boolean shouldAttemptRecovery(ActionStatus actionStatus, Justification justification, Context context) {
        // ActionStatus.FAIL is only the case if an action script explicitly exits during execution (i.e., exit(FAIL))
        // NOTE: ActionStatus.FAIL happens during the cancelGoal goal when no matching goal is found, and we don't want to recover in that case
        if (actionStatus.isFailure() && actionStatus != ActionStatus.FAIL) {
            switch (actionStatus) {
                case EXIT_RECOVERY:
                    recoveryStatus = RecoveryStatus.EXIT;
                    return false;
                case FAIL_RECOVERY:
                    // TODO: investigate if this is what we want for action recovery, and exactly what the difference is
                    //       between EXIT_RECOVERY (explicit exit from asl?) and FAIL_RECOVERY
                    recoveryStatus = RecoveryStatus.PLAN;
                    return true;
                case FAIL_POSTCONDITIONS:
                    // bring agent belief in line with world state (i.e., discrepancies)
                    Set<Predicate> failureReasons = new HashSet<>(justification.getFailureReason());
                    Utilities.correctBeliefState(failureReasons);
                case FAIL_OVERALLCONDITIONS:
                case FAIL_PRECONDITIONS:
                case FAIL_RETURNVALUE:
                case FAIL_NOTFOUND:
                    if (hasRecoveryPolicy(context, actionStatus, justification)) {
                        if (shouldTerminateRecovery(context, actionStatus, justification)) {
                            return false;
                        }
                        postRecoveryExecutionType = lastMatchingConstraints.postRecoveryExecutionType;
                        originalFailureStatus = originalFailureStatus == null ? actionStatus : originalFailureStatus;
                        originalFailureJustification = originalFailureJustification == null ? justification : originalFailureJustification;
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

    public void updateRecoveryStatus(Context context) {
        //If in RECOVER mode and last goal (i.e. recovery goal) succeeded, set relevant recovery status and potentially
        //  PostRecoveryExecutionType so setupNextStep in the corresponding context can handle setting up relevant children.
        //Also set flags to begin checking for infinite recovery loops if the recovery policy didn't actually fix things
        //  and ends up causing repeated failures
        if (recoveryStatus == RecoveryStatus.RECOVER && context.getChildContexts().getCurrent().isSuccess()) {
            //Inspect whether recovery policy has a return value and, if so, whether it is a valid PostRecoveryExecutionType. Allow return value to override policy's default behavior if so
            ActionContext recoveryActionContext = (ActionContext) context.getChildContexts().getCurrent().getChildContexts().getCurrent();
            if (recoveryActionContext.hasArgumentInScope("?postRecoveryType")) {
                postRecoveryExecutionType = (PostRecoveryExecutionType) recoveryActionContext.getArgumentValue("?postRecoveryType");

            }
            if (context instanceof GoalContext) {
                //Special case: root goal context is an action, applied action recovery under the goal context
                if (postRecoveryExecutionType == PostRecoveryExecutionType.SUCCESS || postRecoveryExecutionType == PostRecoveryExecutionType.PARENT_SUCCESS) {
                    //PostRecovery success == set status to success, don't re-execute/replan
                    recoveryStatus = RecoveryStatus.SUCCESS;
                } else {
                    //PostRecovery retry == re-execute action (i.e. replan == reselect action for goal)
                    recoveryStatus = RecoveryStatus.PLAN;
                    retryingContextExecution = 0;
                }
            } else {
                recoveryStatus = RecoveryStatus.SUCCESS;
                if (postRecoveryExecutionType == PostRecoveryExecutionType.RETRY) {
                    retryingContextExecution = 1;
                } else if (postRecoveryExecutionType == PostRecoveryExecutionType.PARENT_RETRY) {
                    retryingContextExecution = 2;
                }
            }
        }
    }

    public ActionContext getContextToRetry() {
        return failedChildContext;
    }

    /**
     * Checks if there are any valid recovery policies for the current failure. If there is at least one
     * policy available, selects one, and caches it for later use in selectRecoveryPolicy.
     */
    private boolean hasRecoveryPolicy(Context context, ActionStatus failureStatus, Justification failureJustification) {
        // don't attempt to recover if no recovery policy exists
        selectedRecoveryAction = selectRecoveryAction(context, failureStatus, failureJustification);
        if (selectedRecoveryAction == null) {
            log.debug("Not recovering. No recovery policy could be found. Not recovering from context: " + context.getCommand());
            return false;
        }

        return true;
    }

    /**
     * Gets all valid recovery policy actions from the Database.
     */
    private ActionDBEntry selectRecoveryAction(Context context, ActionStatus failureStatus, Justification failureJustification) {
        // populate policy constraints based on current context
        GoalContext goalContext;
        FailureInformation newFailureInfo = new FailureInformation();
        if (context instanceof GoalContext) {
            goalContext = (GoalContext)context;
            newFailureInfo.contextType = RecoveryType.GOAL;
            newFailureInfo.actor = goalContext.getGoal().getActor();
        } else if (context instanceof ActionContext) {
            // If the action context is a leaf, return null so that the failure can potentially be caught in a parent
            if (context.getChildContexts().isEmpty()) {
                return null;
            }
            newFailureInfo.contextType = RecoveryType.ACTION;
            goalContext = ContextUtils.getGoalContext(context);
            newFailureInfo.catchingContext = context.getSignatureInPredicateForm();
            newFailureInfo.actor = (Symbol)context.getArgumentValue("?actor");
        } else {
            log.error("[selectRecoveryAction] called with incompatible context type {}", context.getClass());
            return null;
        }
        newFailureInfo.goal = goalContext == null ? null : goalContext.getGoal().getPredicate();
        newFailureInfo.actionStatus = failureStatus;
        newFailureInfo.failedActions = getFailedActions(context,failureStatus,failureJustification);
        newFailureInfo.isCatchingContextParent = failedActionIsChild;
        newFailureInfo.failureReasons = failureJustification.getFailedConditions();

        // look up recovery policy options
        List<PolicyConstraints> matchingConstraints = Database.getRecoveryPolicyDB().getMatchingPolicyConstraints(newFailureInfo);

        if (matchingConstraints.isEmpty()) {
            // Special case where the top level goal is an action: need to potentially apply action recovery under the
            //  goal context in order to maintain proper context history structure
            if (context instanceof GoalContext && ((GoalContext) context).getGoal().isAction() && failedActionIsChild) {
                newFailureInfo.isCatchingContextParent = true;
                newFailureInfo.contextType = RecoveryType.ACTION;
                newFailureInfo.goal = ((GoalContext) context).getGoal().getPredicate();
                matchingConstraints = Database.getRecoveryPolicyDB().getMatchingPolicyConstraints(newFailureInfo);
            }
        }

        if (newFailureInfo.equals(failureInfo)) {
            repeatedIdenticalFailures += 1;
        } else {
            repeatedIdenticalFailures = 1;
        }
        failureInfo = newFailureInfo;

        //Cannot have RETRY post recovery if no specific child action at this level failed (possible in the case of FAIL_OVERALLCONDITIONS, FAIL_POSTCONDITIONS)
        //Is practically also an issue with FAIL_NOTFOUND because there is no current functionality to reattempt to parse the previous eventSpec.
        // Q: should this ^ case be allowed? (recovery when an action isn't known/its service isn't available)
        if (newFailureInfo.contextType == RecoveryType.ACTION && newFailureInfo.failedActions.isEmpty()) {
            matchingConstraints.removeIf(e -> e.postRecoveryExecutionType == PostRecoveryExecutionType.RETRY);
        }

        if (matchingConstraints.isEmpty()) {
            return null;
        } else {
            List<ActionDBEntry> recoveryPolicies = Database.getRecoveryPolicyDB().getRecoveryPolicies(matchingConstraints);
            if (recoveryPolicies.size() > 1) {
                log.warn("More than one valid recovery policy found. Using first one in list.");
            }
            lastMatchingConstraints = matchingConstraints.get(0);
            return recoveryPolicies.get(0);
        }
    }

    /**
     * Helper method to find the failed action in the context tree, and return
     * the Predicate form of the failed action.
     *
     * @param context
     * @return
     */
    private List<Predicate> getFailedActions(Context context, ActionStatus failureStatus, Justification failureJustification) {
        List<Predicate> failedActionPredicates = new ArrayList<>();
        Context failedContext = null;

        //Step 1: Find the failed leaf context
        if (failureStatus != ActionStatus.FAIL_NOTFOUND) {
            //Find the failed action context
            ActionContext failedAction = ContextUtils.getActionContext(context.getChildContexts().getCurrent(), failureJustification.getStep(), failureStatus);
            if (failedAction != null) {
                failedContext = failedAction;
            }
        } else {
            //If failure reason is FAIL_NOTFOUND, either didn't find an action or couldn't plan. Regardless, navigate down the
            // current context branch down to the failed context.
            failedContext = context.getChildContexts().getCurrent();
            while (!failedContext.getChildContexts().isEmpty()) {
                failedContext = failedContext.getChildContexts().getCurrent();
            }
        }

        failedActionIsChild = failedContext == context.getChildContexts().getCurrent();

        //Step 2: Traverse back up the tree from the failed context to gather the trace of failed actions
        if (failedContext != null) {
            while (failedContext != context) {
                if (failedContext instanceof ActionContext) {
                    failedActionPredicates.add(failedContext.getSignatureInPredicateForm());
                    failedChildContext = (ActionContext) failedContext;
                }
                failedContext = failedContext.getParentContext();
            }
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

    //feel like relationship between recovery and context can change to prevent this control flow
    /**
     * Called when this Recovery's corresponding ActionContext calls setupNextStep from its parser
     */
    public void signalStep() {
        if (retryingContextExecution == 0 || retryingContextExecution == 1) {
            // If execution continues from the parser, means that the re-executed context has terminated successfully.
            retryingContextExecution = -1;
            // For Action Recovery RETRY case, reset recovery attempts if retry is successful
            // (in case the same policy is used for a different step in the same script)
            for (Map.Entry<FailureInformation,Integer> entry : currentStepPolicyApplications.entrySet()) {
                Integer count = policyApplications.get(entry.getKey());
                if (count != null) {
                    policyApplications.put(entry.getKey(),entry.getValue()-count);
                }
            }
            currentStepPolicyApplications = new HashMap<>();
        }
    }

    /**
     * Returns indication of whether recovery for this context should be aborted or allowed to continue. Returns true if
     * looping recovery attempts are detected or the chosen recovery policy's maximum allotted attempts has been exceeded.
     */
    private boolean shouldTerminateRecovery(Context context, ActionStatus failureStatus, Justification failureJustification) {
        if (isLoopingRecovery(context, failureStatus, failureJustification)) {
            log.warn("[shouldTerminateRecovery] detected recovery loop, returning true");
            return true;
        } else if (hasExhaustedRecoveryAttempts(context)) {
            log.warn("[shouldTerminateRecovery] have exhausted recovery policy applications, returning true");
            return true;
        }
        return false;
    }

    /**
     * Helper method to determine whether the chosen recovery policy has reached its maximum allotted number of attempts
     * without success.
     */
    private boolean hasExhaustedRecoveryAttempts(Context context) {
        // Get current count of times this policy has been applied for this reason before
        int maxAttempts = lastMatchingConstraints.maximumAttempts;
        int count = policyApplications.getOrDefault(failureInfo,0)+1;

        //TODO: Do we want similar logic for Goal Recovery? Or related logic which has two separate thresholds - one
        // for maximum consecutive failures for the same reason and maximum total failures for the same reason?
        // (Relevant because we cannot just clear the count when a different failure is encountered in case of cyclical failures)
        //PARENT_RETRY: If we have explicitly made more progress in the script since last attempt, don't increment count
        if (count > 1 && retryingContextExecution == 2 && checkForExecutionProgress(context)) {
            return false;
        }
        // Return true to exit recovery and allow the context to fail
        if (count > maxAttempts) {
            return true;
        }
        // Increment count and return false
        policyApplications.put(failureInfo,count);
        //Action Recovery RETRY: additionally update currentPolicyApplications
        if (lastMatchingConstraints.postRecoveryExecutionType == PostRecoveryExecutionType.RETRY) {
            currentStepPolicyApplications.put(failureInfo,count);
        }
        return false;
    }

    /**
     * Helper method to determine if the most recent execution attempt of this context has made more progress than the
     * previous attempt (Action Recovery PARENT_RETRY only).
     */
    private boolean checkForExecutionProgress(Context context) {
        // Q: In Action Recovery PARENT_RETRY case, do we want a distinction between the parent context failing again
        //    on the same/a previous step vs on a step which is deeper in the script?
        //    I.E. don't increment attempt count if we made more progress since last recovery
        int count = 1;
        ChildContexts childContexts = context.getChildContexts();
        int childrenSize = childContexts.size();
        //Search children backwards for the same recovery policy as the one currently selected (while counting)
        while (count < childrenSize) {
            Context childContext = childContexts.get(childrenSize - count);
            //Found recovery policy which is same as the one about to be selected
            if (isSelectedRecovery(childContext)) {
                //Now that the previous recovery context's position is known, compare execution traces leading to recovery selection
                for (int i = 1; i < count; i++) {
                    Context previousAttemptContext = childContexts.get(childrenSize - count - i);
                    Context currentAttemptContext = childContexts.get(childrenSize - i);
                    if (previousAttemptContext == null || !previousAttemptContext.getCommand().equals(currentAttemptContext.getCommand())) {
                        //trace is different, different route to failure
                        retryingContextExecution = -1;
                        return true;
                    }
                }
                //Same trace so far, check if previous trace was longer
                Context traceRoot = childContexts.get(childrenSize - count * 2);
                if (traceRoot == null || isSelectedRecovery(traceRoot)) {
                    return false;
                } else {
                    retryingContextExecution = -1;
                    return true;
                }
            }
            count += 1;
        }
        //Shouldn't reach this
        return false;
    }

    /**
     * Helper method to determine whether the supplied context is entering a repeated recovery loop
     */
    private boolean isLoopingRecovery(Context context, ActionStatus failureStatus, Justification failureJustification) {
        //Rewrite this section if recovery interface is changed/moved
        //Prevent application of a recovery policy from within execution of that same recovery policy
        Context parent = context.getParentContext();
        while (parent != null) {
            Recovery recovery;
            if (parent instanceof ActionContext) {
                recovery = ((ActionContext)parent).getRecovery();
            } else if (parent instanceof GoalContext) {
                recovery = ((GoalContext)parent).getRecovery();
            } else {
                parent = parent.getParentContext();
                continue;
            }
            if (recovery.getStatus() == RecoveryStatus.RECOVER) {
                return failureStatus.compareTo(recovery.originalFailureStatus) == 0 &&
                        failureJustification.equals(recovery.originalFailureJustification) &&
                        selectedRecoveryAction.equals(recovery.selectedRecoveryAction);
            }
            parent = parent.getParentContext();
        }
        return false;
    }
}
