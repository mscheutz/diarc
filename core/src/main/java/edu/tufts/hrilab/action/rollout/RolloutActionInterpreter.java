/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.rollout;

import edu.tufts.hrilab.action.ActionInterpreter;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.execution.ActionContext;
import edu.tufts.hrilab.action.execution.Context;

import java.util.ArrayList;
import java.util.List;

public class RolloutActionInterpreter extends ActionInterpreter {

    protected int rolloutLength;
    protected int currStep = 0;

    /**
     * Create new ActionInterpreter instance from an action context.
     * @param step context to be executed (e.g. root context)
     */
    public static RolloutActionInterpreter createInterpreterFromContext(Goal goal, Context step, int rolloutLength) {
        return new RolloutActionInterpreter(goal, step, rolloutLength);
    }

    /**
     * Only ActionInterpreter constructor. All AIs should be constructed through
     * a factory method (e.g., createInterpreterFrom___(...)).
     *
     * @param goal
     * @param context
     */
    private RolloutActionInterpreter(Goal goal, Context context, int rolloutLength) {
        super(goal, context);
        this.rolloutLength = rolloutLength;
        setCurrentContext(context);
    }

    public static RolloutActionInterpreter createRolloutInterpreterFromExecutionTree(Goal goal, Context goalRoot, int rolloutLength) {
        RolloutActionInterpreter ai = new RolloutActionInterpreter(goal, goalRoot, rolloutLength);
        Context startStep = ai.createExecutionStack(goalRoot);
        ai.setCurrentStep(startStep);
        goal.setCurrentContext(startStep);
        return ai;
    }

    @Override
    protected Context runCycle(Context step) {
        if(step.isAction()
                && !((ActionContext)step).isScript()
                && ((ActionContext)step).getExecType().shouldExecute()) { // Action primitive
            this.currStep += 1;
            if(this.currStep > this.rolloutLength) {
                this.suspend();
                this.getRoot().setStatus(ActionStatus.CANCEL); //todo: Untested, but maybe what we want instead of halt
//                this.halt();
                return null;
            }
        }

        return super.runCycle(step);
    }

    @Override
    protected Context runAsyncCycle(Context step) {
        if(step.isAction()
                && !((ActionContext)step).isScript()
                && ((ActionContext)step).getExecType().shouldExecute()) { // Action primitive
            this.currStep += 1;
            if(this.currStep > this.rolloutLength) {
                this.getRoot().setStatus(ActionStatus.CANCEL); //todo: Untested, but maybe what we want instead of halt
                return null;
            }
        }

        return super.runAsyncCycle(step);
    }

    private void setCurrentContext(Context context) {
        List<Context> callers = new ArrayList<>();
        while(context.getParentContext() != null) {
            callers.add(context);
            context = context.getParentContext();
        }
        for(int i = callers.size() - 1; i > 0; i--) {
//            Don't actually run cycles -- that'll cause things to get popped from the call stack.
            callStack.before(callers.get(i));
            callers.get(i).setStatus(ActionStatus.PROGRESS);
        }
        currentStep = callers.get(0);
    }

}
