/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.recovery;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.*;
import edu.tufts.hrilab.action.execution.GoalContext;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.action.selector.ActionSelector;
import edu.tufts.hrilab.action.state.StateMachine;

public class NormPlanningGoalRecovery extends GoalRecovery {

//    protected Context cachedContext;

    @Override
    public ParameterizedAction selectNextAction(Goal goal, ActionConstraints constraints, StateMachine stateMachine) {
        return ActionSelector.getInstance().selectActionForGoal(goal, constraints, stateMachine);
    }

    @Override
    public boolean shouldAttemptRecovery(ActionStatus actionStatus, Justification justification, GoalContext goalContext) {
        if(actionStatus.isFailure() && justification.getPredicates().get(0).getName().equals("violated_norms")) {
            recoveryStatus = RecoveryStatus.RECOVER;
//            this.cachedContext = goalContext;
            return true;
        }
        return false;
    }

    @Override
    public ParameterizedAction selectRecoveryPolicy(GoalContext goalContext) {
        Goal recoveryGoal = getExplorationGoal(goalContext);
        ParameterizedAction recoveryPolicy = selectNextAction(recoveryGoal, goalContext.getConstraints(), goalContext.getStateMachine());
        return recoveryPolicy;
    }

    private Goal getExplorationGoal(GoalContext goalContext) {
//        For now, we can potentially put the MCTS stuff here.
        try {
            TRADE.getAvailableService(new TRADEServiceConstraints().name("enableSim")).call(void.class); //enable simulation execution
            TRADE.getAvailableService(new TRADEServiceConstraints().name("getLowLevelPlan")).call(void.class,goalContext);
            TRADE.getAvailableService(new TRADEServiceConstraints().name("disableSim")).call(void.class); //disable simulation execution
        } catch(TRADEException te) {
            log.error("[getExplorationGoal]", te);
        }

        recoveryStatus = RecoveryStatus.PLAN;

        return null;
    }
}
