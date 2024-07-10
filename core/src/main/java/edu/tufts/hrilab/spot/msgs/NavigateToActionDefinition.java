/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.spot.msgs;

import pinorobotics.jros2actionlib.actionlib_msgs.Action2Definition;
import pinorobotics.jros2actionlib.actionlib_msgs.Action2GetResultRequestMessage;
import pinorobotics.jros2actionlib.actionlib_msgs.Action2GoalMessage;
import pinorobotics.jros2actionlib.actionlib_msgs.Action2ResultMessage;

public class NavigateToActionDefinition
        implements Action2Definition<NavigateToGoalMessage, NavigateToResultMessage> {

    @Override
    public Class<? extends Action2GoalMessage<NavigateToGoalMessage>> getActionGoalMessage() {
        return NavigateToActionGoalMessage.class;
    }

    @Override
    public Class<? extends Action2ResultMessage<NavigateToResultMessage>> getActionResultMessage() {
        return NavigateToActionResultMessage.class;
    }

    @Override
    public Class<? extends Action2GetResultRequestMessage> getActionResultRequestMessage() {
        return NavigateToActionGetResultRequestMessage.class;
    }
}
