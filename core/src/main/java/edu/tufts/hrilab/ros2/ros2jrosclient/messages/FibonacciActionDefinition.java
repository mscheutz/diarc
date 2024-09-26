/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.ros2.ros2jrosclient.messages;

import pinorobotics.jros2actionlib.actionlib_msgs.Action2Definition;
import pinorobotics.jros2actionlib.actionlib_msgs.Action2GetResultRequestMessage;
import pinorobotics.jros2actionlib.actionlib_msgs.Action2GoalMessage;
import pinorobotics.jros2actionlib.actionlib_msgs.Action2ResultMessage;

public class FibonacciActionDefinition
        implements Action2Definition<FibonacciGoalMessage, FibonacciResultMessage> {

    @Override
    public Class<? extends Action2GoalMessage<FibonacciGoalMessage>> getActionGoalMessage() {
        return FibonacciActionGoalMessage.class;
    }

    @Override
    public Class<? extends Action2ResultMessage<FibonacciResultMessage>> getActionResultMessage() {
        return FibonacciActionResultMessage.class;
    }

    @Override
    public Class<? extends Action2GetResultRequestMessage> getActionResultRequestMessage() {
        return FibonacciActionGetResultRequestMessage.class;
    }
}
