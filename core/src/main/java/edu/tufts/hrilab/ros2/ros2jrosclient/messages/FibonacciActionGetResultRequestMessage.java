/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.ros2.ros2jrosclient.messages;

import id.jrosmessages.MessageMetadata;
import id.jrosmessages.RosInterfaceType;
import pinorobotics.jros2actionlib.actionlib_msgs.Action2GetResultRequestMessage;
import pinorobotics.jros2actionlib.actionlib_msgs.Action2GoalIdMessage;

/** Definition for ROS2 GoalIdMessage */
@MessageMetadata(
        name = FibonacciActionGetResultRequestMessage.NAME,
        interfaceType = RosInterfaceType.ACTION)
public class FibonacciActionGetResultRequestMessage implements Action2GetResultRequestMessage {

    static final String NAME = "action_tutorials_interfaces/FibonacciActionGetResult";

    public Action2GoalIdMessage goal_id;

    @Override
    public FibonacciActionGetResultRequestMessage withGoalId(Action2GoalIdMessage goal_id) {
        this.goal_id = goal_id;
        return this;
    }
}
