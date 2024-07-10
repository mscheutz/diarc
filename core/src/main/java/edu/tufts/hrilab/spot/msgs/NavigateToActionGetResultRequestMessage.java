/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.spot.msgs;

import id.jrosmessages.MessageMetadata;
import id.jrosmessages.RosInterfaceType;
import pinorobotics.jros2actionlib.actionlib_msgs.Action2GetResultRequestMessage;
import pinorobotics.jros2actionlib.actionlib_msgs.Action2GoalIdMessage;

/** Definition for ROS2 GoalIdMessage */
@MessageMetadata(
        name = NavigateToActionGetResultRequestMessage.NAME,
        interfaceType = RosInterfaceType.ACTION)
public class NavigateToActionGetResultRequestMessage implements Action2GetResultRequestMessage {

    static final String NAME = "spot_msgs/NavigateToActionGetResult";

    public Action2GoalIdMessage goal_id;

    @Override
    public NavigateToActionGetResultRequestMessage withGoalId(Action2GoalIdMessage goal_id) {
        this.goal_id = goal_id;
        return this;
    }
}
