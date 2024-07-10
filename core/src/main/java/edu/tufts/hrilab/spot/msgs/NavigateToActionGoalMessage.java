/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.spot.msgs;

import id.jrosmessages.MessageMetadata;
import id.jrosmessages.RosInterfaceType;
import id.xfunction.XJson;
import pinorobotics.jros2actionlib.actionlib_msgs.Action2GoalIdMessage;
import pinorobotics.jros2actionlib.actionlib_msgs.Action2GoalMessage;

import java.util.Objects;

@MessageMetadata(name = NavigateToActionGoalMessage.NAME, interfaceType = RosInterfaceType.ACTION, fields = {"goal_id", "goal"})
public class NavigateToActionGoalMessage implements Action2GoalMessage<NavigateToGoalMessage> {

  static final String NAME = "spot_msgs/NavigateToActionGoal";

  public Action2GoalIdMessage goal_id;

  public NavigateToGoalMessage goal;

  @Override
  public NavigateToActionGoalMessage withGoalId(Action2GoalIdMessage goal_id) {
    this.goal_id = goal_id;
    return this;
  }

  @Override
  public NavigateToActionGoalMessage withGoal(NavigateToGoalMessage goal) {
    this.goal = goal;
    return this;
  }

  @Override
  public int hashCode() {
    return Objects.hash(goal_id, goal);
  }

  @Override
  public boolean equals(Object obj) {
    var other = (NavigateToActionGoalMessage) obj;
    return Objects.equals(goal_id, other.goal_id) && Objects.equals(goal, other.goal);
  }

  @Override
  public String toString() {
    return XJson.asString(
            "goal_id", goal_id,
            "goal", goal);
  }
}
