/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.forklift.messages;

import pinorobotics.jrosservices.msgs.ServiceDefinition;


//ros2 service call /controller_service crayler_nav_msgs/srv/ControllerCommand "{end_x: 2.0,
// end_y: 11.0, end_yaw: 1.5, driving_direction: 0, command: 0}"
public class ControllerCommandServiceDefinition implements ServiceDefinition<ControllerCommandRequestMessage, ControllerCommandResponseMessage> {

  @Override
  public Class<ControllerCommandRequestMessage> getServiceRequestMessage() {
    return ControllerCommandRequestMessage.class;
  }

  @Override
  public Class<ControllerCommandResponseMessage> getServiceResponseMessage() {
    return ControllerCommandResponseMessage.class;
  }
}
