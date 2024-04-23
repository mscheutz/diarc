/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ros2.ros2jrosclient.generated;

import id.jrosmessages.Message;
import id.jrosmessages.MessageMetadata;
import id.jrosmessages.RosInterfaceType;

@MessageMetadata(name = ControllerCommandResponseMessage.NAME, interfaceType = RosInterfaceType.SERVICE)
public class ControllerCommandResponseMessage implements Message {
  static final String NAME = "crayler_nav_msgs/ControllerCommandServiceResponse";
  public boolean success;

  public ControllerCommandResponseMessage() {

  }
}

