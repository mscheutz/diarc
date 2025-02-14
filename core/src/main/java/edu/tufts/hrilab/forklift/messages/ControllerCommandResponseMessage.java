/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.forklift.messages;

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

