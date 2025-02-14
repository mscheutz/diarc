/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.forklift.messages;

import id.jrosmessages.Message;
import id.jrosmessages.MessageMetadata;
import id.jrosmessages.RosInterfaceType;

@MessageMetadata(name = DeliverResponseMessage.NAME, interfaceType = RosInterfaceType.SERVICE)
public class DeliverResponseMessage implements Message {
  static final String NAME = "crayler_nav_msgs/ControllerCommandServiceResponse"; //TODO: UPDATE
  public boolean success;
  public String message;

  public DeliverResponseMessage() {

  }
}

