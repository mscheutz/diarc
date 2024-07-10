/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.spot.msgs;

import id.jrosmessages.Message;
import id.jrosmessages.MessageMetadata;
import id.jrosmessages.RosInterfaceType;
import id.jrosmessages.std_msgs.Int32Message;
import id.xfunction.XJson;

import java.util.Arrays;
import java.util.Objects;

@MessageMetadata(fields = {"success, message"}, name = NavigateToResultMessage.NAME, interfaceType = RosInterfaceType.ACTION)
public class NavigateToResultMessage implements Message {

  static final String NAME = "actionlib_tutorials/FibonacciResult";

  public boolean success;
  public String message;

}
