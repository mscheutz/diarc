/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ros2.ros2jrosclient.messages;

import id.jrosmessages.Message;
import id.jrosmessages.MessageMetadata;
import id.jrosmessages.RosInterfaceType;
import id.xfunction.XJson;

import java.util.Objects;

@MessageMetadata(name = AddTwoIntsResponseMessage.NAME, interfaceType = RosInterfaceType.SERVICE)
public class AddTwoIntsResponseMessage implements Message {
  static final String NAME = "example_interfaces/AddTwoIntsServiceResponse";

  public long sum;

  public AddTwoIntsResponseMessage() {

  }

  @Override
  public int hashCode() {
    return Objects.hash(sum);
  }

  @Override
  public boolean equals(Object obj) {
    var other = (AddTwoIntsResponseMessage) obj;
    return sum == other.sum;
  }

  @Override
  public String toString() {
    return XJson.asString("sum", sum);
  }
}

