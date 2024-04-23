/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ros2.ros2jrosclient.generated;


import id.jrosmessages.Message;
import id.jrosmessages.MessageMetadata;
import id.jrosmessages.RosInterfaceType;
import id.xfunction.XJson;

import java.util.Objects;

@MessageMetadata(name = AddTwoIntsRequestMessage.NAME, interfaceType = RosInterfaceType.SERVICE)
public class AddTwoIntsRequestMessage implements Message {

  static final String NAME = "example_interfaces/AddTwoIntsServiceRequest";
  public long a;
  public long b;

  public AddTwoIntsRequestMessage() {
  }

  public AddTwoIntsRequestMessage(long a, long b) {
    this.a = a;
    this.b = b;
  }

  @Override
  public int hashCode() {
    return Objects.hash(a, b);
  }

  @Override
  public boolean equals(Object obj) {
    var other = (AddTwoIntsRequestMessage) obj;
    return Objects.equals(a, other.b) && Objects.equals(a, other.b);
  }

  @Override
  public String toString() {
    return XJson.asString(
        "a", a,
        "b", b);
  }
}