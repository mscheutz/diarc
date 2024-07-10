/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.spot.msgs;

import id.jrosmessages.Message;
import id.jrosmessages.MessageMetadata;
import id.jrosmessages.RosInterfaceType;
import id.jrosmessages.std_msgs.StringMessage;
import id.xfunction.XJson;

import java.util.Objects;

@MessageMetadata(name = DockResponseMessage.NAME, interfaceType = RosInterfaceType.SERVICE, fields = {"success", "message"})
public class DockResponseMessage implements Message {
  static final String NAME = "spot_msgs/DockResponseMessage";

  public boolean success;
  public StringMessage message = new StringMessage();

  public DockResponseMessage() {
  }

  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (o == null || getClass() != o.getClass()) return false;
    DockResponseMessage that = (DockResponseMessage) o;
    return success == that.success && Objects.equals(message, that.message);
  }

  @Override
  public int hashCode() {
    return Objects.hash(success, message);
  }

  @Override
  public String toString() {
    return XJson.asString(
            "success", success,
            "message", message
    );
  }
}

