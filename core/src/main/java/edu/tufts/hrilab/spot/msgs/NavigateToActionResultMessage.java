/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.spot.msgs;

import id.jrosmessages.MessageMetadata;
import id.jrosmessages.RosInterfaceType;
import id.xfunction.XJson;
import pinorobotics.jros2actionlib.actionlib_msgs.Action2ResultMessage;
import pinorobotics.jros2actionlib.actionlib_msgs.StatusType;

import java.util.Objects;


@MessageMetadata(name = NavigateToActionResultMessage.NAME, interfaceType = RosInterfaceType.ACTION,         fields = {"status", "result"}

)
public class NavigateToActionResultMessage implements Action2ResultMessage<NavigateToResultMessage> {

  static final String NAME = "spot_msgs/NavigateToActionResult";

  public byte status;

  public NavigateToResultMessage result;

  @Override
  public int hashCode() {
    return Objects.hash(status, result);
  }

  @Override
  public boolean equals(Object obj) {
    var other = (NavigateToActionResultMessage) obj;
    return Objects.equals(status, other.status) && Objects.equals(result, other.result);
  }

  @Override
  public String toString() {
    return XJson.asString(
            "status", status,
            "result", result);
  }

  @Override
  public StatusType getStatus() {
    return StatusType.values()[status];
  }

  @Override
  public NavigateToResultMessage getResult() {
    return result;
  }
}
