/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.spot.msgs;


import id.jrosmessages.Message;
import id.jrosmessages.MessageMetadata;
import id.jrosmessages.RosInterfaceType;
import id.jrosmessages.std_msgs.UInt16Message;

@MessageMetadata(name = DockRequestMessage.NAME, interfaceType = RosInterfaceType.SERVICE)
public class DockRequestMessage implements Message {
  static final String NAME = "spot_msgs/DockServiceRequest";
  public UInt16Message dock_id = new UInt16Message();
  public DockRequestMessage() {
  }
}