/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.spot.msgs;


import id.jrosmessages.Message;
import id.jrosmessages.MessageMetadata;
import id.jrosmessages.RosInterfaceType;
import id.xfunction.XJson;

import java.util.Objects;

@MessageMetadata(name = TriggerRequestMessage.NAME, interfaceType = RosInterfaceType.SERVICE)
public class TriggerRequestMessage implements Message {
  static final String NAME = "std_srvs/TriggerServiceRequest";

  public TriggerRequestMessage() {
  }
}