/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.spot.msgs;

import pinorobotics.jrosservices.msgs.ServiceDefinition;

public class TriggerServiceDefinition
    implements ServiceDefinition<TriggerRequestMessage, TriggerResponseMessage> {

  @Override
  public Class<TriggerRequestMessage> getServiceRequestMessage() {
    return TriggerRequestMessage.class;
  }

  @Override
  public Class<TriggerResponseMessage> getServiceResponseMessage() {
    return TriggerResponseMessage.class;
  }
}
