/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.forklift.messages;

import pinorobotics.jrosservices.msgs.ServiceDefinition;


public class DeliverServiceDefinition implements ServiceDefinition<DeliverRequestMessage, DeliverResponseMessage> {

  @Override
  public Class<DeliverRequestMessage> getServiceRequestMessage() {
    return DeliverRequestMessage.class;
  }

  @Override
  public Class<DeliverResponseMessage> getServiceResponseMessage() {
    return DeliverResponseMessage.class;
  }
}
