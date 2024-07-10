/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.spot.msgs;

import pinorobotics.jrosservices.msgs.ServiceDefinition;

public class DockServiceDefinition
    implements ServiceDefinition<DockRequestMessage, DockResponseMessage> {

  @Override
  public Class<DockRequestMessage> getServiceRequestMessage() {
    return DockRequestMessage.class;
  }

  @Override
  public Class<DockResponseMessage> getServiceResponseMessage() {
    return DockResponseMessage.class;
  }
}
