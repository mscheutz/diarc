/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ros2.ros2jrosclient.generated;

import pinorobotics.jrosservices.msgs.ServiceDefinition;

public class AddTwoIntsServiceDefinition
    implements ServiceDefinition<AddTwoIntsRequestMessage, AddTwoIntsResponseMessage> {

  @Override
  public Class<AddTwoIntsRequestMessage> getServiceRequestMessage() {
    return AddTwoIntsRequestMessage.class;
  }

  @Override
  public Class<AddTwoIntsResponseMessage> getServiceResponseMessage() {
    return AddTwoIntsResponseMessage.class;
  }
}
