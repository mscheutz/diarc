package edu.tufts.hrilab.forklift.messages;

import pinorobotics.jrosservices.msgs.ServiceDefinition;

public class LoadServiceDefinition implements ServiceDefinition<LoadRequestMessage, LoadResponseMessage> {
  @Override
  public Class<LoadRequestMessage> getServiceRequestMessage() {
    return null;
  }

  @Override
  public Class<LoadResponseMessage> getServiceResponseMessage() {
    return null;
  }
}
