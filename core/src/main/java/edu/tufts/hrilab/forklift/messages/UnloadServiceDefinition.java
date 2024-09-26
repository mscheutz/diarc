package edu.tufts.hrilab.forklift.messages;

import pinorobotics.jrosservices.msgs.ServiceDefinition;

public class UnloadServiceDefinition implements ServiceDefinition<UnloadRequestMessage, UnloadResponseMessage> {
  @Override
  public Class<UnloadRequestMessage> getServiceRequestMessage() {
    return null;
  }

  @Override
  public Class<UnloadResponseMessage> getServiceResponseMessage() {
    return null;
  }
}
