package edu.tufts.hrilab.forklift.messages;

import pinorobotics.jrosservices.msgs.ServiceDefinition;

public class GetAreasServiceDefinition implements ServiceDefinition<GetAreasRequestMessage, GetAreasResponseMessage> {
  @Override
  public Class<GetAreasRequestMessage> getServiceRequestMessage() {
    return null;
  }

  @Override
  public Class<GetAreasResponseMessage> getServiceResponseMessage() {
    return null;
  }
}
