/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.test.framework.gson;

import com.fasterxml.jackson.annotation.JsonProperty;

import java.util.Arrays;
import java.util.Objects;

/**
 * The string-ified version of a TRADE service call including the service name and service args (input and output).
 */
public class ServiceCallInstance {
  /**
   * TRADE service name.
   */
  public final String serviceName;
  /**
   * TRADE service args (input and output).
   */
  public final String[] serviceArgs;

  /**
   * Mainly used for instantiating from JSON.
   * @param serviceName
   * @param serviceArgs
   */
  public ServiceCallInstance(@JsonProperty("serviceName") String serviceName, @JsonProperty("serviceArgs") String[] serviceArgs) {
    this.serviceName = serviceName;
    this.serviceArgs = serviceArgs;
  }

  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (o == null || getClass() != o.getClass()) return false;
    ServiceCallInstance that = (ServiceCallInstance) o;
    return Objects.equals(serviceName, that.serviceName) && Arrays.equals(serviceArgs, that.serviceArgs);
  }

  @Override
  public int hashCode() {
    int result = Objects.hash(serviceName);
    result = 31 * result + Arrays.hashCode(serviceArgs);
    return result;
  }

  @Override
  public String toString() {
    return "CallInstance{" +
            "serviceName='" + serviceName + '\'' +
            ", serviceArgs=" + Arrays.toString(serviceArgs) +
            '}';
  }
}
