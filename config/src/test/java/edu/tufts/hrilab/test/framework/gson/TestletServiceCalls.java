/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.test.framework.gson;

import com.fasterxml.jackson.annotation.JsonProperty;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TestletServiceCalls {
  private List<ServiceCallInstance> serviceCalls;

  public TestletServiceCalls() {
    this.serviceCalls = new ArrayList<>();
  }

  public TestletServiceCalls(@JsonProperty("serviceCalls") ServiceCallInstance[] serviceCalls) {
    this.serviceCalls = Arrays.asList(serviceCalls);
  }

  public void addObservedServiceCall(ServiceCallInstance callInstance) {
    serviceCalls.add(callInstance);
  }

  public ServiceCallInstance[] getServiceCalls() {
    return serviceCalls.toArray(new ServiceCallInstance[0]);
  }
}
