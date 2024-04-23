/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.test.framework.gson;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.annotation.JsonProperty;

import java.util.ArrayList;
import java.util.List;

/**
 * This class contains all the observed service calls during a single @Test. Each @Test
 * consists of one or more testlets (user input, observed service calls).
 */
public class TestServiceCalls {
  /**
   * JUnit @Test name.
   */
  private String testName;
  /**
   * Collection of service call results from all testlets within a @Test.
   */
  private List<TestletServiceCalls> testlets;
  /**
   * To keep track of what teslet is currently being added to.
   */
  private int addIndex = -1;
  /**
   * To keep track of which teslet results to return next.
   */
  private int getterIndex = 0;

  /**
   * Instantiate class for a new @Test.
   * @param testName
   */
  public TestServiceCalls(String testName) {
    this.testName = testName;
    this.testlets = new ArrayList<>();
  }

  /**
   * Used to populate this class from a JSON.
   * @param testName
   * @param testlets
   */
  public TestServiceCalls(@JsonProperty("testName") String testName, @JsonProperty("testlets") List<TestletServiceCalls> testlets) {
    this.testName = testName;
    this.testlets = new ArrayList<>(testlets);
  }

  /**
   * Get the JUnit @Test name.
   * @return
   */
  public String getTestName() {
    return testName;
  }

  /**
   * Get a shallow copy of all the testlets.
   * @return
   */
  public List<TestletServiceCalls> getTestlets() {
    return new ArrayList<>(testlets);
  }

  /**
   * Called to indicate the start of a new testlet.
   */
  public void startNextTestlet() {
    testlets.add(new TestletServiceCalls());
    addIndex = testlets.size() - 1;
  }

  /**
   * Add an observed service call for the currently executing testlet.
   * @param observedServiceCall
   */
  public void addTestletServiceCall(ServiceCallInstance observedServiceCall) {
    this.testlets.get(addIndex).addObservedServiceCall(observedServiceCall);
  }

  /**
   * Get all the observed service calls for the next testlet.
   * @return
   */
  @JsonIgnore
  public ServiceCallInstance[] getNextOutputSet() {
    return testlets.get(getterIndex++).getServiceCalls();
  }
}
