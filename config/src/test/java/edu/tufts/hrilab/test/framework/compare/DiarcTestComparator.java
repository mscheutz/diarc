/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.test.framework.compare;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public abstract class DiarcTestComparator {
  protected Logger log = LoggerFactory.getLogger(this.getClass());

  protected String testName;
  protected volatile boolean isDone = false;
  protected volatile boolean testResult = false;

  /**
   * Reset comparator so it can be re-used.
   */
  protected void reset() {
    isDone = false;
    testResult = false;
  }

  public void setTestName(String testName) {
    this.testName = testName;
  }

  public String getTestName() {
    return testName;
  }

  public boolean isDone() {
    return isDone;
  }

  public boolean getResult() {
    return testResult;
  }

  public abstract void handleNextOutput(Object[] nextOutput);

  public abstract boolean unobservedCallsExist();

  public abstract String getUnobservedCallsString();
}