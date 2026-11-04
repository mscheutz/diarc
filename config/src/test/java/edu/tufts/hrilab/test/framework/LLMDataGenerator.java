/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.test.framework;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.test.framework.compare.GenerativeTestComparator;
import edu.tufts.hrilab.test.framework.gson.ServiceCallInstance;
import edu.tufts.hrilab.test.framework.tester.GenerativeDiarcConfigTester;
import edu.tufts.hrilab.test.framework.tester.LLMDataGenratorTester;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.junit.After;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Rule;
import org.junit.rules.TestName;

import java.util.concurrent.TimeUnit;

/**
 * Base class for generative integration tests. That is, integration tests
 * that can be executed in generation mode to produce the desired testing output,
 * as opposed to hand-writing the expected output.
 */
public class LLMDataGenerator {
  protected Logger log = LoggerFactory.getLogger(this.getClass());
  /**
   * Tester used for reading/writing generated JSONs and performing test execution.
   */
  protected static LLMDataGenratorTester tester;
  /**
   * Comparator used to compare expected output with observed output.
   */
  private static GenerativeTestComparator comparator;

  /**
   * Used to cache the underlying tester timeout when temporarily
   * setting the timeout using setSingleTestTimeout.
   */
  private Long permanentTimeoutDuration = null;
  private TimeUnit permanentTimeoutUnit = null;

  /**
   * Instantiate the DIARC Config Tester.
   */
  @BeforeClass
  public static void initTesterAndComparator() {
    // create and register tester
    tester = new LLMDataGenratorTester();
    tester.registerWithTRADE();

    // set default comparator
    comparator = new GenerativeTestComparator();
  }

  /**
   * Override the default comparator.
   *
   * @param comparator
   */
  protected void setComparator(GenerativeTestComparator comparator) {
    this.comparator = comparator;
  }

  /**
   * Set the temporary timeout used in a single @Test. The timeout is
   * applied for each input within a test, but will revert to the
   * permanent timeout set with setPermanentTimeout.
   *
   * @param timeoutDuration
   * @param unit
   */
  public void setSingleTestTimeout(long timeoutDuration, TimeUnit unit) {
    // cache permanent timeout values before temporarily setting them
    // set back to permanent values in @Before method
    permanentTimeoutDuration = tester.getTimeoutDuration();
    permanentTimeoutUnit = tester.getTimeoutUnit();

    tester.setTimeoutDuration(timeoutDuration, unit);
  }

  /**
   * Automatically revert to permanent timeout before next @Test.
   */
  @Before
  public void revertTimeout() {
    if (permanentTimeoutDuration != null && permanentTimeoutUnit != null) {
      tester.setTimeoutDuration(permanentTimeoutDuration, permanentTimeoutUnit);
    }
  }

  /**
   * Set the timeout for all subsequent tests. This will persist across @Tests.
   * This is a convenience method to setting the timeout directly in the tester instance.
   *
   * @param timeoutDuration
   * @param unit
   */
  public void setPermanentTimeout(long timeoutDuration, TimeUnit unit) {
    tester.setTimeoutDuration(timeoutDuration, unit);
    permanentTimeoutDuration = null;
    permanentTimeoutUnit = null;
  }

  /**
   * Compare the observed results with the expected results. When running in generative mode,
   * no comparisons are made, but the method sleeps for the duration specified in the tester instance.
   * This duration can be set per test using the setTemporaryTimeout method, or for all tests using the
   * setPermanentTimeout.
   */
  protected void evaluateResults() {
    if (tester.generativeMode) {
      // just wait for diarc to run
      // TODO: brad: is there a better way to do this?
      try {
        log.warn("sleeping to mimic wait for results");
        Thread.sleep(tester.getTimeoutUnit().toMillis(tester.getTimeoutDuration()));
      } catch (InterruptedException e) {
        log.error("exception waiting for TestStep output", e);
      }
    }
  }

  /**
   * Add TRADE service to list of calls to be tested.
   *
   * @param serviceName
   * @param serviceArgsClasses
   */
//  protected void addServiceToObserve(String serviceName, String[] serviceArgsClasses) {
//    try {
//      TRADE.afterService("addObservedOutput", new String[]{Object[].class.getName()}, new TRADEServiceConstraints(), serviceName, serviceArgsClasses, null);
//    } catch (TRADEException e) {
//      log.error("Exception adding before wrapper for service: " + serviceName, e);
//    }
//  }
//
//  protected void addServiceToObserve(String serviceName, String[] serviceArgsClasses, boolean before) {
//    if(before){
//      try {
//        TRADE.beforeService("addObservedOutput", new String[]{Object[].class.getName()}, new TRADEServiceConstraints(), serviceName, serviceArgsClasses, null);
//      } catch (TRADEException e) {
//        log.error("Exception adding before wrapper for service: " + serviceName, e);
//      }
//    }else {
//      try {
//        TRADE.afterService("addObservedOutput", new String[]{Object[].class.getName()}, new TRADEServiceConstraints(), serviceName, serviceArgsClasses, null);
//      } catch (TRADEException e) {
//        log.error("Exception adding before wrapper for service: " + serviceName, e);
//      }
//    }
//  }

  /**
   * Add TRADE service to list of calls to be tested, using the TRADEServiceConstraints to constrain the service calls
   * used in the test.
   *
   * @param serviceName
   * @param serviceArgsClasses
   * @param serviceConstraints
   */
//  protected void addServiceToObserve(String serviceName, String[] serviceArgsClasses, TRADEServiceConstraints serviceConstraints) {
//    try {
//      TRADE.afterService("addObservedOutput", new String[]{Object[].class.getName()}, new TRADEServiceConstraints(), serviceName, serviceArgsClasses, serviceConstraints);
//    } catch (TRADEException e) {
//      log.error("Exception adding before wrapper for service: " + serviceName, e);
//    }
//  }

  /**
   * Allows for access to test name within tests.
   */
  @Rule
  public TestName name = new TestName();

  /**
   * Notifies tester that a new @Test is starting.
   */
  @Before
  public void startTest() {
    tester.startTest(this.getClass(), name.getMethodName());
  }

  /**
   * Notifies tester that a @Test has ended.
   */
  @After
  public void endTest() {
    tester.endTest(this.getClass(), name.getMethodName());
  }

}
