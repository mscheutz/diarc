/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.test.framework.tester;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.test.framework.compare.DiarcTestComparator;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/**
 * Core DIARC config testing class.
 */
public class DiarcConfigTester {

  protected static Logger log = LoggerFactory.getLogger(DiarcConfigTester.class);
  /**
   * Default duration before a test times out (seconds).
   */
  private long timeoutDuration = 600;
  private TimeUnit timeoutUnit = TimeUnit.SECONDS;

  /**
   * Executor service to manage thread running the test.
   */
  private ExecutorService testExecutor = Executors.newSingleThreadExecutor();
  /**
   * Queue of observed output during test execution.
   */
  private final Queue<Object[]> observedOutput = new ConcurrentLinkedDeque<>();
  /**
   * Lock for observedOutput queue.
   */
  private final Lock observedOutputEmptyLock = new ReentrantLock();
  /**
   * Lock condition for observedOutput queue.
   */
  private final Condition observedOutputNotEmpty = observedOutputEmptyLock.newCondition();

  /**
   * Set the timeout for a single input, not a single @Test.
   * @param timeoutDuration
   * @param unit
   */
  public void setTimeoutDuration(long timeoutDuration, TimeUnit unit) {
    this.timeoutDuration = timeoutDuration;
    this.timeoutUnit = unit;
  }

  public TimeUnit getTimeoutUnit() {
    return this.timeoutUnit;
  }

  public long getTimeoutDuration() {
    return this.timeoutDuration;
  }

  /**
   * Register's methods with TRADE so that addObservedOutput can be called to collect observed output.
   */
  public void registerWithTRADE() {
    try {
      TRADE.registerAllServices(this,new ArrayList<>());
    } catch (TRADEException e) {
      log.error("error registering TRADE Services", e);
    }
  }

  /**
   * This method should be used as a before wrapper for calls that we want to confirm are receiving the correct arguments.
   *
   * @param args method name of service that is being wrapped followed by its args.
   */
  @TRADEService
  public void addObservedOutput(Object[] args) {
    log.debug("adding observed output:  " + Arrays.deepToString(args));

    observedOutput.add(args);
    //get the lock
    observedOutputEmptyLock.lock();
    try {
      //signal that it is not empty
      observedOutputNotEmpty.signal();
      log.trace("signaled");
    } finally {
      //unlock
      observedOutputEmptyLock.unlock();
      log.trace("unlocked in event");
    }
  }

  /**
   * Main entry point to evaluate integration test results. The comparator must
   * be populated with the expected output.
   *
   * @param comparator
   */
  public void evaluateResults(DiarcTestComparator comparator) {
    Future<Boolean> future = testExecutor.submit(() -> runEvaluation(comparator));
    waitForResults(future);
  }

  /**
   * Helper method that handles passing observed output into the comparator, monitors
   * for test completion, and returns comparator results.
   *
   * @param comparator
   * @return
   */
  private Boolean runEvaluation(DiarcTestComparator comparator) {
    while (!comparator.isDone()) {
      log.debug("[runEvaluation] top of loop: " + comparator.getTestName());
      // need to this cancel thread if test has timed out (in case interruption exception for await doesn't get entered)
      if (Thread.currentThread().isInterrupted()) {
        log.warn("[runEvaluation] Thread interrupted. Test did not complete.");
        if (comparator.unobservedCallsExist()) {
          log.error("unobservedCalls: " + comparator.getUnobservedCallsString());
        }
        return false;
      }

      if (observedOutput.isEmpty()) {
        observedOutputEmptyLock.lock();
        try {
          log.debug("waiting on input");
          observedOutputNotEmpty.await();
        } catch (InterruptedException e) {
          log.error("[runEvaluation] interrupted while waiting on input. Test did not complete.");
          if (comparator.unobservedCallsExist()) {
            log.error("unobservedCalls: " + comparator.getUnobservedCallsString());
          }
          return false; // exit test
        } finally {
          observedOutputEmptyLock.unlock();
        }
      } else {
        //get results from observedOutput
        Object[] currentObservedOutput = observedOutput.remove();
        log.debug("Testing observed output: " + Arrays.toString(currentObservedOutput));

        comparator.handleNextOutput(currentObservedOutput);
      }
    }

    // finally, return test results
    log.debug("[runEvaluation] returning results for test: " + comparator.getTestName() +" result: "+comparator.getResult());

    return comparator.getResult();
  }

  /**
   * By the time this method returns the test will have succeeded and the assertion will have been true.
   * It would have failed and the assertion would be false (Found everything that was expected and more).
   * Or it would have timed out (didn't find everything that was expected.)
   * <p>
   * This method converts everything to Strings, so classes in tracked calls need to have a meaningful
   * tString implementation. it is intended to be used with demo snapshot based tests.
   *
   * @param result result of callable to wait on
   */
  protected void waitForResults(Future<Boolean> result) {

    try {
      //wait to collect input
      log.debug("waiting for test results");
      assertTrue("Did not find matching results.",result.get(timeoutDuration, timeoutUnit));
    } catch (TimeoutException te) {
      log.error("Timed out waiting for test results.");
      result.cancel(true);
      fail("Did not find matching results in time.");
    }
    catch (Exception e) {
      log.error("Exception waiting for test results.", e);
      result.cancel(true);
      fail("Exception waiting for test results.");
    }
  }

  /**
   * Clears any observed outputs so a new set can be collected.
   */
  public final void clearObservedOutput() {
    observedOutput.clear();
  }

  /**
   * Shutdown the tester by attempting to shutdown the tester thread and
   * degistering from TRADE.
   */
  public void shutdown() {
    observedOutput.clear();
    testExecutor.shutdown();
    try {
      testExecutor.awaitTermination(20, TimeUnit.SECONDS);
    } catch (InterruptedException e) {
      throw new RuntimeException(e);
    }
    try {
      TRADE.deregister(this);
    } catch (TRADEException e) {
      log.error("[shutdown]", e);
    }
  }

}
