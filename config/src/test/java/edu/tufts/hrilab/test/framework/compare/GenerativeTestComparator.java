/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.test.framework.compare;

import edu.tufts.hrilab.test.framework.gson.ServiceCallInstance;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

public class GenerativeTestComparator extends DiarcTestComparator {
  private ServiceCallInstance[] expectedOutputs;
  protected List<ServiceCallInstance> unobservedCalls;

  /**
   * Observed goalId to expected goalId. Populated during submitGoal handling in order
   * to match goalIds between submitGoal (which contains the goal predicate) and
   * joinOnGoal (which only contains the goalId).
   */
  private Map<String, String> goalIdMap = new HashMap<>();

  /**
   * Set the expected output for the next test comparison. This also resets the isDone
   * and testResults flag, so that the same comparator can be re-used within and across tests.
   *
   * @param testName        - name of JUnit @Test. Used mainly for debugging.
   * @param expectedOutputs - expected outputs
   */
  public void setExpectedOutput(String testName, ServiceCallInstance[] expectedOutputs) {
    this.reset();
    this.testName = testName;
    this.expectedOutputs = expectedOutputs;
    this.unobservedCalls = new ArrayList<>(Arrays.asList(expectedOutputs));
  }

  @Override
  public void handleNextOutput(Object[] nextOutput) {
    log.info("Testing observed output: " + Arrays.toString(nextOutput));

    //convert observed output to Strings so it can be compared to the strings stored in JSON.
    ServiceCallInstance observedOutput = getObservedServiceCall(nextOutput);

    Iterator<ServiceCallInstance> expectedIterator = unobservedCalls.iterator();
    boolean matchFound = false;
    while (expectedIterator.hasNext() && !matchFound) {
      ServiceCallInstance expectedOutput = expectedIterator.next();
      log.debug("Checking for match for expected call: " + expectedOutput);

      if (expectedOutput.serviceName.equals(observedOutput.serviceName)) {

        //check if they match
        if (expectedOutput.serviceName.equals("submitGoal")) {
          List<String> newExpected = new ArrayList(Arrays.stream(observedOutput.serviceArgs).toList());
          newExpected.set(newExpected.size()-1, "GOALID");
          List<String> newObserved = new ArrayList(Arrays.stream(observedOutput.serviceArgs).toList());
          newObserved.set(newObserved.size()-1, "GOALID");
          if (newExpected.equals(newObserved)) {
            goalIdMap.put(observedOutput.serviceArgs[1], expectedOutput.serviceArgs[1]);
            expectedIterator.remove();
            matchFound = true;
          }
        } else if (expectedOutput.serviceName.equals("joinOnGoal")) {
          String expectedGoalId = expectedOutput.serviceArgs[0];
          String observedGoalId = observedOutput.serviceArgs[0];
          if (!goalIdMap.containsKey(observedGoalId)) {
            log.error("Goal ID not in map, submitGoal must not have been observed)");
          } else if (goalIdMap.get(observedGoalId).equals(expectedGoalId)) {
            // comparing the correct joinOnGoals
            if (expectedOutput.serviceArgs[1].equals(observedOutput.serviceArgs[1])) {
              // correct goal status observed
              expectedIterator.remove();
              matchFound = true;
            } else {
              // incorrect goal status observed
              matchFound = false;
              break;
            }
          }
        } else if (Arrays.equals(expectedOutput.serviceArgs, observedOutput.serviceArgs)) {
          log.debug("Found matching call: " + expectedOutput + " " + observedOutput);
          expectedIterator.remove();
          matchFound = true;
        }
      }
    }

    if (!matchFound) {
      log.error("Unexpected output observed: " + observedOutput);
      log.error("Unobserved calls: "+unobservedCalls);
      isDone = true;
      testResult = false;
    } else if (unobservedCalls.isEmpty()) {
      // observed everything that was expected (and nothing more)
      isDone = true;
      testResult = true;
    }
  }

  protected ServiceCallInstance getObservedServiceCall(Object[] nextOutput) {
    //convert observed output to Strings so it can be compared to the strings stored in JSON.
    String observedServiceName = nextOutput[0].toString();
    String[] observedServiceArgs = new String[nextOutput.length - 1];
    for (int i = 1; i < nextOutput.length; i++) {
      if (nextOutput[i] == null) {
        observedServiceArgs[i - 1] = "null";
      } else {
        observedServiceArgs[i - 1] = nextOutput[i].toString();
      }
    }
    return new ServiceCallInstance(observedServiceName, observedServiceArgs);
  }

  protected List<ServiceCallInstance> getUnobservedCalls() {
    return unobservedCalls;
  }

  @Override
  public boolean unobservedCallsExist() {
    return !unobservedCalls.isEmpty();
  }

  @Override
  public String getUnobservedCallsString() {
    return unobservedCalls.toString();
  }
}
