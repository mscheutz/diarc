/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.test.framework.compare;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.fol.Factory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.ListIterator;
import java.util.Set;
import java.util.stream.Collectors;

/**
 * Comparator for integration tests that have hard coded expected results (vs. auto-generated expected results).
 *
 * This should be merged with the GenerativeTestComparator once the JSON read/write functionality uses Object[]
 * instead of Callable[]
 */
public class HardCodedTestComparator extends DiarcTestComparator {
  protected Set<Object[]> expectedOutputs;
  private List<Object[]> unobservedCalls;

  /**
   * Set the expected output for the next test comparison. This also resets the isDone
   * and testResults flag, so that the same comparator can be re-used within and across tests.
   *
   * @param testName        - name of JUnit @Test. Used mainly for debugging.
   * @param expectedOutputs - expected outputs
   */
  public void setExpectedOutput(String testName, Object[][] expectedOutputs) {
    this.reset();
    this.testName = testName;
    this.expectedOutputs = new HashSet<>(Arrays.asList(expectedOutputs));
    this.unobservedCalls = new ArrayList<>(this.expectedOutputs);
  }

  @Override
  public void handleNextOutput(Object[] nextOutput) {
    //get results from observedOutput
    log.debug("Testing observed output: " + Arrays.toString(nextOutput));

    // This allows expected output for joinOnGoal to use either the standard joinOnGoal (using the goalId),
    // or a test-only alternative using the goal predicate
    Object[] joinOnGoalAlternate = new Object[3];
    if (nextOutput[0].equals("joinOnGoal")) {
      long goalId = (long) nextOutput[1];
      Goal goal = getGoal(goalId);
      if (goal != null) {
        joinOnGoalAlternate[0] = "joinOnGoal";
        if (goal.isAction()) {
          joinOnGoalAlternate[1] = goal.getPredicate();
        } else if (goal.isObservation()) {
          joinOnGoalAlternate[1] = Factory.createPredicate(("obs"), goal.getActor(), goal.getPredicate());
        } else {
          joinOnGoalAlternate[1] = Factory.createPredicate(("goal"), goal.getActor(), goal.getPredicate());
        }
        joinOnGoalAlternate[2] = nextOutput[2]; // goal status
      }
    }

    boolean foundMatch = false;
    ListIterator<Object[]> expectedIterator = unobservedCalls.listIterator();
    while (expectedIterator.hasNext()) {
      Object[] expectedOutput = expectedIterator.next();
      log.debug("Checking for match for expected call: " + Arrays.deepToString(expectedOutput));

      //check if they match
      if (Arrays.equals(expectedOutput, nextOutput) || Arrays.equals(expectedOutput, joinOnGoalAlternate)) {
        expectedIterator.remove();
        //if they match remove this instance of expected output.
        log.debug("found matching call: " + Arrays.toString(expectedOutput) + " " + Arrays.deepToString(nextOutput));
        foundMatch = true;
        break;
      }
    }

    if (!foundMatch) {
      isDone = true;
      testResult = false;

      log.error("finished with unexpected observations");
      log.error("unexpectedObservations: " + Arrays.deepToString(nextOutput));
      log.error("unobservedCalls: " + getUnobservedCallsString());
    } else if (unobservedCalls.isEmpty()) {
      // observed everything that was expected (and nothing more)
      isDone = true;
      testResult = true;
    }

    log.debug("unobservedCalls: " + getUnobservedCallsString());
  }

  /**
   * Helper method to get Goal from a goal ID.
   *
   * @param goalId
   * @return
   */
  private Goal getGoal(long goalId) {
    Goal goal = null;
    try {
      goal = TRADE.getAvailableService(new TRADEServiceConstraints().name("getGoal").argTypes(Long.class)).call(Goal.class, goalId);
    } catch (TRADEException e) {
      log.error("Error trying to get Goal for goalId: " + goalId, e);
    }
    return goal;
  }

  @Override
  public boolean unobservedCallsExist() {
    return !unobservedCalls.isEmpty();
  }

  @Override
  public String getUnobservedCallsString() {
    return unobservedCalls.stream().map(Arrays::deepToString).collect(Collectors.toList()).toString();
  }

}
