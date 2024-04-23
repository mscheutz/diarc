/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.test.framework.compare;

import edu.tufts.hrilab.test.framework.gson.ServiceCallInstance;

import java.util.Arrays;
import java.util.Iterator;

public class FetchPerformanceTestComparator extends GenerativeTestComparator {

  @Override
  public void handleNextOutput(Object[] nextOutput) {

    ServiceCallInstance observedOutput = getObservedServiceCall(nextOutput);

    boolean matchFound = false;
    if (observedOutput.serviceName.equals("sayText")) {
      log.info("Testing observed output: " + Arrays.toString(nextOutput));
      Iterator<ServiceCallInstance> expectedIterator = getUnobservedCalls().iterator();
      while (expectedIterator.hasNext() && !matchFound) {
        ServiceCallInstance expectedOutput = expectedIterator.next();
        if (expectedOutput.serviceName.equals(observedOutput.serviceName)) {
          matchFound = sayTextMatches(expectedOutput, observedOutput);
          if (matchFound) {
            expectedIterator.remove();
          }
        }
      }

      if (!matchFound) {
        log.error("Unexpected output observed: " + observedOutput);
        log.error("Unobserved calls: " + unobservedCalls);
        isDone = true;
        testResult = false;
      } else if (getUnobservedCalls().isEmpty()) {
        // observed everything that was expected (and nothing more)
        isDone = true;
        testResult = true;
      }

    } else {
      super.handleNextOutput(nextOutput);
    }
  }

  private boolean sayTextMatches(ServiceCallInstance expectedOutput, ServiceCallInstance observedOutput) {
    boolean matches = false;
    String expectedSayText;
    String observedSayText;

    //todo: is there a better way to handle the case where you have an agent on the sayText service?
    if (expectedOutput.serviceArgs.length > 2) {
      expectedSayText = expectedOutput.serviceArgs[1];
      observedSayText = observedOutput.serviceArgs[1];
    } else {
      expectedSayText = expectedOutput.serviceArgs[0];
      observedSayText = observedOutput.serviceArgs[0];
    }
    String durText = "it will take";
    String probText = "Probability";

    // remove quotes if they exist
    if (expectedSayText.startsWith("\"")) {
      expectedSayText = expectedSayText.substring(1, expectedSayText.length()-1);
    }

    if (expectedSayText.contains(probText) && observedSayText.contains(probText)) {
      String[] splitText = expectedSayText.split(" ");
      double expProbValue = Double.parseDouble(splitText[splitText.length - 1]);
      String[] splitObs = observedSayText.split(" ");
      double obsProbValue = Double.parseDouble(splitText[splitObs.length - 1]);
      matches = Math.abs(expProbValue - obsProbValue) < 0.05;
    } else if (expectedSayText.contains(durText) && observedSayText.contains(durText)) {
      // it will take about X minutes and Y seconds
      String[] splitText = expectedSayText.split("about ")[1].split(" ");
      double expTime = Integer.parseInt(splitText[0]) * 60 + Integer.parseInt(splitText[splitText.length - 2]);
      String[] obsSplitText = expectedSayText.split("about ")[1].split(" ");
      double obsTime = Integer.parseInt(obsSplitText[0]) * 60 + Integer.parseInt(obsSplitText[obsSplitText.length - 2]);
      matches = Math.abs(expTime - obsTime) < 5;
    } else if (Arrays.equals(expectedOutput.serviceArgs, observedOutput.serviceArgs)) {
      log.debug("Found matching call: " + expectedOutput + " " + observedOutput);
      matches = true;
    }
    return matches;
  }
}
