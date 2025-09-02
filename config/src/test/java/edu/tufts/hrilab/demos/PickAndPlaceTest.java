/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.demos;

import edu.tufts.hrilab.action.execution.ExecutionType;
import edu.tufts.hrilab.config.PickAndPlaceMock;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.test.framework.GenerativeDiarcIntegrationTest;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class PickAndPlaceTest extends GenerativeDiarcIntegrationTest {

  protected static Logger log = LoggerFactory.getLogger(PickAndPlaceTest.class);
  private PickAndPlaceMock config;

  //components that we care about interacting with in tests
  private SimSpeechRecognitionComponent speechInput;

  //@Before is run before every @Test
  @Before
  public void initializeConfig() {
    //instantiate DIARCInstance (in mock)
    config = new PickAndPlaceMock();
    config.runConfiguration();
    speechInput = config.simspeech;

    //The services we wish to observe mapped to their arguments
    addServiceToObserve("assertBelief", Term.class);
    addServiceToObserve("retractBelief", Term.class);
    addServiceToObserve("submitGoal", Predicate.class);
    addServiceToObserve("submitGoal", Predicate.class, ExecutionType.class, Symbol.class);
    addServiceToObserve("openGripper");
    addServiceToObserve("closeGripper");
    addServiceToObserve("getCameraData", String.class);
    addServiceToObserve("moveToCognexTarget", List.class, Integer.class);
    addServiceToObserve("moveToCognexTarget", Symbol.class);
    addServiceToObserve("getDescriptorForID", Symbol.class);
    addServiceToObserve("sayText", String.class);
    addServiceToObserve("goToPose", ai.thinkingrobots.mtracs.util.MPose.class);
    //todo: there are almost certainly other services we want to track here.
    tester.setTimeoutDuration(6, TimeUnit.SECONDS);
  }

  @After
  public void shutdownDiarc() {
    log.debug("[shutdownDiarc] started");
    config.shutdownConfiguration();
    log.debug("[shutdownDiarc] completed");
  }

  //This wrapper exists so that the generator can appropriately catch input
  private void addUserInput(String input) {
    tester.markNewInput();
    speechInput.setText(input);
  }

  @Test
  public void pickAndPlaceScriptTest() {

    addUserInput("init");
    evaluateResults();

    addUserInput("get the red block to location three");
    evaluateResults();

    addUserInput("location location one");
    evaluateResults();

    addUserInput("go to location two");
    evaluateResults();

    addUserInput("pick up the yellow block");
    evaluateResults();

    addUserInput("go to location three");
    evaluateResults();

    addUserInput("put down the yellow block");
    evaluateResults();

    addUserInput("pick up the block");
    evaluateResults();

    addUserInput("nevermind");
    evaluateResults();

    addUserInput("the green block is at location one");
    evaluateResults();

    addUserInput("what blocks are you aware of");
    evaluateResults();

    addUserInput("move the green block from location one to location three");
    evaluateResults();

    addUserInput("yes");
    evaluateResults();

    addUserInput("block the red block");
    evaluateResults();
  }
}
