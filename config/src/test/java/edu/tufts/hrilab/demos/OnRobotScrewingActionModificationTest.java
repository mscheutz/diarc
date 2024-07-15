/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.demos;

import edu.tufts.hrilab.action.execution.ExecutionType;
import edu.tufts.hrilab.config.OnRobotScrewingActionModificationMock;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.test.framework.GenerativeDiarcIntegrationTest;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.junit.Before;
import org.junit.Test;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class OnRobotScrewingActionModificationTest extends GenerativeDiarcIntegrationTest {

  protected static Logger log = LoggerFactory.getLogger(OnRobotScrewingActionModificationTest.class);
  private OnRobotScrewingActionModificationMock config;

  //components that we care about interacting with in tests
  private SimSpeechRecognitionComponent speechInputBrad;

  //@Before is run before every @Test
  @Before
  public void initializeConfig() {
    //instantiate DIARCInstance (in mock)
    config = new OnRobotScrewingActionModificationMock(false, true);
    config.runConfiguration();
    speechInputBrad = config.simspeech;

    //The services we wish to observe mapped to their arguments
    addServiceToObserve("assertBelief", Term.class);
    addServiceToObserve("retractBelief", Term.class);
//    addServiceToObserve("runScrewdriverProgram", Integer.class);
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
    tester.setTimeoutDuration(5, TimeUnit.SECONDS);
  }

  //This wrapper exists so that the generator can appropriately catch input
  private void addUserInput(String input) {
    tester.markNewInput();
    speechInputBrad.setText(input);
  }

  @Test
  public void screwingActionModificationTestOrdered() {

//    private void demoSetupTest() {
    addUserInput("setup poses");
    evaluateResults();

//    private void screwingParameterTeachingTest() {
    addUserInput("Define new screw type deep M3");
    evaluateResults();

    addUserInput("4 mm");
    evaluateResults();

    addUserInput("150 millinewton meters");
    evaluateResults();

    addUserInput("pose screw feeder");
    evaluateResults();

    addUserInput("define new item NV30FAU");
    evaluateResults();

    addUserInput("job n v face");
    evaluateResults();


//    private void screwingTeachingTest() {
    addUserInput("I will teach you how to assemble a NV30FAU");
    evaluateResults();

    addUserInput("first go to pose conveyor");
    evaluateResults();

    addUserInput("then verify that you can see the NV30FAU");
    evaluateResults();

    addUserInput("then get the NV30FAU on the work area");
    evaluateResults();

    addUserInput("then search for 2 mounting holes");
    evaluateResults();

    addUserInput("screw a deep M3 into the bottom mounting hole");
    evaluateResults();

    addUserInput("then go to pose work area");
    evaluateResults();

    addUserInput("screw a deep M3 into the top mounting hole");
    evaluateResults();

    addUserInput("then get the NV30FAU on the conveyor");
    evaluateResults();

    addUserInput("then advance the conveyor belt");
    evaluateResults();

    addUserInput("that is how you assemble a NV30FAU");
    evaluateResults();

    addUserInput("robot one assemble a NV30FAU");
    evaluateResults();

    addUserInput("robot two assemble a NV30FAU");
    evaluateResults();

    addUserInput("Define new screw type M3");
    evaluateResults();

    addUserInput("3 mm");
    evaluateResults();

    addUserInput("150 millinewton meters");
    evaluateResults();

    addUserInput("pose screw feeder");
    evaluateResults();

    addUserInput("define new item NF32SV");
    evaluateResults();

    addUserInput("job circuit breaker face");
    evaluateResults();

//    private void modifiedAssemblyTeachingTest() {
    addUserInput("assemble an NF32SV is like assemble an NV30FAU");
    evaluateResults();

    addUserInput("replace search for 2 mounting holes with search for 2 m3 holes");
    evaluateResults();

    addUserInput("replace screw a deep M3 into the bottom mounting hole with screw an M3 screw into the left M3 hole");
    evaluateResults();

    addUserInput("replace screw a deep M3 into the top mounting hole with screw an M3 screw into the right M3 hole");
    evaluateResults();

    addUserInput("that is all");
    evaluateResults();

    addUserInput("robot one assemble an NF32SV");
    evaluateResults();

    addUserInput("robot two assemble an NF32SV");
    evaluateResults();
  }
}
