/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.demos;

import edu.tufts.hrilab.action.execution.ExecutionType;
import edu.tufts.hrilab.config.MockTwoNaoDemo;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.test.framework.GenerativeDiarcIntegrationTest;
import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import java.util.concurrent.TimeUnit;


public class TwoNaoDemoTest extends GenerativeDiarcIntegrationTest {
  private MockTwoNaoDemo diarcConfig;
  private SimSpeechRecognitionComponent speechInputTrusted;
  private SimSpeechRecognitionComponent speechInputUntrusted;

  @Before
  public void initializeDiarc() {
    // instantiate DIARC configuration (in mock)
    diarcConfig = new MockTwoNaoDemo();
    diarcConfig.runConfiguration();

    speechInputTrusted = diarcConfig.trustedSpeechRec;
    speechInputUntrusted = diarcConfig.untrustedSpeechRec;

    // the services we wish to observe
    addServiceToObserve("assertBelief", Term.class);
    addServiceToObserve("retractBelief", Term.class);
    addServiceToObserve("submitGoal", Predicate.class);
    addServiceToObserve("submitGoal", Predicate.class, ExecutionType.class, Symbol.class);
    addServiceToObserve("joinOnGoal", Long.class);
    addServiceToObserve("sayText", String.class);

    setPermanentTimeout(10, TimeUnit.SECONDS);
  }

  @After
  public void shutdownDiarc() {
    log.debug("[shutdownDiarc] started");
    diarcConfig.shutdownConfiguration();
    log.debug("[shutdownDiarc] completed");
  }

  //This wrapper exists so that the generator can appropriately catch input
  public void sendTrustedUserInput(String input) {
    tester.markNewInput();
    speechInputTrusted.setText(input);
    evaluateResults();
  }

  public void sendUntrustedUserInput(String input) {
    tester.markNewInput();
    speechInputUntrusted.setText(input);
    evaluateResults();
  }

  //tests dialogue, and to some degree action
  @Test
  public void greetingTest() {
    sendTrustedUserInput("hello dempster");
    sendTrustedUserInput("hello shafer");
    sendUntrustedUserInput("hello shafer");
  }

  //this is really only testing the parser and probably can be removed,
  //but it is required for the demo to work...
  @Test
  public void directAddress() {
    sendTrustedUserInput("hello dempster");
    sendTrustedUserInput("stand");
    sendTrustedUserInput("shafer crouch");
  }

  @Test
  public void ISATest() {
    sendTrustedUserInput("hello dempster");
    sendTrustedUserInput("i want you to stand");
    sendTrustedUserInput("can you do a squat");
    setObstacle("dempster", true);
    sendTrustedUserInput("could you please walk forward");
    setObstacle("dempster", false);
    sendTrustedUserInput("could you please walk forward");
    sendTrustedUserInput("stop");
    sendTrustedUserInput("bye");
  }

  @Test
  public void questionsTest() {
    sendUntrustedUserInput("hello dempster");
    setObstacle("dempster", true);
    sendUntrustedUserInput("do you see an obstacle");
    setObstacle("dempster", false);
    sendUntrustedUserInput("do you see an obstacle now");
    setFloorSupport("dempster", true);
    sendUntrustedUserInput("do you see support");
    sendUntrustedUserInput("do you trust me");
    sendUntrustedUserInput("who do you trust");
    sendTrustedUserInput("do you trust me dempster");
  }

  @Test
  public void safetyCondition() {
    sendTrustedUserInput("hello dempster");
    setObstacle("dempster", true);
    sendTrustedUserInput("walk forward");
    setObstacle("dempster", false);
    setFloorSupport("dempster", false);
    sendTrustedUserInput("please walk forward");
    sendTrustedUserInput("i will catch you");
    sendTrustedUserInput("walk forward");
    sendTrustedUserInput("stop");
    sendTrustedUserInput("forget that I will catch you");
    sendTrustedUserInput("walk forward");
  }

  @Test
  public void permissionsTest() {
    sendUntrustedUserInput("hello dempster");
    sendUntrustedUserInput("please stand");
    sendUntrustedUserInput("walk backward");
    sendUntrustedUserInput("the area behind you is safe");
    sendTrustedUserInput("ravenna is trusted");
    sendUntrustedUserInput("the area behind you is safe");
    sendUntrustedUserInput("walk backward");
    sendUntrustedUserInput("stop");
    sendUntrustedUserInput("forget that the area behind you is safe");
    sendUntrustedUserInput("forget that evan is trusted");
    sendTrustedUserInput("forget that ravenna is trusted");
    sendUntrustedUserInput("walk backward");
    sendUntrustedUserInput("the area behind you is safe");
  }

  @Test
  public void actionLearningTest() {
    sendUntrustedUserInput("hello shafer");
    sendUntrustedUserInput("please nod");
    sendUntrustedUserInput("i will teach you how to nod");
    sendTrustedUserInput("i will teach you how to nod");
    sendTrustedUserInput("execute while learning");
    sendTrustedUserInput("look up");
    sendTrustedUserInput("look down");
    sendTrustedUserInput("look up");
    sendTrustedUserInput("look down");
    sendTrustedUserInput("that is how you nod");
    sendUntrustedUserInput("nod");
    sendUntrustedUserInput("nod dempster");
  }

  @Test
  public void actionModificationTest() {
    sendTrustedUserInput("hello dempster");
    sendTrustedUserInput("i will teach you how to do a squat");
    sendTrustedUserInput("raise your arms");
    sendTrustedUserInput("crouch down");
    sendTrustedUserInput("stand up");
    sendTrustedUserInput("that is how you do a squat");
    sendUntrustedUserInput("dempster do a squat");
    sendTrustedUserInput("lower your arms");
    sendTrustedUserInput("describe how to do a squat");
    sendTrustedUserInput("when you do a squat make sure you lower your arms after you stand up");
    sendUntrustedUserInput("do a squat");
    sendUntrustedUserInput("do a squat shafer");
  }

  @Test
  public void componentSharingTest() {
    sendUntrustedUserInput("shafer tell dempster to stand");
    setObstacle("dempster", true);
    sendUntrustedUserInput("does dempster see an obstacle");
    sendUntrustedUserInput("tell dempster to walk forward");
    setObstacle("dempster", false);
    sendUntrustedUserInput("does dempster see an obstacle now");
    sendUntrustedUserInput("can dempster look up");
  }

  @Test
  public void multiAgentInteraction2() {
    sendTrustedUserInput("stand dempster");
    sendTrustedUserInput("dance with shafer");
    sendTrustedUserInput("I will teach you how to dance with shafer");
    sendTrustedUserInput("raise your arms");
    sendTrustedUserInput("crouch down shafer");
    sendTrustedUserInput("lower your arms dempster");
    sendTrustedUserInput("stand up shafer");
//    sendTrustedUserInput("dempster that is how you dance with shafer"); // TODO: this leading direct address doesn't parse
    sendTrustedUserInput("that is how you dance with shafer dempster"); // TODO: this leading direct address doesn't parse
    sendTrustedUserInput("dance with shafer");
    sendTrustedUserInput("hello shafer");
    sendTrustedUserInput("dance with dempster");
  }

  /**
   * TODO: move this to a new integration test involving the fetch.
   */
  @Test
  @Ignore
  public void multiAgentInteraction3() {
    sendTrustedUserInput("hello dempster");
    sendTrustedUserInput("dance with shafer and andy");
    sendTrustedUserInput("I will teach you how to dance with shafer and andy");
    sendTrustedUserInput("look left");
    sendTrustedUserInput("look right shafer");
    sendTrustedUserInput("look up andy");
    sendTrustedUserInput("dempster that is how you dance with shafer and andy");
    sendTrustedUserInput("dance with shafer and andy");
    sendTrustedUserInput("hello andy");
    sendTrustedUserInput("dance with dempster and shafer");
  }
  /**
   * TODO: move this to a new integration test involving the fetch.
   */
  @Ignore("timing issues prevent test from working as intended")
  @Test
  public void objectLearning() {
    sendTrustedUserInput("andy do you see the knife");
    sendTrustedUserInput("this object is a knife");
    sendTrustedUserInput("point to the knife");
  }

  private void setObstacle(String name, boolean value) {
    log.info("setObstacle: " + value);
    switch (name) {
      case "dempster":
        diarcConfig.dempster.setObstacle(value);
        break;
      case "shafer":
        diarcConfig.shafer.setObstacle(value);
        break;
      default:
        log.warn("[setObstacle] Invalid agent name: " + name);
    }
  }

  private void setFloorSupport(String name, boolean value) {
    switch (name) {
      case "dempster":
        diarcConfig.dempster.setFloorSupport(value);
        break;
      case "shafer":
        diarcConfig.shafer.setFloorSupport(value);
        break;
      default:
        log.warn("[setSupport] Invalid agent name: " + name);
    }
  }

}
