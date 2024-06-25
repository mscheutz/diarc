/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.demos;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import edu.tufts.hrilab.action.GoalManagerImpl;
import edu.tufts.hrilab.action.db.performanceMeasures.PerformanceMeasures;
import edu.tufts.hrilab.config.FetchPerformanceAssessmentDemoMock;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;

import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.test.framework.GenerativeDiarcIntegrationTest;
import edu.tufts.hrilab.test.framework.compare.FetchPerformanceTestComparator;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.concurrent.TimeUnit;

public class FetchPerformanceAssessmentDemoTest extends GenerativeDiarcIntegrationTest {
  private FetchPerformanceAssessmentDemoMock diarcConfig;
  private SimSpeechRecognitionComponent simSpeechRec;
  private GoalManagerImpl gm;
  private final static int randomNumberSeed = 10;
  private Symbol agent = Factory.createSymbol("andy", "agent");

  @Before
  public void initializeDiarc() {
    setComparator(new FetchPerformanceTestComparator());
    diarcConfig = new FetchPerformanceAssessmentDemoMock();
    diarcConfig.runConfiguration();
    simSpeechRec = diarcConfig.simSpeechRec;
    gm = diarcConfig.gm;
    PerformanceMeasures.setSeed(randomNumberSeed);
    addServiceToObserve("reportRecognizedSpeech", Utterance.class);
    addServiceToObserve("sayText", String.class);
  }

  @After
  public void shutdownDiarc() {
    try {
      Thread.sleep(2000);
    } catch (InterruptedException ignored) {
    }

    log.debug("[cleanup] started");
    log.debug("[shutdownConfig] tester shutdown");
    diarcConfig.shutdownConfiguration();
    log.debug("[shutdownConfig] completed");
    log.info("[cleanup] ended");
  }

  public void sendUserInput(String input) {
    tester.markNewInput();
    simSpeechRec.setText(input);
    evaluateResults();
  }

  @Test
  //@Ignore
  public void beforeNoAssessmentModification() {
    setState();
    testPerfAssessment("what is the probability that you can assemble the caddy");
  }

  @Test
  //@Ignore
  public void beforeCompleteStep() {
    setState();
    testPerfAssessment("what is the probability that you can assemble the caddy if you successfully fetch the small gear from the small gear location");
  }

  @Test
  //@Ignore
  public void beforeModifyStep() {
    setState();
    testPerfAssessment("what is the probability that you can assemble the caddy if you do not fetch the small gear from the small gear location");
  }
  //@Test
  //public void duringNoAssessmentModification() {
  //  partiallyExecuteAssembleTask();
  //  testPerfAssessment("what is the probability that you can complete the task to assemble the caddy");
  //}

  //@Test
  //public void duringCompleteStep() {
  //  partiallyExecuteAssembleTask();
  //  testPerfAssessment("what is the probability that you can complete the task to assemble the caddy "
  //      + "if you successfully fetch the large gear from the large gear location");
  //}

  //@Test
  //public void duringModifyStep() {
  //  partiallyExecuteAssembleTask();
  //  testPerfAssessment("what is the probability that you can complete the task to assemble the caddy "
  //      + "if you do not fetch the large gear from the large gear location");

  //}

  private void testPerfAssessment(String inputString) {
    setSingleTestTimeout(40, TimeUnit.SECONDS);
    sendUserInput(inputString);

    setSingleTestTimeout(20, TimeUnit.SECONDS);
    sendUserInput("how long will it take to do that");

    setSingleTestTimeout(20, TimeUnit.SECONDS);
    sendUserInput("where do you foresee issues");
  }

  private void partiallyExecuteAssembleTask() {
    setState();

    // execute assemble task
    Predicate goal = Factory.createPredicate("assemble", agent, Factory.createSymbol("object_0"));
    gm.submitGoal(goal);

    // wait 5 seconds to partially execute task
    try {
      Thread.sleep(5000);
    } catch (InterruptedException e) {
    }

    // suspend assemble task
    long goalId = gm.submitGoal(Factory.createPredicate("suspendGoal", agent, goal));
    gm.joinOnGoal(goalId);
  }

  private void setState() {
    // TODO: the submitGoal version causes a race condition -- figure out why
//    Predicate goal = Factory.createPredicate("setState", agent, Factory.createPredicate("at(andy:agent,location_0:location)"));
//    long goalId = gm.submitGoal(goal);
//    gm.joinOnGoal(goalId);

    gm.setState(Factory.createPredicate("at(andy:agent,location_0:location)"));
  }
}
