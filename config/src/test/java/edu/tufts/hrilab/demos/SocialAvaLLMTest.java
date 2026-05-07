/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */

package edu.tufts.hrilab.demos;

import edu.tufts.hrilab.config.tr.SocialAvaConfig;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.test.framework.GenerativeDiarcIntegrationTest;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.concurrent.TimeUnit;

public class SocialAvaLLMTest extends GenerativeDiarcIntegrationTest {
  private SocialAvaConfig diarcConfig;
  private SimSpeechRecognitionComponent simSpeechRec;
  private int timeoutDuration = 15;

  @Before
  public void initializeDiarc() {
    diarcConfig = new SocialAvaConfig();
    diarcConfig.useMockAva = true;
    diarcConfig.useGUI = true;
    diarcConfig.llmOnly = true;
    diarcConfig.persistKiosk = false;
    diarcConfig.initKiosk = false;
    diarcConfig.runConfiguration();
    simSpeechRec = diarcConfig.simSpeechRec;
    tester.setServiceProvider(diarcConfig.serviceProvider);
    tester.setSpeaker(Factory.createSymbol("webapp"));
    tester.setAgentTeam(Factory.createSymbol("ava"));

    setPermanentTimeout(timeoutDuration, TimeUnit.SECONDS);
  }

  @After
  public void shutdownDiarc() {
    //Give last step a chance to properly complete
    try {
      Thread.sleep(5000);
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
  }

  @Test
  public void oneShotScheduleTest() {
    addUserInput("I want you to play the test video every 5 minutes starting in 10 minutes and repeating indefinitely");
    evaluateResults();

    //Currently asks an additional question which isn't desired (about end time)

    addUserInput("yes");
    evaluateResults();

    addUserInput("yes");
    evaluateResults();
  }

  @Test
  public void systemLedScheduleTest() {
    addUserInput("I want to schedule a new task");
    evaluateResults();

    addUserInput("yes");
    evaluateResults();

    addUserInput("the task is to say hi this is a new task");
    evaluateResults();

    addUserInput("perform it first in 15 minutes");
    evaluateResults();

    addUserInput("i want it to repeat every hour");
    evaluateResults();

    addUserInput("stop in 3 hours");
    evaluateResults();

    addUserInput("yes");
    evaluateResults();

    addUserInput("cancel all scheduled tasks");
    evaluateResults();

    addUserInput("yes");
    evaluateResults();
  }

  @Test
  public void partialScheduledTest() {
    addUserInput("I want to schedule the task to play the test video");
    evaluateResults();

    addUserInput("yes");
    evaluateResults();

    //Not handling well
    //addUserInput("It should occur in 3 hours. It should not repeat");
    //evaluateResults();

    addUserInput("It should occur in 3 hours");
    evaluateResults();

    addUserInput("It should not repeat");
    evaluateResults();

    addUserInput("yes");
    evaluateResults();
  }

  @Test
  public void negativeValidationScheduledTest() {
    //addUserInput("I want you to measure the temperature at reception every hour starting now until I tell you to stop");
    addUserInput("I want you to measure the temperature at reception every hour starting now indefinitely");
    evaluateResults();

    addUserInput("yes");
    evaluateResults();

    //(Having more inconsistent results on this test, sometimes submits immediately, sometimes misses end timing detail)
    //Obviously don't want this step
    addUserInput("I already told you there is no end time");
    evaluateResults();

    addUserInput("no, actually can you do the measuring every two hours");
    evaluateResults();

    //Ideally direct submission validation rather than llm text response validation
    addUserInput("that sounds good");
    evaluateResults();

    addUserInput("yes");
    evaluateResults();
  }

  @Test
  public void oneShotTourTeachingTest() {
    tester.setTimeoutDuration(20, TimeUnit.SECONDS);

    addUserInput("Learn a new task called measurement patrol made up of four steps. Start at the dock and say starting the measurement patrol. Steps 2 and 3 should be to measure wifi signal and temperature at reception and at the pavilion. Step 4 is to say “done measuring” back at dock");
    evaluateResults();

    addUserInput("yes");
    evaluateResults();

    addUserInput("yes");
    evaluateResults();

    addUserInput("What are the steps of the measurement patrol I taught you?");
    evaluateResults();

    addUserInput("forget how to perform the measurement patrol");
    evaluateResults();

    addUserInput("yes");
    evaluateResults();

    tester.setTimeoutDuration(timeoutDuration, TimeUnit.SECONDS);
  }

  @Test
  public void systemLedTourTeachingTest() {
    addUserInput("i want you to learn a new task sequence");
    evaluateResults();

    addUserInput("yes");
    evaluateResults();

    addUserInput("the task should be called measurement patrol");
    evaluateResults();

    addUserInput("the path of locations should be reception, pavilion, elevators, and then the dock");
    evaluateResults();

    addUserInput("at the first three locations you should measure the temperature and the wifi");
    evaluateResults();

    //TODO: get more natural phrasing working
    addUserInput("back at the dock you should report the wifi and the temperature of the other three locations");
//    addUserInput("back at the dock you should report the wifi of the other three locations and then report the temperature of the other three locations");
    evaluateResults();

    tester.setTimeoutDuration(20, TimeUnit.SECONDS);

    addUserInput("that is all");
    evaluateResults();

    addUserInput("yes");
    evaluateResults();

    addUserInput("forget how to perform the measurement patrol");
    evaluateResults();

    addUserInput("yes");
    evaluateResults();

    tester.setTimeoutDuration(timeoutDuration, TimeUnit.SECONDS);

  }

  @Test
  public void partialTourTeachingTestTest() {
    addUserInput("I want you to learn a new task sequence called the measurement patrol where you measure both the wifi signal and the temperature at all specified locations");
    evaluateResults();

    addUserInput("yes");
    evaluateResults();

    addUserInput("the specified locations are reception, pavilion, and the elevators");
    evaluateResults();

    addUserInput("that is the whole task");
    evaluateResults();

    addUserInput("yes");
    evaluateResults();

    addUserInput("forget how to perform the measurement patrol");
    evaluateResults();

    addUserInput("yes");
    evaluateResults();
  }


  @Test
  public void negativeValidationTourTeachingTest() {
    tester.setTimeoutDuration(20,TimeUnit.SECONDS);

    addUserInput("Learn a new task called measurement patrol made up of four steps. Start at the dock and say starting the measurement patrol. Steps 2 and 3 should be to measure wifi signal at reception and at the pavilion. Step 4 is to say “done measuring” back at dock");
    evaluateResults();

    addUserInput("yes");
    evaluateResults();

    addUserInput("No, can you actually add a step at the elevator between the reception and the pavilion where you say this step was added");
    evaluateResults();

    addUserInput("please save that task sequence");
    evaluateResults();

    addUserInput("yes");
    evaluateResults();

    addUserInput("forget how to perform the measurement patrol");
    evaluateResults();

    addUserInput("yes");
    evaluateResults();

    tester.setTimeoutDuration(timeoutDuration,TimeUnit.SECONDS);

  }

  @Test
  public void currentLocationITKTest() {
    addUserInput("where are you right now?");
    evaluateResults();

    addUserInput("could you get to reception");
    evaluateResults();

    addUserInput("please go there");
    evaluateResults();

    addUserInput("where are you now?");
    evaluateResults();
  }

  @Test
  public void savedLocationITKTest() {
    addUserInput("what locations do you know about?");
    evaluateResults();

    addUserInput("can you tell me more?");
    evaluateResults();
  }

  @Test
  public void measurementsITKTest() {
    addUserInput("do you know the wifi signal strength at reception?");
    evaluateResults();

    addUserInput("could you go figure that out for me?");
    evaluateResults();

    addUserInput("yes");
    evaluateResults();

    addUserInput("What is the wifi signal strength at reception?");
    evaluateResults();

    addUserInput("What is the wifi signal strength where you are right now?");
    evaluateResults();

    addUserInput("get and tell me the temperature of where you are");
    evaluateResults();
  }

  @Test
  public void goalITKTest() {
    addUserInput("Let me schedule something with you");
    evaluateResults();

    addUserInput("yes");
    evaluateResults();

    addUserInput("hold on I need a second, can we pause scheduling");
    evaluateResults();

    addUserInput("yes");
    evaluateResults();

    addUserInput("go to reception");
    evaluateResults();

    addUserInput("yes");
    evaluateResults();

    addUserInput("escort eric from reception to cardiology check-in");
    evaluateResults();

    addUserInput("yes");
    evaluateResults();

    addUserInput("what are you doing right now?");
    evaluateResults();

    addUserInput("after we finish that suspended task of scheduling a future task, what will you do next?");
    evaluateResults();

    addUserInput("and after that?");
    evaluateResults();

    addUserInput("and then?");
    evaluateResults();

    addUserInput("what are your scheduled tasks?");
    evaluateResults();

    addUserInput("how do i resume scheduling our task?");
    evaluateResults();
  }

  @Test
  public void invalidArgumentTest() {
    addUserInput("Go to the nurse's office");
    evaluateResults();
  }

  @Test
  public void teachActionInfoITKTest() {
    addUserInput("Can I teach you to do new things?");
    evaluateResults();

    addUserInput("What information would I need to prepare to teach you something?");
    evaluateResults();

    addUserInput("Okay let me try it out");
    evaluateResults();
  }

  //@Test
  //public void Test() {
  //  addUserInput("");
  //  evaluateResults();
  //}

  //This wrapper exists so that the generator can appropriately catch input
  private void addUserInput(String input) {
    tester.markNewInput();
    simSpeechRec.setText(input);
  }
}
