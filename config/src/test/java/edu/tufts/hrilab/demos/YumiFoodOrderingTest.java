/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.demos;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import edu.tufts.hrilab.config.YumiFoodOrderingMock;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.test.framework.GenerativeDiarcIntegrationTest;
import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

public class YumiFoodOrderingTest extends GenerativeDiarcIntegrationTest {
  private YumiFoodOrderingMock diarcConfig;
  private SimSpeechRecognitionComponent simSpeechRec;
  private SimSpeechRecognitionComponent untrustedSimSpeechRec;

  @Before
  public void initializeDiarc() {
    diarcConfig = new YumiFoodOrderingMock(true);
    diarcConfig.runConfiguration();
    simSpeechRec = diarcConfig.simSpeechRec;
    untrustedSimSpeechRec = diarcConfig.untrustedSimSpeechRec;

    addServiceToObserve("sayText", String.class);
    addServiceToObserve("assertProperties", Map.class, Double.class, List.class);
    addServiceToObserve("assertProperties", Symbol.class, List.class);
    addServiceToObserve("openGripperRapid");
    addServiceToObserve("closeGripperRapid");
    addServiceToObserve("pourSauce");
    addServiceToObserve("sayText", String.class, Boolean.class);
    addServiceToObserve("perceiveEntityFromSymbol", Symbol.class);
    addServiceToObserve("pickupItem", Symbol.class);
    addServiceToObserve("putDownItem", Symbol.class, Symbol.class);
    addServiceToObserve("cookItem", Symbol.class, Term.class);
    addServiceToObserve("sauteItem", Symbol.class, Term.class);
    addServiceToObserve("goToCameraPose", Symbol.class);
    addServiceToObserve("defineGraspPointForDescriptor", Symbol.class, Symbol.class);
    addServiceToObserve("checkError", Term.class);

    setPermanentTimeout(5, TimeUnit.SECONDS);

  }

  @After
  public void shutdownDiarc() {
    log.debug("[cleanup] started");
    log.debug("[shutdownConfig] tester shutdown");
    diarcConfig.shutdownConfiguration();
    log.debug("[shutdownConfig] completed");

    try {
      TRADE.reset("");
    } catch (TRADEException e) {
      log.error("[shutdownConfig]", e);
    }
    log.info("[cleanup] ended");
  }

  public void sendUserInput(String input) {
    tester.markNewInput();
    simSpeechRec.setText(input);
  }

  public void sendUntrustedUserInput(String input) {
    tester.markNewInput();
    untrustedSimSpeechRec.setText(input);
  }

  //demo script
  @Test
  public void yumiFoodOrderingTest() {

    setSingleTestTimeout(10, TimeUnit.SECONDS);
    addUserInput("init");
    evaluateResults();

    setSingleTestTimeout(5, TimeUnit.SECONDS);
    addUserInput("right arm go to prep area");
    evaluateResults();

    diarcConfig.rightArm.setCameraResultDetected(false);
    addUserInput("right arm do you see a bell pepper");
    evaluateResults();

    diarcConfig.rightArm.setCameraResultDetected(true);
    addUserInput("right arm do you see a bell pepper");
    evaluateResults();

    addUserInput("left arm go to box area");
    evaluateResults();

    addUserInput("left arm do you see a serving box");
    evaluateResults();

    addUserInput("right arm pick up the bell pepper");
    evaluateResults();

    addUserInput("get the bell pepper to the prep area");
    evaluateResults();

    diarcConfig.rightArm.setErrorState(true);
    addUserInput("right arm go to prep area");
    evaluateResults();

    diarcConfig.rightArm.setErrorState(false);
    addUserInput("now right arm go to prep area");
    evaluateResults();

    addUserInput("it is safe to proceed");
    evaluateResults();

    addUserInput("reset");
    evaluateResults();

    setSingleTestTimeout(2, TimeUnit.SECONDS);
    addUserInput("define new item southwest bowl");
    evaluateResults();
    addUserInput("first get a serving box to serving area");
    evaluateResults();
    addUserInput("then get a bell pepper to the hot plate");
    evaluateResults();
    addUserInput("then right arm saute the bell pepper for 2 seconds");
    evaluateResults();
    addUserInput("then get the bell pepper in the serving box");
    evaluateResults();
    addUserInput("then get a corn to the cooktop");
    evaluateResults();
    addUserInput("then right arm cook the corn for 5 seconds");
    evaluateResults();
    addUserInput("then get the corn in the serving box");
    evaluateResults();
    addUserInput("then get a carrot to the hot plate");
    evaluateResults();
    addUserInput("then right arm saute the carrot for 3 seconds");
    evaluateResults();
    addUserInput("then get the carrot in the serving box");
    evaluateResults();
    addUserInput("then left arm drizzle on chipotle sauce");
    evaluateResults();
    addUserInput("that is how you prepare a southwest bowl");
    evaluateResults();

    setSingleTestTimeout(25, TimeUnit.SECONDS);
    addUserInput("prepare a southwest bowl");
    evaluateResults();

    addUserInput("here it is");
    evaluateResults();

    addUserInput("here it is");
    evaluateResults();

    addUserInput("here it is");
    evaluateResults();

    setSingleTestTimeout(6, TimeUnit.SECONDS);
    addUserInput("define new item by analogy puerto rican bowl");
    evaluateResults();

    addUserInput("southwest bowl");
    evaluateResults();

    addUserInput("replace right arm saute the carrot for 3 seconds with right arm saute the chicken for 4 seconds");
    evaluateResults();

    addUserInput("remove the bell pepper");
    evaluateResults();

    addUserInput("add get a plantain in the serving box after get the chicken in the serving box");
    evaluateResults();

    addUserInput("suspend current task");
    evaluateResults();

    addUserInput("now define new ingredient plantain");
    evaluateResults();

    addUserInput("pantry");
    evaluateResults();

    addUserInput("detect plantain");
    evaluateResults();

    addUserInput("prep area");
    evaluateResults();

    setSingleTestTimeout(10, TimeUnit.SECONDS);
    addUserInput("grasp it here");
    evaluateResults();

    setSingleTestTimeout(3, TimeUnit.SECONDS);
    addUserInput("what is your current task");
    evaluateResults();

    addUserInput("resume previous task");
    evaluateResults();

    addUserInput("add get a plantain in the serving box after get the chicken in the serving box");
    evaluateResults();

    addUserInput("no more differences");
    evaluateResults();

    setSingleTestTimeout(25, TimeUnit.SECONDS);
    addUserInput("prepare a puerto rican bowl");
    evaluateResults();

    addUserInput("here it is");
    evaluateResults();

    addUserInput("here it is");
    evaluateResults();

    addUserInput("here it is");
    evaluateResults();

    setSingleTestTimeout(5, TimeUnit.SECONDS);
    sendUntrustedUserInput("right arm go to prep area");
    evaluateResults();

    addUserInput("add new supervisor front");
    evaluateResults();

    sendUntrustedUserInput("right arm go to prep area");
    evaluateResults();

    sendUntrustedUserInput("define new item puerto rican bowl");
    evaluateResults();

    addUserInput("add new admin front");
    evaluateResults();

    sendUntrustedUserInput("define new item puerto rican bowl");
    evaluateResults();
  }

  //Off scripts tests for robustness and/or desired functionality
  //TODO: Need to create methods to do and then actually correctly test that defineItem, defineItemByAnalogy, and defineIngredient
  //  (and all internal methods with side effects - positReference, addDetectionType, defineIngredientHelper, ...) can be
  //  properly interrupted and/or canceled while having all side effects undone. Currently this is not the case and it is
  //  not safe to arbitrarily perform off-script behaviors during/after interruptions.
  @Test
  public void questionAskingInterruptionTest() {

    setSingleTestTimeout(10, TimeUnit.SECONDS);
    addUserInput("init");
    evaluateResults();

    setSingleTestTimeout(5, TimeUnit.SECONDS);
    addUserInput("define new item southwest bowl");
    evaluateResults();
    addUserInput("suspend current task");
    evaluateResults();
    addUserInput("now say hello");
    evaluateResults();
    addUserInput("what is your current task");
    evaluateResults();
    addUserInput("resume previous task");
    evaluateResults();

    setSingleTestTimeout(2, TimeUnit.SECONDS);
    addUserInput("first get a serving box to serving area");
    evaluateResults();
    addUserInput("then get a bell pepper to the hot plate");
    evaluateResults();
    addUserInput("then right arm saute the bell pepper for 2 seconds");
    evaluateResults();
    addUserInput("then get the bell pepper in the serving box");
    evaluateResults();
    addUserInput("then get a corn to the cooktop");
    evaluateResults();
    addUserInput("then right arm cook the corn for 5 seconds");
    evaluateResults();
    addUserInput("then get the corn in the serving box");
    evaluateResults();
    addUserInput("then get a carrot to the hot plate");
    evaluateResults();
    addUserInput("then right arm saute the carrot for 3 seconds");
    evaluateResults();
    addUserInput("then get the carrot in the serving box");
    evaluateResults();
    addUserInput("then left arm drizzle on chipotle sauce");
    evaluateResults();
    addUserInput("that is how you prepare a southwest bowl");
    evaluateResults();

    setSingleTestTimeout(25, TimeUnit.SECONDS);
    addUserInput("prepare a southwest bowl");
    evaluateResults();

    addUserInput("here it is");
    evaluateResults();

    addUserInput("here it is");
    evaluateResults();

    addUserInput("here it is");
    evaluateResults();
  }

  //TODO: Other tests that would be required if the underlying functionality was actually implemented (which would be
  // required if this domain is to be safely used for off script interactions or further dev on these fronts)
  //Interruption of all the other teaching scripts and during different points within these script
  //(If extended to actually fully reset domain) thorough reset tests
  //Interruption and/or failure during all primitives and planner actions

  //This wrapper exists so that the generator can appropriately catch input
  private void addUserInput(String input) {
    tester.markNewInput();
    simSpeechRec.setText(input);
  }
}
