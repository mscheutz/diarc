/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.demos.llm;

import edu.tufts.hrilab.config.llm.PickAndPlaceLLMDemo;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.test.framework.GenerativeDiarcIntegrationTest;
import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

public class PickAndPlaceLLMDemoTest extends GenerativeDiarcIntegrationTest {
    private PickAndPlaceLLMDemo diarcConfig;
    private SimSpeechRecognitionComponent simSpeechRec;

    @Before
    public void initializeDiarc() {
        diarcConfig = new PickAndPlaceLLMDemo();
        diarcConfig.runConfiguration();
        simSpeechRec = diarcConfig.simSpeechRec;

        addServiceToObserve("sayText", String.class);
        addServiceToObserve("assertProperties", Map.class, Double.class, List.class);
        addServiceToObserve("assertProperties", Symbol.class, List.class);
        addServiceToObserve("openGripperRapid");
        addServiceToObserve("closeGripperRapid");
        addServiceToObserve("pickupItem", Symbol.class);
        addServiceToObserve("putDownItem", Symbol.class, Symbol.class);
        addServiceToObserve("goToCameraPose", Symbol.class);
        addServiceToObserve("sayText", String.class, Boolean.class);
        addServiceToObserve("perceiveEntityFromSymbol", Symbol.class);
        addServiceToObserve("waitForResponse", Predicate.class);
        addServiceToObserve("addDetectionType", Symbol.class, Symbol.class);
        addServiceToObserve("addItem", Symbol.class);

        setPermanentTimeout(5, TimeUnit.SECONDS);

    }

    @After
    public void shutdownDiarc() {
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
    public void PickAndPlaceDemoTestEnglish() {

        setSingleTestTimeout(15, TimeUnit.SECONDS);
        addUserInput("Get the corn to the hotplate");
        evaluateResults();

        addUserInput("Get a chicken to the cooktop");
        evaluateResults();

        addUserInput("I will teach you a new ingredient named chicken");
        evaluateResults();

        addUserInput("the pantry");
        evaluateResults();

        addUserInput("the chicken job");
        evaluateResults();

        addUserInput("the prep area");
        evaluateResults();

        addUserInput("hold it here");
        evaluateResults();

        addUserInput("get a chicken to the cooktop");
        evaluateResults();

        addUserInput("I got the chicken for you");
        evaluateResults();

        addUserInput("Get the chicken to the preparation area");
        evaluateResults();

        addUserInput("Right arm go to the hotplate");
        evaluateResults();

        addUserInput("Right arm pick up the corn");
        evaluateResults();

        addUserInput("Right arm go to the preparation area");
        evaluateResults();

        addUserInput("Right arm put down the corn");
        evaluateResults();

        addUserInput("Get the corn in the serving box");
        evaluateResults();

        addUserInput("Get the chicken in the serving box");
        evaluateResults();
    }

    @Test
    @Ignore
    public void PickAndPlaceDemoTestJapanese() {

        setSingleTestTimeout(15, TimeUnit.SECONDS);
        addUserInput("トウモロコシをホットプレートに持ってきます");
        evaluateResults();

        addUserInput("ニワトリをクックトップに持ってきます");
        evaluateResults();

        addUserInput("チキンと言う新しい食材について教えます");
        evaluateResults();

        addUserInput("パントリー");
        evaluateResults();

        addUserInput("ジョブチキン");
        evaluateResults();

        addUserInput("準備エリア");
        evaluateResults();

        addUserInput("ここで握って\u200B\u200Bください");
        evaluateResults();

        addUserInput("ニワトリをクックトップに持ってきます");
        evaluateResults();

        addUserInput("あなたのために鶏肉を持ってきました");
        evaluateResults();

        addUserInput("チキンを準備エリアに運びます");
        evaluateResults();

        addUserInput("右腕、ホットプレートに行く");
        evaluateResults();

        addUserInput("右腕、トウモロコシを拾う");
        evaluateResults();

        addUserInput("右腕, 準備エリアに行く");
        evaluateResults();

        addUserInput("右腕、トウモロコシを下ろしてください");
        evaluateResults();

        addUserInput("トウモロコシをサービングボックスに入れる");
        evaluateResults();

        addUserInput("鶏肉をサービングボックスに入れます");
        evaluateResults();
    }

    //This wrapper exists so that the generator can appropriately catch input
    private void addUserInput(String input) {
        tester.markNewInput();
        simSpeechRec.setText(input);
    }
}
