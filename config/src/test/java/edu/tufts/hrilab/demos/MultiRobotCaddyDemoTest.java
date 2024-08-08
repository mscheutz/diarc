/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.demos;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import edu.tufts.hrilab.action.db.performanceMeasures.PerformanceMeasures;
import edu.tufts.hrilab.action.execution.ExecutionType;
import edu.tufts.hrilab.config.MultiRobotCaddyDemoMock;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.common.Utterance;
import edu.tufts.hrilab.test.framework.GenerativeDiarcIntegrationTest;
import edu.tufts.hrilab.vision.MockVisionComponent;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

public class MultiRobotCaddyDemoTest extends GenerativeDiarcIntegrationTest {
    private MultiRobotCaddyDemoMock diarcConfig;
    private SimSpeechRecognitionComponent simSpeechRec;
    private MockVisionComponent mockVision;
    private final static int randomNumberSeed = 10;

    @Before
    public void initializeDiarc() {
        setComparator(new edu.tufts.hrilab.test.framework.compare.FetchPerformanceTestComparator());
        diarcConfig = new MultiRobotCaddyDemoMock();
        diarcConfig.runConfiguration();
        simSpeechRec = diarcConfig.simSpeechRec;
        mockVision = diarcConfig.vision;
        PerformanceMeasures.setSeed(randomNumberSeed);
        addServiceToObserve("reportRecognizedSpeech", Utterance.class);
        addServiceToObserve("submitGoal", Predicate.class);
        addServiceToObserve("submitGoal", Predicate.class, ExecutionType.class, Symbol.class);
        //TODO:brad: is this signature used in this test?
        addServiceToObserve("sayText", String.class);
        addServiceToObserve("assertBelief", Term.class);
        addServiceToObserve("retractBelief", Term.class);
        addServiceToObserve("assertProperties", Map.class, Double.class, List.class);
        addServiceToObserve("assertProperties", Symbol.class, List.class);
        addServiceToObserve("openGripper");
        addServiceToObserve("openGripper", String.class);
        addServiceToObserve("closeGripper");
        addServiceToObserve("closeGripper", String.class);
        addServiceToObserve("approachLocation", Symbol.class);
        addServiceToObserve("approachLocation", Symbol.class, Symbol.class);
        addServiceToObserve("moveTo", String.class, Symbol.class);
        addServiceToObserve("moveToRelative", String.class, Point3d.class, Quat4d.class);
        addServiceToObserve("moveObjectAbove", Symbol.class, Symbol.class, String.class);
        addServiceToObserve("moveObjectFetchItPrimitive", Symbol.class, String.class, String.class);
        addServiceToObserve("getTypeId", List.class);
        addServiceToObserve("graspObject", String.class, Symbol.class, Float.class);
        addServiceToObserve("goToPose", String.class, Symbol.class);
        addServiceToObserve("goToPose", Symbol.class);
        addServiceToObserve("releaseObject", String.class, Symbol.class, Float.class);
        addServiceToObserve("waitForResponse", Predicate.class);
        addServiceToObserve("waitForResponse", Predicate.class, Long.class);
        addServiceToObserve("moveArmToCarryPosition");
        addServiceToObserve("moveArmOverBody");
        addServiceToObserve("moveArmOverTable");
        addServiceToObserve("goToLocation", Symbol.class);
        addServiceToObserve("goToLoc", Symbol.class, Boolean.class);
        addServiceToObserve("stowArm");
        addServiceToObserve("unStowArm");
        addServiceToObserve("moveTo", String.class, Symbol.class);
        addServiceToObserve("moveObject", Symbol.class, String.class, String.class);
        addServiceToObserve("getTypeId", Symbol.class);
        addServiceToObserve("getTokenIds", Symbol.class);
        addServiceToObserve("releaseObject", String.class, Symbol.class, Float.class);
        addServiceToObserve("detectAndOpenDoor");
        addServiceToObserve("initLocation", Symbol.class);

        PerformanceMeasures.setSeed(randomNumberSeed);
    }

    @After
    public void shutdownDiarc() {
        log.debug("[shutdownDiarc] started");
        diarcConfig.shutdownConfiguration();
        log.debug("[shutdownDiarc] completed");
    }

    public void sendUserInput(String input) {
        tester.markNewInput();
        simSpeechRec.setText(input);
        evaluateResults();
    }

    @Test
    public void multiRobotCaddyDemoTest() {
        setSingleTestTimeout(10, TimeUnit.SECONDS);
        addUserInput("setup demo");
        evaluateResults();

        addUserInput("define new kit medkit");
        evaluateResults();

        setSingleTestTimeout(5, TimeUnit.SECONDS);
        addUserInput("it uses a medical caddy");
        evaluateResults();

        addUserInput("there is 1 medical caddy at table a");
        evaluateResults();

        addUserInput("it contains 2 painkillers");
        evaluateResults();

        addUserInput("there are 2 painkillers at table b");
        evaluateResults();

        addUserInput("it contains one antiseptic");
        evaluateResults();

        addUserInput("there is 1 antiseptic at table c");
        evaluateResults();

        addUserInput("it contains one bandage box");
        evaluateResults();

        addUserInput("there is 1 bandage box at table e");
        evaluateResults();

        addUserInput("that is all");
        evaluateResults();

        setSingleTestTimeout(60, TimeUnit.SECONDS);
        addUserInput("deliver a medkit to alpha");
        evaluateResults();

        setSingleTestTimeout(15, TimeUnit.SECONDS);
        addUserInput("describe your plan");
        evaluateResults();

        // need to insert the bandagebox object into the mock vision scene, after the recovery
        // action for the spot to get the bandagebox for the fetch
        mockVision.setSceneIndex(1);

        setSingleTestTimeout(10, TimeUnit.SECONDS);
        addUserInput("yes");
        evaluateResults();

        setSingleTestTimeout(60, TimeUnit.SECONDS);
        addUserInput("ready");
        evaluateResults();
    }

    //This wrapper exists so that the generator can appropriately catch input
    private void addUserInput(String input) {
        tester.markNewInput();
        simSpeechRec.setText(input);
    }
}
