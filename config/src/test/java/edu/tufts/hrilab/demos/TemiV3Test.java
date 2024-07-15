/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.demos;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.config.TemiV3Mock;
import edu.tufts.hrilab.temiv3.TemiV3PoseReference;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.test.framework.HardCodedDiarcIntegrationTest;
import edu.tufts.hrilab.test.framework.compare.HardCodedTestComparator;
import edu.tufts.hrilab.test.framework.tester.DiarcConfigTester;
import org.apache.commons.lang3.tuple.Pair;
import org.junit.*;

import java.util.*;
import java.util.concurrent.TimeUnit;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import static org.junit.Assert.*;

public class TemiV3Test extends HardCodedDiarcIntegrationTest {

  static DiarcConfigTester tester;
  TemiV3Mock config;
  private final static Pattern alphaNumericPattern = Pattern.compile("\\w+");
  private Map<Variable, Symbol> ackResponseBindings = new HashMap();
  private Map<Variable, Symbol> freezeResponseBindings = new HashMap();

  //components that we care about interacting with in tests
  SimSpeechRecognitionComponent speechInputBrad;

  @Override
  public void initializeDiarc() {
    log.info("[initialize]");
    //create and register tester
    tester = new DiarcConfigTester();
    tester.setTimeoutDuration(20, TimeUnit.SECONDS);
    tester.registerWithTRADE();

    //instantiate DIARCInstance
    config = new TemiV3Mock();
    config.runConfiguration();
    speechInputBrad = config.simspeech;
    ackResponseBindings.put(Factory.createVariable("X"), Factory.createSymbol("ack"));
    freezeResponseBindings.put(Factory.createVariable("X"), Factory.createVariable("X"));


    //todo: use a similar/same abstraction as to the one used in generative tests
    List<Pair<String, Class<?>[]>> servicesToObserve = new ArrayList<>();
    servicesToObserve.add(Pair.of("assertBelief", new Class<?>[]{Term.class}));
    servicesToObserve.add(Pair.of("retractBelief", new Class<?>[]{Term.class}));
    servicesToObserve.add(Pair.of("saveLocation", new Class<?>[]{Symbol.class}));
    servicesToObserve.add(Pair.of("deleteLocation", new Class<?>[]{Symbol.class}));
    servicesToObserve.add(Pair.of("goToLoc", new Class<?>[]{Symbol.class, Boolean.class}));
    servicesToObserve.add(Pair.of("goToPos", new Class<?>[]{List.class, Symbol.class, Boolean.class}));
    servicesToObserve.add(Pair.of("knowsLocation", new Class<?>[]{Symbol.class}));
    servicesToObserve.add(Pair.of("sayText", new Class<?>[]{String.class}));
    servicesToObserve.add(Pair.of("waitForResponse", new Class<?>[]{Predicate.class}));
    servicesToObserve.add(Pair.of("waitForResponse", new Class<?>[]{Predicate.class, Long.class}));
    servicesToObserve.add(Pair.of("cancelWaitForResponse", new Class<?>[]{Predicate.class}));
    servicesToObserve.add(Pair.of("startAcknowledgeDisplay", new Class<?>[]{String.class}));
    servicesToObserve.add(Pair.of("endAcknowledgeDisplay", new Class<?>[]{}));
    servicesToObserve.add(Pair.of("stopMoving", new Class<?>[]{}));
    servicesToObserve.add(Pair.of("freezeTemi", new Class<?>[]{}));
    servicesToObserve.add(Pair.of("endFreezeTemi", new Class<?>[]{}));
    servicesToObserve.add(Pair.of("followMeBlocking", new Class<?>[]{}));
    servicesToObserve.add(Pair.of("stopMoving", new Class<?>[]{}));
    servicesToObserve.add(Pair.of("endRepositionLoop", new Class<?>[]{}));
    servicesToObserve.add(Pair.of("startQRCodeDisplay", new Class<?>[]{String.class, String.class}));
    servicesToObserve.add(Pair.of("endQRCodeDisplay", new Class<?>[]{}));
    servicesToObserve.add(Pair.of("playVideo", new Class<?>[]{Symbol.class}));
    servicesToObserve.add(Pair.of("setEscortWaitAttempts", new Class<?>[]{Symbol.class}));
    servicesToObserve.add(Pair.of("setEscortWaitDuration", new Class<?>[]{Symbol.class}));
    servicesToObserve.add(Pair.of("interruptGoToLocation", new Class<?>[]{}));
    servicesToObserve.add(Pair.of("interruptFollowBlocking", new Class<?>[]{}));
    servicesToObserve.add(Pair.of("setSayTextBlocks", new Class<?>[]{Boolean.class}));

    for (Pair<String, Class<?>[]> service : servicesToObserve) {
      try {
        TRADEServiceInfo wrapperTSI = TRADE.getAvailableService(new TRADEServiceConstraints().name("addObservedOutput").argTypes(Object[].class));
        TRADEServiceInfo TSItoWrap = TRADE.getAvailableService(new TRADEServiceConstraints().name(service.getKey()).argTypes(service.getValue()));
        TRADE.afterService(wrapperTSI, TSItoWrap);
      } catch (TRADEException e) {
        log.error("exception adding after wrapper for: " + service.getKey(), e);
      }
    }

    try {
      TRADEServiceInfo wrapperTSI = TRADE.getAvailableService(new TRADEServiceConstraints().name("addObservedOutput").argTypes(Object[].class));
      TRADEServiceInfo TSItoWrap = TRADE.getAvailableService(new TRADEServiceConstraints().name("waitForResponse").argTypes(Predicate.class));
      TRADE.beforeService(wrapperTSI, TSItoWrap);
      TSItoWrap = TRADE.getAvailableService(new TRADEServiceConstraints().name("waitForResponse").argTypes(Predicate.class, Long.class));
      TRADE.beforeService(wrapperTSI, TSItoWrap);
    } catch (TRADEException e) {
      log.error("exception adding before wrapper for waitForResponse", e);
    }

  }

  @Override
  public void shutdownDiarc() {
    //Give last step a chance to properly complete
    try {
      Thread.sleep(2000);
    } catch (InterruptedException ignored) {
    }

    log.debug("[cleanup] started");
    tester.shutdown();
    tester = null;
    log.debug("[shutdownConfig] tester shutdown");
    config.shutdownConfiguration();
    log.debug("[shutdownConfig] completed");
    log.info("[cleanup] ended");
  }

  private Long getGoalId(String goal) {
    Predicate goalPred = Factory.createPredicate(goal);
    Long goalID = null;
    try {
      List<Long> goalIDs = TRADE.getAvailableService(new TRADEServiceConstraints().name("getGoalIds").argTypes(Predicate.class)).call(List.class, goalPred);
      if (goalIDs != null && goalIDs.size() > 0) {
        goalID = goalIDs.get(0);
      }
    } catch (TRADEException e) {
      log.error("Error trying to get goal ID for goal: " + goal);
    }
    assertNotNull(goalID);
    log.debug("[getGoalId] got goalID " + goalID);
    return goalID;
  }

  private Symbol getLocRef(String locName) {
    log.info("[getLocRef] calling temi.getLocationReferenceFromName for loc: " + locName);
    TemiV3PoseReference ref = config.temi.getLocationReferenceFromName(locName);
    if (ref != null) {
      log.info("[getLocRef] " + locName + " " + ref.refId + " " + ref.getName());
      return ref.refId;
    } else {
      log.error("could not find locRef for name: " + locName);
      return null;
    }
  }

  //TODO:brad: this could maybe be put in a utility method somewhere for convenience sake.

  /**
   * Convenience method for wrapping names that should variably be in ""
   *
   * @param desc string to be wrapped
   * @return wrapped string
   */
  private static String escapeSpecialCharacters(String desc) {
    if (desc == null || desc.isEmpty()) {
      return null;
    }
    Matcher matcher = alphaNumericPattern.matcher(desc);
    if (!matcher.matches() || desc.contains(" ")) {
      return "\"" + desc + "\"";
    }
    return desc;
  }


  /**
   * Main helper method to execute speech input, and test the expected results.
   * @param userInput
   * @param expectedOutputs
   */
  public void runUserGeneratedTest(String userInput, Object[][] expectedOutputs) {
    speechInputBrad.setText(userInput);

    HardCodedTestComparator comparator = new HardCodedTestComparator();
    comparator.setExpectedOutput(name.getMethodName(), expectedOutputs);
    tester.evaluateResults(comparator);
  }

  /**
   * Tests cases:
   * Issuing command to go to never before known location - fail in parser
   * Saving a new location - succeed
   * Going to a newly saved location - succeed
   * Deleting a known location - succeed
   * Issuing command to go to a deleted/previously known location - fail in goToLocation with notKnowWhere(location)
   * Saving same location twice - succeed - tests if consultant recognized to overwrite existing location rather
   * than create new ref
   * Going to a resaved location - succeed
   */
  @Test
  public void consultantNavigationTest() {

//    runUserGeneratedTest("hello", new Object[][]{
//            {"sayText", Factory.createSymbol("hello brad"), null},
//    });

    //TODO: don't want parens
    //Need to make sure this is a unique name
    runUserGeneratedTest("go to uniquelocation", new Object[][]{
            {"sayText", "sorry, I do Not Know What uniquelocation means", null},
    });

    runUserGeneratedTest("save this location as uniquelocation", new Object[][]{
            {"sayText", "okay", null},
            {"saveLocation", Factory.createSymbol("uniquelocation"), null}
    });

    Symbol locSymbol = getLocRef("uniquelocation");

    runUserGeneratedTest("go to uniquelocation", new Object[][]{
            {"sayText", "okay", null},
            {"knowsLocation", locSymbol, true},
            {"goToLoc", locSymbol, true, true},
    });


    runUserGeneratedTest("delete location uniquelocation", new Object[][]{
            {"sayText", "okay", null},
            {"deleteLocation", locSymbol, null},
    });

//    //This isn't how it works anymore - deleting a location from the consultant doesn't remove the ref,
//    //  just the activated entity
////        locationRefMap.remove("uniquelocation");
//
//    //Need to save an extra location in order for reference resolution in failure response to work
//    //  Issue on line 448 of Resolver.java
//    runUserGeneratedTest("save this location as extralocation", new Object[][]{
//            {"sayText", "okay", null},
//            {"saveLocation", Factory.createSymbol("extralocation"), null}
//    });
//
//
//    runUserGeneratedTest("go to uniquelocation", new Object[][]{
//            {"sayText", "okay", null},
//            {"knowsLocation", locSymbol, false},
//            //can we get this better in demosgen.prag?
//            {"sayText", Factory.createSymbol("I can not go To Location the uniquelocation true because I know not Where the uniquelocation"), null},
//    });

    runUserGeneratedTest("go to uniquelocation", new Object[][]{
            {"sayText", "sorry, I do Not Know What uniquelocation means", null},
    });

    runUserGeneratedTest("save this location as uniquelocation", new Object[][]{
            {"sayText", "okay", null},
            {"saveLocation", Factory.createSymbol("uniquelocation"), null},
    });

    runUserGeneratedTest("save this location as uniquelocation", new Object[][]{
            {"sayText", "okay", null},
            {"saveLocation", Factory.createSymbol("uniquelocation"), null},
    });

    locSymbol = getLocRef("uniquelocation");

    runUserGeneratedTest("go to uniquelocation", new Object[][]{
            {"sayText", "okay", null},
            {"knowsLocation", locSymbol, true},
            {"goToLoc", locSymbol, true, true},
    });
  }

  /**
   * Tests basic successful escort behavior
   */
  @Test
  public void escortSuccessTest() {

    runUserGeneratedTest(
            "save this location as test",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"saveLocation", Factory.createSymbol("test"), null}
            }
    );

    Symbol locSymbol = getLocRef("test");

    runUserGeneratedTest(
            "save this location as homebase",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"saveLocation", Factory.createSymbol("homebase"), null}
            }
    );

    runUserGeneratedTest(
            "setescortdurationto 10",
            new Object[][]{
                    {"setEscortWaitDuration", Factory.createSymbol("10"), null}
            }
    );
    long waitTime = config.temi.getEscortWaitDuration();

    runUserGeneratedTest(
            "escort the patient from test to homebase",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"knowsLocation", locSymbol, true},
                    {"goToLoc", locSymbol, false, true},
                    {"startAcknowledgeDisplay", "Hello patient. Please come with me. I will show you to homebase.", null},
                    {"sayText", "Hello patient. Please come with me. I will show you to homebase.", null},
                    {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X"), waitTime},
            }
    );

    locSymbol = getLocRef("homebase");

    int waitAttempts = config.temi.getEscortWaitAttempts();

    runUserGeneratedTest(
            "okay",
            new Object[][]{
//                    {"sayText", "okay", null},
                    {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X"), waitTime, ackResponseBindings},
                    {"endAcknowledgeDisplay", null},
                    {"knowsLocation", locSymbol, true},
                    {"goToLoc", locSymbol, true, true},
                    {"sayText", "Arrived at homebase", null},
            }
    );

  }

  /**
   * Tests basic escort behavior without receiving acknowledgement
   */
  @Test
  public void escortNoAckTest() {

    runUserGeneratedTest(
            "setescortattemptsto 2",
            new Object[][]{
                    {"setEscortWaitAttempts", Factory.createSymbol("2"), null}
            }
    );

    runUserGeneratedTest(
            "setescortdurationto 2",
            new Object[][]{
                    {"setEscortWaitDuration", Factory.createSymbol("2"), null}
            }
    );

    runUserGeneratedTest(
            "save this location as test",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"saveLocation", Factory.createSymbol("test"), null}
            }
    );

    Symbol locSymbol = getLocRef("test");
    assertNotNull(locSymbol);

    runUserGeneratedTest(
            "save this location as homebase",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"saveLocation", Factory.createSymbol("homebase"), null}
            }
    );


    Symbol homebaseLocSymbol = getLocRef("homebase");
    assertNotNull(homebaseLocSymbol);

    runUserGeneratedTest(
            "escort the patient from test to homebase",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"knowsLocation", locSymbol, true},
                    {"goToLoc", locSymbol, false, true},
                    {"startAcknowledgeDisplay", "Hello patient. Please come with me. I will show you to homebase.", null},
                    {"endAcknowledgeDisplay", null},
                    {"sayText", "Hello patient. Please come with me. I will show you to homebase.", null},
                    {"sayText", "Hello patient. Please come with me. I will show you to homebase.", null},
                    {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X"), 2L},
                    {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X"), 2L},
                    {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X"), 2L, null},
                    {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X"), 2L, null},
                    {"cancelWaitForResponse", Factory.createPredicate("waitForAckTemi", "X"), null},
                    {"knowsLocation", homebaseLocSymbol, true},
                    {"goToLoc", homebaseLocSymbol, true, true},
//                    {"sayTextDialogueHistory", Factory.createSymbol("Arrived at homebase"), true, null},
                    {"sayText", "I could not find the requested patient patient", null},
            }
    );

  }

  /**
   * Tests fetch behavior
   * Failure due to storage location not being set (for command without explicit storage location)
   * Successful execution of default storage location
   * Successful execution of explicit storage location
   */
  @Test
  public void fetchTest() {

    runUserGeneratedTest(
            "save this location as storeroom",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"saveLocation", Factory.createSymbol("storeroom"), null},
            }
    );

    runUserGeneratedTest(
            "save this location as homebase",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"saveLocation", Factory.createSymbol("homebase"), null}
            }
    );

    Symbol locSymbol = getLocRef("homebase");

    runUserGeneratedTest(
            "fetch me braces",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"sayText", "I can not fetch braces because I do not know storage Location", null},
            }
    );

    locSymbol = getLocRef("storeroom");

    runUserGeneratedTest(
            "supplies are stored in the storeroom",
            new Object[][]{
//                    {"sayText", "okay", null},
                    {"retractBelief", Factory.createPredicate("storageArea", "X"), null},
                    {"assertBelief", Factory.createPredicate("storageArea", locSymbol.getName() + ":location"), null},
            }
    );

    runUserGeneratedTest(
            "fetch me braces",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"knowsLocation", locSymbol, true},
                    {"goToLoc", locSymbol, false, true},
                    {"setSayTextBlocks", false, null},
                    {"sayText", "Could I please have braces", null},
                    {"setSayTextBlocks", true, null},
                    {"startAcknowledgeDisplay", "Please place braces on my tray.", null},
                    {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X")},
            }
    );

    locSymbol = getLocRef("homebase");

    runUserGeneratedTest(
            "okay",
            new Object[][]{
                    {"endAcknowledgeDisplay", null},
                    {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X"), ackResponseBindings},
//                    {"sayText", "okay", null},
                    //TODO: do we want to modify action script for this to be a ref?
                    {"knowsLocation", Factory.createSymbol("homebase"), true},
                    {"goToLoc", Factory.createSymbol("homebase"), true, true},
                    {"sayText", "I have brought braces", null},
            }
    );

    runUserGeneratedTest(
            "fetch me braces from homebase",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"knowsLocation", locSymbol, true},
                    {"goToLoc", locSymbol, false, true},
                    {"setSayTextBlocks", false, null},
                    {"sayText", "Could I please have braces", null},
                    {"setSayTextBlocks", true, null},
                    {"startAcknowledgeDisplay", "Please place braces on my tray.", null},
                    {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X")},
            }
    );

    runUserGeneratedTest(
            "okay",
            new Object[][]{
                    {"endAcknowledgeDisplay", null},
                    {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X"), ackResponseBindings},
//                    {"sayText", "okay", null},
                    {"knowsLocation", Factory.createSymbol("homebase"), true},
                    {"goToLoc", Factory.createSymbol("homebase"), true, true},
                    {"sayText", "I have brought braces", null},
            }
    );
  }


  //Test with script?

  /**
   * Tests successfully canceling an action (interrupt behavior is enacted and AI for task terminates with status CANCELED)
   */
  @Test
  public void cancelTaskTest() {

    try {
      TRADEServiceInfo wrapperTSI= TRADE.getAvailableService(new TRADEServiceConstraints().name("addObservedOutput").argTypes(Object[].class));
      TRADEServiceInfo joinOnGoalTSI= TRADE.getAvailableService(new TRADEServiceConstraints().name("joinOnGoal").argTypes(Long.class));
      TRADE.afterService(wrapperTSI, joinOnGoalTSI);
    } catch (TRADEException e) {
      log.error("[cancelTaskTest] add join on goal wrapper", e);
    }

    runUserGeneratedTest("follow me", new Object[][]{
            {"sayText", "okay", null},
            {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X")},
    });

    runUserGeneratedTest("cancel current task", new Object[][]{
            {"followMeBlocking", null},
            {"sayText", "okay", null},
            {"cancelWaitForResponse", Factory.createPredicate("waitForAckTemi", "X"), null},
            {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X"), null},
            {"interruptFollowBlocking", null},
            {"joinOnGoal", Factory.createPredicate("followMeBlocking(self:agent)"), GoalStatus.CANCELED},
            {"joinOnGoal", Factory.createPredicate("cancelSystemGoals(self:agent)"), GoalStatus.SUCCEEDED}
    });

  }

  /**
   * Tests successfully suspending, resuming, and then completing and action
   */
  @Test
  public void suspendResumeTaskTest() {

    try {
      TRADEServiceInfo wrapperTSI= TRADE.getAvailableService(new TRADEServiceConstraints().name("addObservedOutput").argTypes(Object[].class));
      TRADEServiceInfo joinOnGoalTSI= TRADE.getAvailableService(new TRADEServiceConstraints().name("joinOnGoal").argTypes(Long.class));
      TRADE.afterService(wrapperTSI, joinOnGoalTSI);
    } catch (TRADEException e) {
      log.error("[cancelTaskTest] add join on goal wrapper", e);
    }

    runUserGeneratedTest("follow me", new Object[][]{
            {"sayText", "okay", null},
            {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X")},
    });

    runUserGeneratedTest("stop", new Object[][]{
            {"sayText", "okay", null},
            {"cancelWaitForResponse", Factory.createPredicate("waitForAckTemi", "X"), null},
            {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X"), null},
            {"followMeBlocking", null},
            {"interruptFollowBlocking", null},
    });

    long followGid = getGoalId("followMeBlocking(self:agent)");
    // is there a way to do this within the existing equalityTest?
    // EAK: no, integration tests are only meant to use TRADE calls made during the execution of the demo being tested.
    GoalStatus followStatus;
    try {
      followStatus = TRADE.getAvailableService(new TRADEServiceConstraints().name("getGoalStatus").argTypes(Long.class)).call(GoalStatus.class, followGid);

      assertEquals(followStatus, GoalStatus.SUSPENDED);
    } catch (TRADEException e) {
      log.error("Error calling getGoalStatus in suspendResumeTaskTest", e);
    }

    runUserGeneratedTest("resume", new Object[][]{
//            {"sayText", "okay", null},
            {"endFreezeTemi", null},
            {"freezeTemi", null},
            {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X")},
            {"joinOnGoal", Factory.createPredicate("freezeTemi(self:agent)"), GoalStatus.SUCCEEDED},
            {"joinOnGoal", Factory.createPredicate("endFreezeTemi(self:agent)"), GoalStatus.SUCCEEDED}
    });

    runUserGeneratedTest("okay", new Object[][]{
//            {"sayText", "okay", null},
            {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X"), ackResponseBindings},
            {"followMeBlocking", null},
            {"joinOnGoal", followGid, GoalStatus.SUCCEEDED}
    });

  }

  //Note: This functionality has changed with EM updates, will need to update
  //  temi dicts/app code as well. Canceling freeze no longer cancels the underlying
  //  goals and just ends freeze (which will pop the next goal off the queue to
  //  execute)
  /**
   * Tests successfully suspending and then canceling the suspended action
   */
  @Test
  public void suspendCancelTaskTest() {

    try {
      TRADEServiceInfo wrapperTSI= TRADE.getAvailableService(new TRADEServiceConstraints().name("addObservedOutput").argTypes(Object[].class));
      TRADEServiceInfo joinOnGoalTSI= TRADE.getAvailableService(new TRADEServiceConstraints().name("joinOnGoal").argTypes(Long.class));
      TRADE.afterService(wrapperTSI, joinOnGoalTSI);
    } catch (TRADEException e) {
      log.error("[cancelTaskTest] add join on goal wrapper", e);
    }

    runUserGeneratedTest("follow me", new Object[][]{
            {"sayText", "okay", null},
            {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X")},
    });

    long followGid = getGoalId("followMeBlocking(self:agent)");

    runUserGeneratedTest("stop", new Object[][]{
            {"followMeBlocking", null},
            {"sayText", "okay", null},
            {"cancelWaitForResponse", Factory.createPredicate("waitForAckTemi", "X"), null},
            {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X"), null},
            {"interruptFollowBlocking", null},
    });

    long freezeGid = getGoalId("freezeTemi(self:agent)");

    runUserGeneratedTest("cancel task with gid " + followGid, new Object[][]{
            {"sayText", "okay", null},
            {"cancelWaitForResponse", Factory.createPredicate("waitForAckTemi", "X"), null},
            {"interruptFollowBlocking", null},
            {"joinOnGoal", followGid, GoalStatus.CANCELED},
            {"joinOnGoal", Factory.createPredicate("cancelGoal(self:agent," + followGid + ")"), GoalStatus.SUCCEEDED},
    });

    runUserGeneratedTest("cancel current task", new Object[][]{
            {"freezeTemi", null},
            {"endFreezeTemi", null},
            {"sayText", "okay", null},
            {"joinOnGoal", freezeGid, GoalStatus.CANCELED},
            {"joinOnGoal", Factory.createPredicate("cancelSystemGoals(self:agent)"), GoalStatus.SUCCEEDED},
    });

    runUserGeneratedTest("follow me", new Object[][]{
            {"sayText", "okay", null},
            {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X")},
    });

    runUserGeneratedTest("okay", new Object[][]{
            {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X"), ackResponseBindings},
    });

  }

  /**
   * Tests successfully suspending, resuming, and then canceling and action
   */
  @Test
  public void suspendResumeCancelTaskTest() {

    try {
      TRADEServiceInfo wrapperTSI= TRADE.getAvailableService(new TRADEServiceConstraints().name("addObservedOutput").argTypes(Object[].class));
      TRADEServiceInfo joinOnGoalTSI= TRADE.getAvailableService(new TRADEServiceConstraints().name("joinOnGoal").argTypes(Long.class));
      TRADE.afterService(wrapperTSI, joinOnGoalTSI);
    } catch (TRADEException e) {
      log.error("[cancelTaskTest] add join on goal wrapper", e);
    }

    runUserGeneratedTest("follow me",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X")},
            }
    );

    runUserGeneratedTest("stop",
            new Object[][]{
                    {"followMeBlocking", null},
                    {"sayText", "okay", null},
                    {"cancelWaitForResponse", Factory.createPredicate("waitForAckTemi", "X"), null},
                    {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X"), null},
                    {"interruptFollowBlocking", null},
            }
    );

    runUserGeneratedTest("resume",
            new Object[][]{
//                    {"sayText", "okay", null},
                    {"endFreezeTemi", null},
                    {"freezeTemi", null},
                    {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X")},
                    {"joinOnGoal", Factory.createPredicate("freezeTemi(self:agent)"), GoalStatus.SUCCEEDED},
                    {"joinOnGoal", Factory.createPredicate("endFreezeTemi(self:agent)"), GoalStatus.SUCCEEDED}
            }
    );

    long followGid = getGoalId("followMeBlocking(self:agent)");

    runUserGeneratedTest("cancel current task",
            new Object[][]{
                    {"followMeBlocking", null},
                    {"sayText", "okay", null},
                    {"interruptFollowBlocking", null},
                    {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X"), null},
                    {"cancelWaitForResponse", Factory.createPredicate("waitForAckTemi", "X"), null},
                    {"joinOnGoal", followGid, GoalStatus.CANCELED},
                    {"joinOnGoal", Factory.createPredicate("cancelSystemGoals(self:agent)"), GoalStatus.SUCCEEDED}
            }
    );

  }

  /**
   * Test interruption behavior of navigation
   */
  @Test
  public void interruptGoToTest() {

    try {
      TRADEServiceInfo wrapperTSI= TRADE.getAvailableService(new TRADEServiceConstraints().name("addObservedOutput").argTypes(Object[].class));
      TRADEServiceInfo joinOnGoalTSI= TRADE.getAvailableService(new TRADEServiceConstraints().name("joinOnGoal").argTypes(Long.class));
      TRADE.afterService(wrapperTSI, joinOnGoalTSI);
    } catch (TRADEException e) {
      log.error("[cancelTaskTest] add join on goal wrapper", e);
    }

    runUserGeneratedTest("save this location as test",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"saveLocation", Factory.createSymbol("test"), null},
                    {"joinOnGoal", Factory.createPredicate("saveLocation(self:agent,test)"), GoalStatus.SUCCEEDED},
            }
    );

    Symbol locSymbol = getLocRef("test");
    assertNotNull(locSymbol);

    runUserGeneratedTest("go to test",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"knowsLocation", locSymbol, true},
//
            }
    );

    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      log.error("[interruptGoToTest]", e);
    }
    long goToGid = getGoalId("goToLocation(self:agent," + locSymbol + ",true)");

    runUserGeneratedTest("cancel current task",
            new Object[][]{
//                    {"sayTextDialogueHistory", Factory.createSymbol("Canceled current goal"), null},
                    {"goToLoc", locSymbol, true, true},
                    {"sayText", "okay", null},
//                    {"endRepositionLoop", null},
//                    {"endAcknowledgeDisplay", null},
//                    {"stopMoving", null},
                    {"interruptGoToLocation", null},
                    {"joinOnGoal", goToGid, GoalStatus.CANCELED},
                    {"joinOnGoal", Factory.createPredicate("cancelSystemGoals(self:agent)"), GoalStatus.SUCCEEDED}
            }
    );

  }

  /**
   * Tests fetch behavior from an unknown start location with default storage location (should query and go to original
   * coordinates upon receiving the target item, rather than a start location)
   */
  @Test
  public void fetchDefaultFromUnknownStartTest() {

    try {
      TRADEServiceInfo wrapperTSI= TRADE.getAvailableService(new TRADEServiceConstraints().name("addObservedOutput").argTypes(Object[].class));
      TRADEServiceInfo joinOnGoalTSI= TRADE.getAvailableService(new TRADEServiceConstraints().name("joinOnGoal").argTypes(Long.class));
      TRADE.afterService(wrapperTSI, joinOnGoalTSI);
    } catch (TRADEException e) {
      log.error("[cancelTaskTest] add join on goal wrapper", e);
    }

    runUserGeneratedTest("save this location as test",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"saveLocation", Factory.createSymbol("test"), null},
                    {"joinOnGoal", Factory.createPredicate("saveLocation(self:agent,test)"), GoalStatus.SUCCEEDED}
            }
    );

    runUserGeneratedTest("save this location as homebase",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"saveLocation", Factory.createSymbol("homebase"), null},
                    {"joinOnGoal", Factory.createPredicate("saveLocation(self:agent,homebase)"), GoalStatus.SUCCEEDED},
            }
    );

    Symbol locSymbol = getLocRef("test");
    assertNotNull(locSymbol);

    runUserGeneratedTest("go to test",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"knowsLocation", locSymbol, true},
            }
    );

    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      log.error("[fetchDefaultFromUnknownStartTest]", e);
    }
    long goToGid = getGoalId("goToLocation(self:agent," + locSymbol + ",true)");

    runUserGeneratedTest("cancel current task",
            new Object[][]{
                    //TODO:should this okay be here?
                    {"sayText", "okay", null},
//                    {"sayTextDialogueHistory", Factory.createSymbol("Canceled current goal"), null},
                    {"goToLoc", locSymbol, true, true},
//                    {"endRepositionLoop", null},
//                    {"endAcknowledgeDisplay", null},
//                    {"stopMoving", null},
                    {"interruptGoToLocation", null},
                    {"joinOnGoal", goToGid, GoalStatus.CANCELED},
                    {"joinOnGoal", Factory.createPredicate("cancelSystemGoals(self:agent)"), GoalStatus.SUCCEEDED}
            }
    );

    runUserGeneratedTest("supplies are stored in the test",
            new Object[][]{
//                    {"sayText", "okay", null},
                    {"retractBelief", Factory.createPredicate("storageArea", "X"), null},
                    {"assertBelief", Factory.createPredicate("storageArea", locSymbol.getName() + ":location"), null},
                    {"joinOnGoal", Factory.createPredicate("setStorageArea(self:agent," + locSymbol + ")"), GoalStatus.SUCCEEDED}
            }
    );


    runUserGeneratedTest("fetch me braces",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"knowsLocation", locSymbol, true},
                    {"goToLoc", locSymbol, false, true},
                    {"setSayTextBlocks", false, null},
                    {"sayText", "Could I please have braces", null},
                    {"setSayTextBlocks", true, null},
                    {"startAcknowledgeDisplay", "Please place braces on my tray.", null},
                    {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X")},
            }
    );

//    tester.setTimeoutDuration(1, TimeUnit.DAYS);

    runUserGeneratedTest("okay",
            new Object[][]{
//                    {"sayText", "okay", null},
                    {"goToPos", new ArrayList<>(Arrays.asList(1.0f, 1.0f, 1.0f)), Factory.createSymbol("Original Location"), true, true},
                    {"sayText", "I have brought braces", null},
                    {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X"), ackResponseBindings},
                    {"endAcknowledgeDisplay", null},
                    {"joinOnGoal", Factory.createPredicate("fetch(self:agent,braces)"), GoalStatus.SUCCEEDED}
            }
    );

  }

  /**
   * Tests fetch behavior from an unknown start location with explicit storage location (should query and go to original
   * coordinates upon receiving the target item, rather than a start location)
   */
  @Test
  public void fetchFromUnknownStartTest() {

    try {
      TRADEServiceInfo wrapperTSI= TRADE.getAvailableService(new TRADEServiceConstraints().name("addObservedOutput").argTypes(Object[].class));
      TRADEServiceInfo joinOnGoalTSI= TRADE.getAvailableService(new TRADEServiceConstraints().name("joinOnGoal").argTypes(Long.class));
      TRADE.afterService(wrapperTSI, joinOnGoalTSI);
    } catch (TRADEException e) {
      log.error("[cancelTaskTest] add join on goal wrapper", e);
    }

    runUserGeneratedTest("save this location as test",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"saveLocation", Factory.createSymbol("test"), null},
                    {"joinOnGoal", Factory.createPredicate("saveLocation(self:agent,test)"), GoalStatus.SUCCEEDED}
            }
    );

    runUserGeneratedTest("save this location as homebase",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"saveLocation", Factory.createSymbol("homebase"), null},
                    {"joinOnGoal", Factory.createPredicate("saveLocation(self:agent,homebase)"), GoalStatus.SUCCEEDED}
            }
    );

    Symbol locSymbol = getLocRef("test");
    assertNotNull(locSymbol);

    runUserGeneratedTest("go to test",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"knowsLocation", locSymbol, true},
            }
    );

    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      log.error("[fetchFromUnknownStartTest]", e);
    }

    long goToGid = getGoalId("goToLocation(self:agent," + locSymbol + ",true)");

    runUserGeneratedTest("cancel current task",
            new Object[][]{
                    {"goToLoc", locSymbol, true, true},
                    {"sayText", "okay", null},
//                    {"endRepositionLoop", null},
//                    {"endAcknowledgeDisplay", null},
//                    {"stopMoving", null},
                    {"interruptGoToLocation", null},
                    {"joinOnGoal", goToGid, GoalStatus.CANCELED},
//                    {"sayTextDialogueHistory", Factory.createSymbol("Canceled current goal"), null},
                    {"joinOnGoal", Factory.createPredicate("cancelSystemGoals(self:agent)"), GoalStatus.SUCCEEDED}
            }
    );

    runUserGeneratedTest("fetch me braces from test",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"knowsLocation", locSymbol, true},
                    {"goToLoc", locSymbol, false, true},
                    {"setSayTextBlocks", false, null},
                    {"sayText", "Could I please have braces", null},
                    {"setSayTextBlocks", true, null},
                    {"startAcknowledgeDisplay", "Please place braces on my tray.", null},
                    {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X")},
            }
    );

    runUserGeneratedTest("okay",
            new Object[][]{
//                    {"sayText", "okay", null},
                    {"endAcknowledgeDisplay", null},
                    {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X"), ackResponseBindings},
                    {"goToPos", new ArrayList<>(Arrays.asList(1.0f, 1.0f, 1.0f)), Factory.createSymbol("Original Location"), true, true},
                    {"sayText", "I have brought braces", null},
            }
    );
  }

  /**
   * Test for ensuring belief at(self,loc) is correctly asserted for locs with special characters
   * Test by seeing if fetch correctly returns to original location with special characters if that is the starting
   * location (needs to be directly after saveLocation so that it is the name of the location and not ref)
   */
  @Test
  public void assertAtSpecialCharacterTest() {

    runUserGeneratedTest(
            "save this location as fetchDest",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"saveLocation", Factory.createSymbol(escapeSpecialCharacters("fetchdest")), null}
            }
    );

    runUserGeneratedTest(
            "save this location as newl0cn@me's",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"saveLocation", Factory.createSymbol(escapeSpecialCharacters("newl0cn@me's")), null}
            }
    );

    Symbol locSymbol = getLocRef("fetchdest");

    runUserGeneratedTest(
            "fetch me braces from fetchdest",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"knowsLocation", locSymbol, true},
                    {"goToLoc", locSymbol, false, true},
                    {"setSayTextBlocks", false, null},
                    {"sayText", "Could I please have braces", null},
                    {"setSayTextBlocks", true, null},
                    {"startAcknowledgeDisplay", "Please place braces on my tray.", null},
                    {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X")},
            }
    );

    runUserGeneratedTest("okay",
            new Object[][]{
                    {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X"), ackResponseBindings},
                    {"endAcknowledgeDisplay", null},
//                    {"sayText", "okay", null},
                    {"knowsLocation", Factory.createSymbol(escapeSpecialCharacters("newl0cn@me's")), true},
                    {"goToLoc", Factory.createSymbol(escapeSpecialCharacters("newl0cn@me's")), true, true},
                    {"sayText", "I have brought braces", null},
            }
    );
  }

  /**
   * Tests escort behavior without specifying a start location
   */
  @Test
  public void escortNoFromLocationTest() {

    runUserGeneratedTest("save this location as test", new Object[][]{
            {"sayText", "okay", null},
            {"saveLocation", Factory.createSymbol("test"), null}
    });

    runUserGeneratedTest(
            "save this location as homebase",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"saveLocation", Factory.createSymbol("homebase"), null}
            }
    );

    Symbol locSymbol = getLocRef("test");

    runUserGeneratedTest(
            "setescortdurationto 10",
            new Object[][]{
                    {"setEscortWaitDuration", Factory.createSymbol("10"), null}
            }
    );

    float waitTime = config.temi.getEscortWaitDuration();
    int waitAttempts = config.temi.getEscortWaitAttempts();

    runUserGeneratedTest(
            "escort the patient to test",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"sayText", "Hello patient. Please come with me. I will show you to test.", null},
                    {"startAcknowledgeDisplay", "Hello patient. Please come with me. I will show you to test.", null},
                    {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X"), 10L},
            }
    );

    runUserGeneratedTest(
            "okay",
            new Object[][]{
//                    {"sayText", "okay", null},
                    {"endAcknowledgeDisplay", null},
                    {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X"), 10L, ackResponseBindings},
                    {"knowsLocation", locSymbol, true},
                    {"goToLoc", locSymbol, true, true},
                    {"sayText", "Arrived at test", null},
            }
    );
  }

  /**
   * Tests escort behavior where the start location is the current location
   */
  @Test
  public void escortFromCurrentLocationTest() {

    runUserGeneratedTest(
            "save this location as homebase",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"saveLocation", Factory.createSymbol("homebase"), null}
            }
    );

    runUserGeneratedTest("save this location as test", new Object[][]{
            {"sayText", "okay", null},
            {"saveLocation", Factory.createSymbol("test"), null}
    });

    runUserGeneratedTest(
            "setescortdurationto 10",
            new Object[][]{
                    {"setEscortWaitDuration", Factory.createSymbol("10"), null}
            }
    );

    float waitTime = config.temi.getEscortWaitDuration();
    int waitAttempts = config.temi.getEscortWaitAttempts();

    Symbol locSymbol = getLocRef("homebase");
    Symbol testLocSymbol = getLocRef("test");

    runUserGeneratedTest(
            "escort the patient from test to homebase",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"knowsLocation", testLocSymbol, true},
                    {"goToLoc", testLocSymbol, false, true},
                    {"startAcknowledgeDisplay", "Hello patient. Please come with me. I will show you to homebase.", null},
                    {"sayText", "Hello patient. Please come with me. I will show you to homebase.", null},
                    {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X"), 10L},
            }
    );

    runUserGeneratedTest(
            "okay",
            new Object[][]{
                    //don't really want tentativeAccept on acknowledgement
//                    {"sayText", "okay", null},
                    {"endAcknowledgeDisplay", null},
                    {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X"), 10L, ackResponseBindings},
                    {"knowsLocation", locSymbol, true},
                    {"goToLoc", locSymbol, true, true},
                    {"sayText", "Arrived at homebase", null},
            }
    );

  }

  //Tests interruption behavior for waitForAck
  @Test
  public void interruptWaitForAckTest() {
    try {
      TRADEServiceInfo wrapperTSI= TRADE.getAvailableService(new TRADEServiceConstraints().name("addObservedOutput").argTypes(Object[].class));
      TRADEServiceInfo joinOnGoalTSI= TRADE.getAvailableService(new TRADEServiceConstraints().name("joinOnGoal").argTypes(Long.class));
      TRADE.afterService(wrapperTSI, joinOnGoalTSI);
    } catch (TRADEException e) {
      log.error("[cancelTaskTest] add join on goal wrapper", e);
    }

    runUserGeneratedTest("save this location as test",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"saveLocation", Factory.createSymbol("test"), null},
                    {"joinOnGoal", Factory.createPredicate("saveLocation(self:agent,test)"), GoalStatus.SUCCEEDED}
            }
    );

    Symbol locSymbol = getLocRef("test");
    assertNotNull(locSymbol);

    runUserGeneratedTest(
            "setescortdurationto 10",
            new Object[][]{
                    {"setEscortWaitDuration", Factory.createSymbol("10"), null},
                    {"joinOnGoal", Factory.createPredicate("setEscortWaitDuration(self:agent,10)"), GoalStatus.SUCCEEDED}
            }
    );

    float waitTime = config.temi.getEscortWaitDuration();
    int waitAttempts = config.temi.getEscortWaitAttempts();


    runUserGeneratedTest("escort the patient from test to test",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"knowsLocation", locSymbol, true},
                    {"goToLoc", locSymbol, false, true},
                    {"sayText", "Hello patient. Please come with me. I will show you to test.", null},
                    {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X"), 10L},
                    {"startAcknowledgeDisplay", "Hello patient. Please come with me. I will show you to test.", null},
            }
    );

    long escortGid = getGoalId("escort(self:agent," + locSymbol + ",patient," + locSymbol + ")");

    runUserGeneratedTest("cancel current task",
            new Object[][]{
                    {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X"), 10L, null},
                    {"cancelWaitForResponse", Factory.createPredicate("waitForAckTemi", "X"), null},
                    {"sayText", "okay", null},
                    {"endAcknowledgeDisplay", null},
                    {"joinOnGoal", escortGid, GoalStatus.CANCELED},
//                    {"sayTextDialogueHistory", Factory.createSymbol("Canceled current goal"), null},
                    {"joinOnGoal", Factory.createPredicate("cancelSystemGoals(self:agent)"), GoalStatus.SUCCEEDED}
            }
    );

  }


  /**
   * Tests goToThenPlayVideo behavior
   */
  @Test
  public void goToThenPlayVideoTest() {

    config.parser.injectDictionaryEntry("flossing", "VID", "flossing", "");

    runUserGeneratedTest(
            "play video flossing",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"playVideo", Factory.createSymbol("flossing"), null},
            }
    );

    runUserGeneratedTest(
            "save this location as test",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"saveLocation", Factory.createSymbol("test"), null}
            }
    );

    Symbol locSymbol = getLocRef("test");

    runUserGeneratedTest(
            "go to test then play video flossing",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"knowsLocation", locSymbol, true},
                    {"goToLoc", locSymbol, false, true},
                    {"playVideo", Factory.createSymbol("flossing"), null},
            }
    );
  }

  /**
   * Tests displayQRCode behavior
   */
  @Test
  public void displayQRTest() {

    runUserGeneratedTest(
            "display qr code www.google.com with message hello",
            new Object[][]{
                    {"sayText", "okay", null},
            }
    );


    //TODO:brad: in this case the user would need to hit it multiple times for things to work, is that what we want? do we want some sort of queue?
    //need to sleep here to simulate delay input for the ack to work
    try {
      Thread.sleep(5000);
    } catch (InterruptedException e) {
      log.error("[displayQRTest]", e);
    }

    runUserGeneratedTest(
            "okay",
            new Object[][]{
                    {"startQRCodeDisplay", "www.google.com", "hello", null}
            }
    );

  }

  @Test
  public void saySpecialCharacterTest() {
    //log.info("\n\n\n\n\n\n\n\n\n\nNEWTEST\n[saySpecialCharacterTest]");

    runUserGeneratedTest(
            "say hello's",
            new Object[][]{
                    {"sayText", "hello's", null},
            }
    );

    runUserGeneratedTest(
            "say hello\"s",
            new Object[][]{
                    {"sayText", "hello\"s", null},
            }
    );

  }

  /**
   * Prevent location names from colliding with other dictionary entries, introduces possibility for previously valid
   * utterances to be misunderstood
   */
  @Test
  public void conflictingDictionaryEntryTest() {

    runUserGeneratedTest(
            "setescortdurationto 10",
            new Object[][]{
                    {"setEscortWaitDuration", Factory.createSymbol("10"), null}
            }
    );

    long waitTime = config.temi.getEscortWaitDuration();
    int waitAttempts = config.temi.getEscortWaitAttempts();

    runUserGeneratedTest(
            "save this location as escortFromLocation",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"saveLocation", Factory.createSymbol(escapeSpecialCharacters("escortfromlocation")), null}
            }
    );

    Symbol locSymbol = getLocRef("escortfromlocation");

    runUserGeneratedTest(
            "save this location as escortToLocation",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"saveLocation", Factory.createSymbol(escapeSpecialCharacters("escorttolocation")), null}
            }
    );

    runUserGeneratedTest(
            "save this location as 2",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"saveLocation", Factory.createSymbol("2"), null}
            }
    );

    runUserGeneratedTest(
            "escort the patient from escortFromLocation to escortToLocation",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"knowsLocation", locSymbol, true},
                    {"goToLoc", locSymbol, false, true},
                    {"startAcknowledgeDisplay", "Hello patient. Please come with me. I will show you to escorttolocation.", null},
                    {"sayText", "Hello patient. Please come with me. I will show you to escorttolocation.", null},
                    {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X"), waitTime},
            }
    );

    locSymbol = getLocRef("escorttolocation");

    runUserGeneratedTest(
            "okay",
            new Object[][]{
//                    {"sayText", "okay", null},
                    {"endAcknowledgeDisplay", null},
                    {"waitForResponse", Factory.createPredicate("waitForAckTemi", "X"), waitTime, ackResponseBindings},
                    {"knowsLocation", locSymbol, true},
                    {"goToLoc", locSymbol, true, true},
                    {"sayText", "Arrived at escorttolocation", null},
            }
    );

  }

  /**
   * Tests various special characters within location names
   */
  @Test
  public void specialLocationNamesTest() {

    List<String> locNames = new ArrayList<>(Arrays.asList("newLocation", "NewLocation", "new location", "tr's office", "tr\"s office", "tr's \"off'i\"ce", "special:Name", "  special : Name   ", "!test", "t!est", "?test", "sp~!`@#$%^&*()_-+=[{]}|\''/?.>,< ", "@", "te\"st loc\"ation", "te\"st", "te,st", "te$t", "te:st", "te\")st"));

    for (String locName : locNames) {
      runUserGeneratedTest(
              "save this location as " + locName,
              new Object[][]{
                      {"sayText", "okay", null},
                      {"saveLocation", Factory.createSymbol(escapeSpecialCharacters(locName.toLowerCase().trim())), null}
              }
      );

      Symbol locSymbol = getLocRef(Factory.createSymbol(escapeSpecialCharacters(locName.toLowerCase().trim())).getName());

      runUserGeneratedTest(
              "go to " + locName,
              new Object[][]{
                      {"sayText", "okay", null},
                      {"knowsLocation", locSymbol, true},
                      {"goToLoc", locSymbol, true, true},
              }
      );
    }
  }

  /**
   * Test for saving the same location (containing special characters) twice
   * Make sure whatever comparator being used for retrieving/overwriting the location from the consultant
   * correctly accounts for special characters (i.e. doesn't create two refs for same name)
   */
  @Test
  public void saveSpecialLocationTwiceTest() {

    runUserGeneratedTest(
            "save this location as te\"st's",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"saveLocation", Factory.createSymbol(escapeSpecialCharacters("te\"st's")), null}
            }
    );

    runUserGeneratedTest(
            "save this location as te\"st's",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"saveLocation", Factory.createSymbol(escapeSpecialCharacters("te\"st's")), null}
            }
    );

    Symbol locSymbol = getLocRef("\"te\"st's\"");

    runUserGeneratedTest(
            "go to te\"st's",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"knowsLocation", locSymbol, true},
                    {"goToLoc", locSymbol, true, true},
            }
    );

  }

  /**
   * Test for failure responses with special characters
   */
  //TODO: This test needs to change because deleting a location now fully deletes the reference and just won't parse
  //      rather than fail.
  //      Additional issue is that now the observed behavior is 'go to test then say something' parses just to 'say something' rather than then(goTo..,say..)
  @Test
  @Ignore
  public void specialCharacterFailResponseTest() {

    runUserGeneratedTest(
            "save this location as test",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"saveLocation", Factory.createSymbol(escapeSpecialCharacters("test")), null}
            }
    );

    Symbol locSymbol = getLocRef("test");

    runUserGeneratedTest(
            "delete location test",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"deleteLocation", locSymbol, null},
                    {"retractBelief", Factory.createPredicate("at(self:agent,test)"), null}
            }
    );

    tester.setTimeoutDuration(1, TimeUnit.DAYS);
    runUserGeneratedTest(
            "go to test then say thi\")s contains, special character's",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"knowsLocation", locSymbol, false},
                    {"sayText", "I can not go To Then Say the test \"thi\")s contains, special character's\" because I know not Where the test", null},
            }
    );

  }

  @Test
  public void quotedLocationTest() {

    runUserGeneratedTest(
            "save this location as \"quoted location\"",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"saveLocation", Factory.createSymbol(escapeSpecialCharacters("\"quoted location\"")), null}
            }
    );

    Symbol locSymbol = getLocRef(Factory.createSymbol(escapeSpecialCharacters("\"quoted location\"")).getName());

    runUserGeneratedTest(
            "go to \"quoted location\"",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"knowsLocation", locSymbol, true},
                    {"goToLoc", locSymbol, true, true},
            }
    );

  }

  /**
   * Make sure locations with : going into consultant/resolver are not being mistakenly split into a name and type
   */
  @Test
  public void mistakenTypeTest() {

    runUserGeneratedTest(
            "save this location as te:st",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"saveLocation", Factory.createSymbol(escapeSpecialCharacters("te:st")), null}
            }
    );

    runUserGeneratedTest(
            "save this location as te",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"saveLocation", Factory.createSymbol(escapeSpecialCharacters("te")), null}
            }
    );

    Symbol locSymbol = getLocRef("\"te:st\"");
    assertNotSame(locSymbol, getLocRef("te"));

    runUserGeneratedTest(
            "go to te:st",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"knowsLocation", locSymbol, true},
                    {"goToLoc", locSymbol, true, true},
            }
    );
  }

  /**
   * tokenizeArgs cannot disambiguate cases where an arbitrary string contains "\",\""
   * In this case it is unclear whether there are two escaped string args or one longer one
   * 1. "example" 2. "string" or 1. "example\",\"string"
   */
  @Test
  public void ambiguousArbitraryStringTest() {
    runUserGeneratedTest(
            "save this location as ambiguous\",\"locname",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"sayText", "I can not save Location \"ambiguous\" \"locname\" because I do not know how to save Location ambiguous locname", null}
            }
    );

    runUserGeneratedTest(
            "save this location as this is my \", \", te,\" st\",,\"\" location,",
            new Object[][]{
                    {"sayText", "okay", null},
                    {"sayText", "I can not save Location \"this is my \", \" te \" st\",\"\" location,\" because I do not know how to save Location \"this is my \\34\\, \" te \" st\\34\\,\\34\\\\34\\ location,\"", null}
            }
    );

  }

  //TODO: add various tests for the queue (normal operation, add same goal twice and cancel/remove both from queue and current, ...)
}
