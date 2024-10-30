/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.asl;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.junit.AfterClass;
import org.junit.Assert;
import org.junit.BeforeClass;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

public class ActionScriptsComponentTest {
  private static GoalManagerComponent component;
  private static Logger log = LoggerFactory.getLogger(ActionScriptsComponentTest.class);
  private String aslPath = "/config/edu/tufts/hrilab/action/asl/";

  @BeforeClass
  public static void init() {
    component = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile demos.pl -beliefinitfile agents/agents.pl");
  }

  @AfterClass
  public static void shutdown(){
    component.shutdownComponent();
  }

  @Test
  public void testSettingDynamicActorScript() {
    String filename = aslPath + "actor_dynamic_setting.asl";
    Database.getInstance().loadDatabaseFromFile(filename);

    // create goal and execute
    Predicate goalPredicate = Factory.createPredicate("runTest(self:agent)");
    executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
  }

  @Test
  public void testJavaObjectsScript() {
    String filename = aslPath + "javaObjects.asl";
    Database.getInstance().loadDatabaseFromFile(filename);

    // create goal and execute
    Predicate goalPredicate = Factory.createPredicate("javaMethodsTest(self:agent)");
    executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);


    // create goal and execute
    goalPredicate = Factory.createPredicate("controlTestWithJavaMethods(self:agent)");
    executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
  }

  @Test
  public void testStiScript() {
    String filename = aslPath + "sti_tests.asl";
    Database.getInstance().loadDatabaseFromFile(filename);

    // create goal and execute
    Predicate goalPredicate = Factory.createPredicate("stiTests(self:agent)");
    executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
  }

  /**
   * TODO: this test doesn't currently test anything useful as the script just logs stuff instead
   * of failing in the failure case
   */
  @Test
  public void testBug339Script() {
    String filename = aslPath + "bug-399.asl";
    Database.getInstance().loadDatabaseFromFile(filename);

    // create goal and execute
    Predicate goalPredicate = Factory.createPredicate("loop(self:agent)");
    executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
  }

  @Test
  public void testForEachScript() {
    String filename = aslPath + "for-each.asl";
    Database.getInstance().loadDatabaseFromFile(filename);

    // create goal and execute
    Predicate goalPredicate = Factory.createPredicate("forEachTest(self:agent)");
    executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
  }

  @Test
  public void testForLoopScript() {
    String filename = aslPath + "for-loop.asl";
    Database.getInstance().loadDatabaseFromFile(filename);

    // create goal and execute
    Predicate goalPredicate = Factory.createPredicate("test(self:agent)");
    executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
  }

  @Test
  public void testStringsScript() {
    String filename = aslPath + "strings.asl";
    Database.getInstance().loadDatabaseFromFile(filename);

    // create goal and execute
    Predicate goalPredicate = Factory.createPredicate("stringTest(self:agent)");
    executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);

    // create goal and execute
    goalPredicate = Factory.createPredicate("concatStringTest(self:agent)");
    executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
  }

  @Test
  public void testTestLucaScript() {
    String filename = aslPath + "testluca.asl";
    Database.getInstance().loadDatabaseFromFile(filename);

    // create goal and execute
    Predicate goalPredicate = Factory.createPredicate("testLuca(self:agent)");
    executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
  }

  @Test
  public void testTryCatchScript() {
    String filename = aslPath + "trycatch_tests.asl";
    Database.getInstance().loadDatabaseFromFile(filename);

    // create goal and execute
    Predicate goalPredicate = Factory.createPredicate("tryCatchTestSuccess(self:agent)");
    executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);

    // create goal and execute
    goalPredicate = Factory.createPredicate("tryCatchTestFail(self:agent)");
    executeAndCheckGoal(goalPredicate, GoalStatus.FAILED);

    // create goal and execute
    goalPredicate = Factory.createPredicate("finallyTest1(self:agent)");
    executeAndCheckGoal(goalPredicate, GoalStatus.FAILED);

    // create goal and execute
    goalPredicate = Factory.createPredicate("finallyTest2(self:agent)");
    executeAndCheckGoal(goalPredicate, GoalStatus.FAILED);

    // create goal and execute
    goalPredicate = Factory.createPredicate("finallyTest3(self:agent)");
    executeAndCheckGoal(goalPredicate, GoalStatus.FAILED);

    // create goal and execute
    goalPredicate = Factory.createPredicate("finallyTest4(self:agent)");
    executeAndCheckGoal(goalPredicate, GoalStatus.FAILED);

    // create goal and execute
    goalPredicate = Factory.createPredicate("finallyTest5(self:agent)");
    TestHelper helper = new TestHelper();
    try {
      TRADE.registerAllServices(helper, new ArrayList<>());
    } catch (TRADEException e) {
      log.error("Error registering TestHelper with TRADE.", e);
    }
    Long goalId = component.submitGoal(goalPredicate);
    component.joinOnGoal(goalId);
    Assert.assertTrue(helper.testStrings.contains("finallyTest5"));

    try {
      TRADE.deregister(helper);
    } catch (TRADEException e) {
      log.error("Could not de-register TestHelper.", e);
    }
  }

  @Test
  public void testControlTestScript() {
    String filename = aslPath + "controlTest.asl";
    Database.getInstance().loadDatabaseFromFile(filename);

    // create goal and execute
    Predicate goalPredicate = Factory.createPredicate("test1(self:agent)");
    executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);

    // create goal and execute
    goalPredicate = Factory.createPredicate("test2(self:agent)");
    executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);

    // create goal and execute
    goalPredicate = Factory.createPredicate("test2(self:agent)");
    executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);

    // instantiate test TRADE services
    TestHelper helper = new TestHelper();
    try {
      TRADE.registerAllServices(helper,new ArrayList<>());
    } catch (TRADEException e) {
      log.error("Could not register TestHelper.", e);
    }

    // create goal and execute
    goalPredicate = Factory.createPredicate("test4(self:agent)");
    executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);

    try {
      TRADE.deregister(helper);
    } catch (TRADEException e) {
      log.error("Could not de-register TestHelper.", e);
    }
  }

  @Test
  public void testAsyncScript() {
    String filename = aslPath + "async_tests.asl";
    Database.getInstance().loadDatabaseFromFile(filename);

    // create goal and execute
    // TODO: this test does not currently fail even if things behave incorrectly. currently it only logs messages in the script
    Predicate goalPredicate = Factory.createPredicate("asyncTest(self:agent)");
    executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);

    // create goal and execute
    // TODO: this test does not currently fail even if things behave incorrectly. currently it only logs messages in the script
    goalPredicate = Factory.createPredicate("asyncTest2(self:agent)");
    executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);

    // create goal and execute
    goalPredicate = Factory.createPredicate("exitTest(self:agent)");
    executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
  }

  @Test
  public void testMultipleReturnsScript() {
    String filename = aslPath + "multiple-returns-test.asl";
    Database.getInstance().loadDatabaseFromFile(filename);

    // create goal and execute
    // TODO: this test does not currently fail even if things behave incorrectly. currently it only logs messages in the script
    Predicate goalPredicate = Factory.createPredicate("test(self:agent)");
    executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
  }

  @Test
  public void testObservationScript() {
    String filename = aslPath + "observation_tests.asl";
    Database.getInstance().loadDatabaseFromFile(filename);

    // create goal and execute
    Predicate goalPredicate = Factory.createPredicate("observationTest(self:agent)");
    executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);
  }

  @Test
  public void testTscScript() {
    String filename = aslPath + "tsc_tests.asl";
    Database.getInstance().loadDatabaseFromFile(filename);

    // instantiate test TRADE services
    TestHelper helper = new TestHelper();
    try {
      TRADE.registerAllServices(helper,new ArrayList<>());
    } catch (TRADEException e) {
      log.error("Could not register TestHelper.", e);
    }

    // create goal and execute
    executeAndCheckGoal(Factory.createPredicate("runPassTest1(self:agent)"), GoalStatus.SUCCEEDED);

    executeAndCheckGoal(Factory.createPredicate("runFailTest1(self:agent)"), GoalStatus.FAILED);

    executeAndCheckGoal(Factory.createPredicate("runFailTest2(self:agent)"), GoalStatus.FAILED);

    executeAndCheckGoal(Factory.createPredicate("runBooleanTest(self:agent)"), GoalStatus.SUCCEEDED);

    try {
      TRADE.deregister(helper);
    } catch (TRADEException e) {
      log.error("Could not de-register TestHelper.", e);
    }

    // re-register TestHelper in agent group
    try {
      TRADE.registerAllServices(helper, "agent:hugo:agent");
    } catch (TRADEException e) {
      log.error("Could not register TestHelper.", e);
    }

    executeAndCheckGoal(Factory.createPredicate("runFailTest3(self:agent)"), GoalStatus.FAILED);

    executeAndCheckGoal(Factory.createPredicate("runPassTest2(self:agent)"), GoalStatus.SUCCEEDED);

    try {
      TRADE.deregister(helper);
    } catch (TRADEException e) {
      log.error("Could not de-register TestHelper.", e);
    }
  }

  @Test
  public void testASLSignatures() {

    String filename = aslPath + "asl_signature_tests.asl";
    Database.getInstance().loadDatabaseFromFile(filename);

    // instantiate test TRADE services
    TestHelper helper = new TestHelper();
    try {
      TRADE.registerAllServices(helper,new ArrayList<>());
    } catch (TRADEException e) {
      log.error("Could not register TestHelper.", e);
    }

    // create goal and execute
    executeAndCheckGoal(Factory.createPredicate("successTests(self:agent)"), GoalStatus.SUCCEEDED);

    try {
      TRADE.deregister(helper);
    } catch (TRADEException e) {
      log.error("Could not de-register TestHelper.", e);
    }
  }

  /**
   * Helper method to submit a goal, wait for it to finish, and then check the status.
   * @param goalPredicate
   * @param desiredStatus
   */
  private void executeAndCheckGoal(Predicate goalPredicate, GoalStatus desiredStatus) {
    Long goalId = component.submitGoal(goalPredicate);
    component.joinOnGoal(goalId);
    Assert.assertEquals(desiredStatus, component.getGoalStatus(goalId));
  }


  /**
   * Helper class to expose various TRADEServices for testing ASL.
   */
  static public class TestHelper {
    private static Logger log = LoggerFactory.getLogger(ActionScriptsComponentTest.TestHelper.class);

    List<String> testStrings = new ArrayList<>();

    @TRADEService
    public void cacheTestString(String testString) {
      testStrings.add(testString);
    }

    @TRADEService
    public void tsc1() {
      log.info("tsc1");
    }

    @TRADEService
    public void tsc2(Symbol arg) {
      log.info("tsc2 arg: {}", arg);
    }

    @TRADEService
    public void tsc3(Symbol actor, Symbol arg) {
      log.info("tsc3 actor: {} arg: {}", actor, arg);
    }

    @TRADEService
    public Justification tsc4() {
      log.info("tsc4");
      return new ConditionJustification(true, Factory.createPredicate("succeeded(tsc4)"));
    }

    @TRADEService
    public boolean tsc_bool_true() {
      log.info("tsc_bool_true");
      return true;
    }

    @TRADEService
    public boolean tsc_bool_false() {
      log.info("tsc_bool_true");
      return false;
    }

    @TRADEService
    public Justification tsc_justification_true() {
      log.info("tsc_justification_true");
      return new ConditionJustification(true);
    }

    @TRADEService
    public Justification tsc_justification_false() {
      log.info("tsc_justification_false");
      return new ConditionJustification(false);
    }

    @Action
    @TRADEService
    public boolean act_bool_true() {
      log.info("act_bool_true");
      return true;
    }

    @Action
    @TRADEService
    public boolean act_bool_false() {
      log.info("act_bool_false");
      return false;
    }

    @Action
    @TRADEService
    public Justification act_justification_true() {
      log.info("act_justification_true");
      return new ConditionJustification(true);
    }

    @Action
    @TRADEService
    public Justification act_justification_false() {
      log.info("act_justification_false");
      return new ConditionJustification(false);
    }
  }
}
