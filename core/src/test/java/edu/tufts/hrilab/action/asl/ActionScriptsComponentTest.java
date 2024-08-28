/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.asl;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.junit.AfterClass;
import org.junit.Assert;
import org.junit.BeforeClass;
import org.junit.Test;

import java.util.ArrayList;

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
      log.error("Could not register TSCTestHelper.", e);
    }

    // create goal and execute
    goalPredicate = Factory.createPredicate("test4(self:agent)");
    executeAndCheckGoal(goalPredicate, GoalStatus.SUCCEEDED);

    try {
      TRADE.deregister(helper);
    } catch (TRADEException e) {
      log.error("Could not de-register TSCTestHelper.", e);
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
      log.error("Could not register TSCTestHelper.", e);
    }

    // create goal and execute
    executeAndCheckGoal(Factory.createPredicate("runPassTest1(self:agent)"), GoalStatus.SUCCEEDED);

    executeAndCheckGoal(Factory.createPredicate("runFailTest1(self:agent)"), GoalStatus.FAILED);

    executeAndCheckGoal(Factory.createPredicate("runFailTest2(self:agent)"), GoalStatus.FAILED);

    executeAndCheckGoal(Factory.createPredicate("runBooleanTest(self:agent)"), GoalStatus.SUCCEEDED);

    try {
      TRADE.deregister(helper);
    } catch (TRADEException e) {
      log.error("Could not de-register TSCTestHelper.", e);
    }

    // re-register TSCTestHelper in agent group
    try {
      TRADE.registerAllServices(helper, "agent:hugo:agent");
    } catch (TRADEException e) {
      log.error("Could not register TSCTestHelper.", e);
    }

    executeAndCheckGoal(Factory.createPredicate("runFailTest3(self:agent)"), GoalStatus.FAILED);

    executeAndCheckGoal(Factory.createPredicate("runPassTest2(self:agent)"), GoalStatus.SUCCEEDED);

    try {
      TRADE.deregister(helper);
    } catch (TRADEException e) {
      log.error("Could not de-register TSCTestHelper.", e);
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
}
