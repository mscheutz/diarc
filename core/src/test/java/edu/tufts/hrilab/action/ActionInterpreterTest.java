/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action;

import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.execution.ActionContext;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.execution.ExecutionType;
import edu.tufts.hrilab.action.execution.FailureContext;
import edu.tufts.hrilab.action.execution.RootContext;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.priority.LinearPriorityCalculator;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;

import java.util.HashSet;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;
import static org.junit.Assert.*;

/**
 *
 * @author willie
 */
public class ActionInterpreterTest {
  private static Logger log = LoggerFactory.getLogger(ActionInterpreterTest.class);
  private RootContext root;
  private ActionInterpreter instance = null;
  private Goal goal;
  private StateMachine sm;
  private Symbol actor = Factory.createSymbol("self");

  @BeforeClass
  public static void setUpClass() {
//    ActionResourceLock.motionLock = new ActionResourceLockLinear("motionLock");
    StateMachine sm = StateMachine.createTestStateMachine(new HashSet<>());
    edu.tufts.hrilab.action.ActionBuilderRepo.buildLookforAction();
    edu.tufts.hrilab.action.ActionBuilderRepo.buildMoveToObjectAction();
    sm.shutdown();
  }

  @AfterClass
  public static void deregister(){
      Database.destroyInstance();
  }

  @Before
  public void createStateMachine(){
    HashSet<Predicate> initFacts = new HashSet<>();
    initFacts.add(Factory.createPredicate("diarcAgent(self)"));
    initFacts.add(Factory.createPredicate("object(self,agent)"));
    initFacts.add(Factory.createPredicate("team(self)"));
    initFacts.add(Factory.createPredicate("memberOf(self,self)"));
    sm= StateMachine.createTestStateMachine(initFacts);
    root = new RootContext(new ActionConstraints(),sm);
  }

  @After
  public void deregisterStateMachine(){
    sm.shutdown();
  }
  
  /**
   * Test instantiating ActionInterpreter given EventSpec
   */
  @Test
  public void testInstantiationEventSpec() {
    //ActionDBEntry adb = ActionDatabase.createDatabaseFromFile("com/action/asl/actioncore.xml", false);
    //assertTrue(adb != null);
    //assertNotNull(ActionDatabase.lookupPost(Factory.createPredicate("handLifted(?mover, ?hand)")));
    goal = new Goal(actor, Factory.createPredicate("located(ball)"));
    EventSpec eventSpec = new EventSpec(EventSpec.EventType.ACTION, "lookfor(self, home, ball)");
    instance = ActionInterpreter.createInterpreterFromEventSpec(goal, root, eventSpec);
    assertNotNull(instance);
    assertNotNull(instance.getGoal());
  }

  /**
   * Test instantiating ActionInterpreter given EventSpec
   */
  @Test
  public void testInstantiationGoal() {
    
    goal = new Goal(actor, Factory.createPredicate("located(ball)"));
	instance = aiForGoal(goal);

    assertNotNull(instance);
    assertEquals("located(ball)", instance.getGoal().getPredicate().toString());
  }

  //TODO:brad: do these even need to exist?
  @Test
  @Ignore
  public void testgetActionStartTime() {
    assertTrue(5 == 5);
  }

  @Test
  @Ignore
  public void testgetActionMaxTime() {
    assertTrue(5 == 5);
  }

  @Test
  @Ignore
  public void testgetActionMinUrg() {
    assertTrue(5 == 5);
  }

  @Test
  @Ignore
  public void testgetActionMaxUrg() {
    assertTrue(5 == 5);
  }

  @Test
  @Ignore
  public void testgetActionBenefit() {
    assertTrue(5 == 5);
  }

  @Test
  public void testgetActionCost() {
    String goalString = "located(ball)";
    goal = new Goal(actor, Factory.createPredicate(goalString));
    instance = aiForGoal(goal);
    instance.call(); // run goal
    Context goalContext = instance.getRoot();
    ActionContext selectedActionContext = (ActionContext) goalContext.getChildContexts().get(0);
    assertEquals(0.23, selectedActionContext.getCost(), 0.0);
  }

  @Test
  public void testPriorities() {
    String goalString = "located(ball)";
    goal = new Goal(actor, Factory.createPredicate(goalString));
    instance = aiForGoal(goal);

    instance.updatePriority(new LinearPriorityCalculator());
    assertEquals(0.0, instance.getActionPriority(), 0.0);

    //EAK: sleep isn't precise enough to reliable run this test -- not sure how to fix it
//    try {
//      sleep(2);
//    } catch (InterruptedException ex) {
//      Logger.getLogger(ActionInterpreterTest.class.getName()).log(Level.SEVERE, null, ex);
//    }
//    instance.updatePriority(new UtilityUrgencyPriorityCalculator());
//    assertEquals(0.00168, instance.getActionPriority(), 0.0006);

  }
  
  @Test
  public void testLocks() {
    String goalString = "located(ball)";
    goal = new Goal(actor, Factory.createPredicate(goalString));
    instance = aiForGoal(goal);
  }

  @Test
  public void testRun() {
    Predicate goalState = Factory.createPredicate("located(ball)");
    goal = new Goal(actor, goalState);

    instance = aiForGoal(goal);
    
    log.debug("starting instance...");
    instance.call();

    assertTrue(sm.holds(goalState).getValue());
  }

  //@Ignore // this test seems to cause other tests to never finish -- not sure what's going on here
  @Test
  public void testRunPreFail() {
    log.debug("testRunPreFail");
    
    String goalString = "movedTo(ball)";
    goal = new Goal(actor, Factory.createPredicate(goalString));

    instance = aiForGoal(goal);
    
    log.debug("starting instance...");
    instance.call();

    assertEquals(GoalStatus.FAILED, goal.getStatus());
    assertEquals(ActionStatus.FAIL_PRECONDITIONS, goal.getRootContext().getStatus());
  }

  @Test
  public void testRunActionWithPre() {
    Symbol actor = Factory.createSymbol("self:agent");
    Predicate goalState = Factory.createPredicate("located(ball)");
    goal = new Goal(actor, goalState);

    instance = aiForGoal(goal);
    instance.call();

    assertTrue(sm.holds(goalState).getValue());
    
    // above here is same as testRunsubl
    // now we see if the next action preconditions pass

    goalState = Factory.createPredicate("movedTo(ball)");
    goal = new Goal(actor, goalState);
    
    instance = aiForGoal(goal);
    instance.call();

    assertTrue(sm.holds(goalState).getValue());
  }

  @Test
  public void testNoActionSelected() {
    goal = new Goal(actor, Factory.createPredicate("locatd(ball)"));  // Intentional misspelling
    instance = aiForGoal(goal);
    instance.call(); // run goal
    Context goalContext = instance.getRoot();
    Context selectedActionContext = goalContext.getChildContexts().get(0);
    assertTrue(selectedActionContext instanceof FailureContext);
  }

  /**
   * TODO: this doesn't actually test an observation...
   */
  @Test
  public void testObservationExecution() {
    log.debug("testObservationExecution");
    
    Predicate p = Factory.createPredicate("located(home, ball)");
    goal = new Goal(actor, p);

    instance = aiForGoal(goal);
    
    log.debug("starting instance...");
    instance.call();

    assertTrue(sm.holds(goal.getPredicate()).getValue());
  }

  private ActionInterpreter aiForGoal(Goal goal) {
    return ActionInterpreter.createInterpreterFromGoal(goal, root, sm, ExecutionType.ACT);
  }

}
