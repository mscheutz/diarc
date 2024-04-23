/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.execution;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.ActionBuilderRepo;
import edu.tufts.hrilab.action.ActionConstraints;
import edu.tufts.hrilab.action.ActionInterpreter;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.lock.ActionResourceLock;
import edu.tufts.hrilab.action.lock.ActionResourceLockLinear;
import edu.tufts.hrilab.action.justification.Justification;

import java.util.HashSet;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import org.junit.*;

import static org.junit.Assert.*;

/**
 *
 * @author willie
 */
public class StepExecutionTest {

  private RootContext root;
  private ActionInterpreter instance = null;
  private Goal goal;
  private StateMachine sm;
  private StepExecution stepExecution;
  private Symbol actor = Factory.createSymbol("self");

  public StepExecutionTest() {
    //FIXME: step execution constructor
    stepExecution = new StepExecution(null, null);
  }

  @BeforeClass
  public static void setUpClass() {
    ActionResourceLock.motionLock = new ActionResourceLockLinear("motionLock");
    ActionBuilderRepo.buildLookforAction();
    ActionBuilderRepo.buildMoveToObjectAction();
    //ActionBuilderRepo.buildGraspObjectAction();
  }

  @AfterClass
  public static void tearDownClass() {
    Database.destroyInstance();
  }

  @Before
  public void resetState() {
    HashSet<Predicate> initFacts = new HashSet<>();
    initFacts.add(Factory.createPredicate("diarcAgent(self)"));
    initFacts.add(Factory.createPredicate("object(self,agent)"));
    initFacts.add(Factory.createPredicate("team(self)"));
    initFacts.add(Factory.createPredicate("memberOf(self,self)"));
    sm= StateMachine.createTestStateMachine(initFacts);
    root = new RootContext(new ActionConstraints(),sm);
  }

  @After
  public void tearDown() {
    sm.shutdown();
  }

  @Test
  public void testCanDoStep() throws NoSuchMethodException {
    System.out.println("testing canDoStep");
    
    String goalString = "located(ball)";
    goal = new Goal(actor, Factory.createPredicate(goalString));

    instance = aiForGoal(goal);
    Justification just = stepExecution.canDoStep(instance.getRoot());
    assertTrue(just.getValue());
  }

  @Test
  public void testDoStep() throws NoSuchMethodException {
    System.out.println("testing doStep");

    String goalString = "located(ball)";
    goal = new Goal(actor, Factory.createPredicate(goalString));

    instance = aiForGoal(goal);
    Justification just = stepExecution.canDoStep(instance.getRoot());
    stepExecution.doStep(instance.getRoot());
    assertTrue(true);
  }

  @Test
  public void testFinishStep() throws NoSuchMethodException {
    String goalString = "located(ball)";
    goal = new Goal(actor, Factory.createPredicate(goalString));

    instance = aiForGoal(goal);

    // do action selection for goal (done in doStep for goal)
    Context goalContext = instance.getRoot();
    stepExecution.canDoStep(goalContext);
    stepExecution.doStep(goalContext);

    // get the action for the goal and execute it
    Context actionContext = goalContext.getNextStep();
    stepExecution.canDoStep(actionContext);
    stepExecution.doStep(actionContext);

    // check that the action context was finished successfully
    Justification actionJustification = stepExecution.finishStep(actionContext);
    assertTrue(actionJustification.getValue());

    // finally, check the goal context was finished correctly
    Justification goalJustification = stepExecution.finishStep(goalContext);
    assertTrue(goalJustification.getValue());
  }

  @Test
  public void testCanDoStepPreFail() throws NoSuchMethodException {
    String goalString = "movedTo(ball)";
    goal = new Goal(actor, Factory.createPredicate(goalString));

    instance = aiForGoal(goal);
    Context goalContext = instance.getRoot();
    goalContext.setupNextStep(); // do action selection step
    Context actionContext = goalContext.getChildContexts().get(0);
    Justification just = stepExecution.canDoStep(actionContext);
    assertEquals(ActionStatus.FAIL_PRECONDITIONS, actionContext.getStatus());
    assertFalse(just.getValue());
    assertFalse(actionContext.getJustification().getFailureReason().isEmpty());
  }

  private ActionInterpreter aiForGoal(Goal goal) {
    return ActionInterpreter.createInterpreterFromGoal(goal,root, sm, ExecutionType.ACT);
  }

  @TRADEService
  public void foo(String agentname, String loc, String obj) {}

}
