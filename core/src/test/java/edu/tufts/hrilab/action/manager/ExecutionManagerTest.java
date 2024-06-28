/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.manager;

import edu.tufts.hrilab.action.*;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.execution.ExecutionType;
import edu.tufts.hrilab.action.execution.RootContext;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.goal.PriorityTier;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Predicate;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.junit.AfterClass;
import org.junit.Test;
import static org.junit.Assert.*;
import org.junit.BeforeClass;

import java.util.ArrayList;
import java.util.HashSet;

/**
 *
 * @author willie
 */
public class ExecutionManagerTest {

  private static ExecutionManager em;
  private static Logger log = LoggerFactory.getLogger(ExecutionManagerTest.class);
  
  @BeforeClass
  public static void setUpClass() {
    HashSet<Predicate> initFacts = new HashSet<>();
    initFacts.add(Factory.createPredicate("diarcAgent(self)"));
    initFacts.add(Factory.createPredicate("diarcAgent(team1)"));
    initFacts.add(Factory.createPredicate("diarcAgent(agent1)"));
    initFacts.add(Factory.createPredicate("diarcAgent(agent2)"));
    initFacts.add(Factory.createPredicate("diarcAgent(agent3)"));
    initFacts.add(Factory.createPredicate("object(self,agent)"));
    initFacts.add(Factory.createPredicate("object(team1,agent)"));
    initFacts.add(Factory.createPredicate("object(agent1,agent)"));
    initFacts.add(Factory.createPredicate("object(agent2,agent)"));
    initFacts.add(Factory.createPredicate("object(agent3,agent)"));
    initFacts.add(Factory.createPredicate("team(self)"));
    initFacts.add(Factory.createPredicate("team(team1)"));
    initFacts.add(Factory.createPredicate("memberOf(self,self)"));
    initFacts.add(Factory.createPredicate("memberOf(team1,self)"));
    initFacts.add(Factory.createPredicate("memberOf(agent1,self)"));
    initFacts.add(Factory.createPredicate("memberOf(agent2,self)"));
    initFacts.add(Factory.createPredicate("memberOf(agent3,self)"));
    initFacts.add(Factory.createPredicate("memberOf(team1,team1)"));
    initFacts.add(Factory.createPredicate("memberOf(agent1,team1)"));
    initFacts.add(Factory.createPredicate("memberOf(agent2,team1)"));
    StateMachine sm = StateMachine.createTestStateMachine(initFacts);
    RootContext rootContext = new RootContext(new ActionConstraints(), sm);
    em = ExecutionManager.createInstance(ExecutionManager.class, sm, rootContext, new ArrayList<>());

//    ActionResourceLock.motionLock = new ActionResourceLockLinear("motionLock");

    ActionDBEntry.Builder lookforBuilder = new ActionDBEntry.Builder("lookfor");
    lookforBuilder.addRole(new ActionBinding.Builder("?loc", Symbol.class).build());
    lookforBuilder.addRole(new ActionBinding.Builder("?obj", Symbol.class).build());
//    lookforBuilder.addResourceLock("motionLock");
    lookforBuilder.setTimeout(1L);
    lookforBuilder.setCost("0.23");
    Predicate pc = Factory.createPredicate("located(?obj)");
    Effect ef = new Effect(pc, EffectType.ALWAYS);
    lookforBuilder.addEffect(ef);
    lookforBuilder.build(true);
    
    ActionDBEntry.Builder doneBuilder = new ActionDBEntry.Builder("testsecond");
    doneBuilder.addRole(new ActionBinding.Builder("?partner", Symbol.class).build());
    doneBuilder.addRole(new ActionBinding.Builder("!value", Double.class).setIsLocal(true).build());
    doneBuilder.addRole(new ActionBinding.Builder("!theta", Double.class).setIsLocal(true).build());
    doneBuilder.addRole(new ActionBinding.Builder("!phi", Double.class).setIsLocal(true).build());
//    doneBuilder.addResourceLock("motionLock");
    doneBuilder.setCost("0.5");
    Predicate pc2 = Factory.createPredicate("done(?actor, ?partner)");
    Effect ef2 = new Effect(pc2, EffectType.ALWAYS);
    doneBuilder.addEffect(ef2);
    doneBuilder.build(true);

    //TODO: Update actions to use new resources when integrated.

    ActionDBEntry.Builder driveBuilder = new ActionDBEntry.Builder("drive");
    //driveBuilder.addResourceLock("driveLock");
    driveBuilder.build(true);

    ActionDBEntry.Builder lookBuilder = new ActionDBEntry.Builder("look");
    //lookBuilder.addResourceLock("visionLock");
    lookBuilder.build(true);

    ActionDBEntry.Builder speakBuilder = new ActionDBEntry.Builder("speak");
    //speakBuilder.addResourceLock("speakerLock");
    speakBuilder.build(true);

    ActionDBEntry.Builder grabBuilder = new ActionDBEntry.Builder("grab");
    //grabBuilder.addResourceLock("armLock");
    grabBuilder.build(true);

    ActionDBEntry.Builder updateSettingsBuilder = new ActionDBEntry.Builder("updateSettings");
    updateSettingsBuilder.build(true);
  }

  @AfterClass
  public static void destroy(){
    em.shutdown();
  }

  @Test
  public void testSubmitActionGoal() {
    Predicate goalPred = Factory.createPredicate("lookfor(self:agent,kitchen,ball)");
    Goal goal = em.submitGoal(new Goal(goalPred), ExecutionType.ACT);
    assertNotNull(goal);
  }

  @Test
  public void testSubmitStateGoal() {
    Predicate goalPred = Factory.createPredicate("goal(self:agent,located(ball))");
    Goal goal = em.submitGoal(new Goal(goalPred), ExecutionType.ACT);
    assertNotNull(goal);
  }

    /**
   * Test of submitGoal method where no action matches the provided goal.
   */
  @Test
  public void testSubmitGoalNoActionFound() {
    log.warn("[testSubmitGoalNoActionFound] start");
    Predicate goalPred = Factory.createPredicate("goal(self:agent,locatd(ball))"); // intentionally misspelled
    Goal goal = em.submitGoal(new Goal(goalPred), ExecutionType.ACT);
    em.joinOnGoal(goal.getId());
    assertTrue(goal.getStatus() == GoalStatus.FAILED);
    assertTrue(em.getActionStatus(goal.getId()) == ActionStatus.FAIL_NOTFOUND);
    log.warn("[testSubmitGoalNoActionFound] end");
  }
  
  @Test
  public void testSubmitDone() {
    log.warn("[testSubmitDone] start");
    Predicate goalPred = Factory.createPredicate("goal(self:agent,done(ball, ball))");
    Goal goal = em.submitGoal(new Goal(goalPred), ExecutionType.ACT);
    em.joinOnGoal(goal.getId());
    long gid = goal.getId();
    assertTrue(gid >= 0);
    assertTrue(goal.getStartTime() > 0);
    log.warn("[testSubmitDone] end");
  }

  //@Test
  //public void testSwapAgentSim() {
  //  BasicGoalManager gm = new BasicGoalManager("self", new ActionConstraints(), new MockActionImplementer());
  //  Database.loadDatabaseFromFile("com/action/asl/manipulation.asl");
  //  Predicate goalPred = Factory.createPredicate("did(graspObject(self, ball))");
  //  Goal goal_sim = em.submitGoal(goalPred, ExecutionType.SIMULATE_ACT);
  //  assertTrue(true);
  //  //assertNotNull(goal_sim);
  //}

  //@Test
  //// TODO: 4/26/17 set up ag.learning
  //public void testSubmitGoalLearning() {
  //  BasicGoalManager gm = new BasicGoalManager("self", new ActionConstraints(), new MockActionImplementer());

  //  Term inPredicate = new Term("actionLearningStart");
  //  List<Map<Variable,Symbol>> bindings = new ArrayList();
  //  Map<Variable,Symbol> vals = new HashMap<Variable,Symbol>();
  //  vals.put(new Variable("comm"), new Symbol("commX"));
  //  vals.put(new Variable("actor"), new Symbol("self"));
  //  vals.put(new Variable("newAction"), new Symbol("newAction(bindings)"));
  //  bindings.add(vals);
  //  em.evaluatePredicate(inPredicate,bindings);

  //  Predicate goalPred = Factory.createPredicate("located(ball)");
  //  Goal goal = em.submitGoal(goalPred);
  //  assertNotNull(goal);
  //  Goal goal_sim = em.submitGoal(goalPred, ExecutionType.SIMULATE);
  //  assertNotNull(goal_sim);
  //  Goal goal_sim_act = em.submitGoal(goalPred, ExecutionType.SIMULATE_ACT);
  //  assertNotNull(goal_sim_act);
  //}
  
  /**
   * Test of setConstraints method, of class BasicGoalManager.
   */
  @Test
  public void testSetConstraints() {
  }

  /**
   * Test of actionComplete method, of class BasicGoalManager.
   */
  @Test
  public void testActionComplete() {
  }

  private Goal submitGoalAndWait(Predicate goalPred) {
    Goal g = em.submitGoal(new Goal(goalPred), ExecutionType.ACT);
    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      log.error(e.getMessage());
    }
    return g;
  }

  private Goal submitGoalAndWait(Predicate goalPred, ExecutionType executionType, long priority, PriorityTier priorityTier) {
    Goal g = new Goal(goalPred);
    g.setPriority(priority);
    g.setPriorityTier(priorityTier);
    em.submitGoal(g, executionType);
    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      log.error(e.getMessage());
    }
    return g;
  }

  private void cancelAllGoals() {
    for (Goal g: em.getPendingGoals()) {
      em.cancelGoal(g.getId());
    }
    for (Goal g: em.getActiveGoals()) {
      em.cancelGoal(g.getId());
    }
  }

  /**
   * Tests that resources are correctly freed on goal completion
   */
  @Test
  public void testResourcesFreedOnCompletion() {
    Predicate goalPred = Factory.createPredicate("freeze", "agent1:agent", "agent1:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);
    submitGoalAndWait(Factory.createPredicate("endFreeze", "agent1:agent", "agent1:agent"));
    em.joinOnGoal(freezeGoal.getId());
    assertSame(freezeGoal.getStatus(), GoalStatus.SUCCEEDED);

    goalPred = Factory.createPredicate("freeze", "agent1:agent", "agent1:agent");
    freezeGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);
    submitGoalAndWait(Factory.createPredicate("endFreeze", "agent1:agent", "agent1:agent"));
    em.joinOnGoal(freezeGoal.getId());
    assertSame(freezeGoal.getStatus(), GoalStatus.SUCCEEDED);

    cancelAllGoals();
  }

  /**
   * Tests canceling a goal correctly frees up resources
   */
  @Test
  public void testCancelGoalReleasesResources() {
    Predicate goalPred = Factory.createPredicate("freeze", "self:agent", "self:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);
    em.cancelGoal(freezeGoal.getId());
    em.joinOnGoal(freezeGoal.getId());
    assertSame(freezeGoal.getStatus(), GoalStatus.CANCELED);

    freezeGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);
    submitGoalAndWait(Factory.createPredicate("endFreeze", "self:agent", "self:agent"));
    em.joinOnGoal(freezeGoal.getId());
    assertSame(freezeGoal.getStatus(), GoalStatus.SUCCEEDED);

    cancelAllGoals();
  }

  /**
   * Tests submitting multiple goals for the same AgentTeam without resource
   * conflicts. Both goals should execute (TODO: update when resources are)
   */
  @Test
  public void testSubmitGoalsNonConflictingResources() {
    Predicate goalPred = Factory.createPredicate("freeze", "agent2:agent", "agent2:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    goalPred = Factory.createPredicate("freeze", "agent1:agent", "agent1:agent");
    Goal freeze2Goal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);
    assertSame(freeze2Goal.getStatus(), GoalStatus.ACTIVE);
    submitGoalAndWait(Factory.createPredicate("endFreeze", "agent1:agent", "agent1:agent"));
    submitGoalAndWait(Factory.createPredicate("endFreeze", "agent2:agent", "agent2:agent"));
    em.joinOnGoal(freezeGoal.getId());
    em.joinOnGoal(freeze2Goal.getId());
    assertSame(freezeGoal.getStatus(), GoalStatus.SUCCEEDED);
    assertSame(freeze2Goal.getStatus(), GoalStatus.SUCCEEDED);

    cancelAllGoals();
  }

  //TODO: Need notion of conflicting resources for base case for the below tests
  ///**
  // * Tests submitting goals with equal priority and conflicting resources. The
  // * second submitted goal should fail with correct conditions
  // */
  //@Test
  //public void testSubmitGoalsConflictingResourcesEqualPriority() {
  //  Predicate goalPred = Factory.createPredicate("freeze", "self:agent", "self:agent");
  //  Goal freezeGoal = submitGoalAndWait(goalPred);
  //  assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);
  //  goalPred = Factory.createPredicate("drive", "self:agent");
  //  Goal driveGoal = submitGoalAndWait(goalPred);
  //  em.joinOnGoal(driveGoal.getId());
  //  assertSame(driveGoal.getStatus(), GoalStatus.FAILED);
  //  submitGoalAndWait(Factory.createPredicate("endFreeze", "self:agent", "self:agent"));
  //  em.joinOnGoal(freezeGoal.getId());
  //  assertSame(freezeGoal.getStatus(), GoalStatus.SUCCEEDED);

  //  cancelAllGoals();
  //}

  ///**
  // * Tests handling of a goal being submitted which has higher priority than
  // * the currently active goal. The current goal should have its execution
  // * superseded and pushed back to pending. Upon completion of the superseder,
  // * the original goal should be returned to active.
  // */
  //@Test
  //public void testSupersedeGoal() {
  //  Predicate goalPred = Factory.createPredicate("freeze", "team1:agent", "team1:agent");
  //  Goal freeze1Goal = submitGoalAndWait(goalPred, ExecutionType.ACT, 1L, PriorityTier.LOW);
  //  assertSame(freeze1Goal.getStatus(), GoalStatus.ACTIVE);

  //  goalPred = Factory.createPredicate("freeze", "self:agent", "self:agent");
  //  Goal freeze2Goal = submitGoalAndWait(goalPred);
  //  assertSame(freeze2Goal.getStatus(), GoalStatus.ACTIVE);
  //  assertSame(freeze1Goal.getStatus(), GoalStatus.SUSPENDED);

  //  Goal endFreezeGoal = submitGoalAndWait(Factory.createPredicate("endFreeze", "self:agent", "self:agent"));
  //  em.joinOnGoal(endFreezeGoal.getId());
  //  em.joinOnGoal(freeze2Goal.getId());
  //  assertSame(freeze2Goal.getStatus(), GoalStatus.SUCCEEDED);
  //  assertSame(freeze1Goal.getStatus(), GoalStatus.ACTIVE);

  //  endFreezeGoal = submitGoalAndWait(Factory.createPredicate("endFreeze", "team1:agent", "team1:agent"));
  //  em.joinOnGoal(endFreezeGoal.getId());
  //  em.joinOnGoal(freeze1Goal.getId());
  //  assertSame(freeze1Goal.getStatus(), GoalStatus.SUCCEEDED);

  //  cancelAllGoals();
  //}

  ///**
  // * Tests that canceling a superseded goal from the pending collection is
  // * handled properly (is different than generally canceling a pending goal
  // * because it will have a partially executed ActionInterpreter and SUSPENDED
  // * status)
  // */
  //@Test
  //public void testCancelSupersededGoal() {
  //  Predicate goalPred = Factory.createPredicate("freeze", "team1:agent", "team1:agent");
  //  Goal freeze1Goal = submitGoalAndWait(goalPred, ExecutionType.ACT, 1L, PriorityTier.LOW);
  //  assertSame(freeze1Goal.getStatus(), GoalStatus.ACTIVE);

  //  goalPred = Factory.createPredicate("freeze", "self:agent", "self:agent");
  //  Goal freeze2Goal = submitGoalAndWait(goalPred);
  //  assertSame(freeze2Goal.getStatus(), GoalStatus.ACTIVE);
  //  assertSame(freeze1Goal.getStatus(), GoalStatus.SUSPENDED);

  //  em.cancelGoal(freeze1Goal.getId());
  //  em.joinOnGoal(freeze1Goal.getId());
  //  assertSame(freeze1Goal.getStatus(), GoalStatus.CANCELED);
  //  //TODO: make sure goal is in pastGoals?

  //  cancelAllGoals();
  //}

  /**
   * Tests action learning pipeline is not blocked by the Queue EM
   */
  @Test
  public void testActionLearning() {
    Predicate goalPred = Factory.createPredicate("updateActionLearning", "self:agent", "testAction()", "start");
    Goal learningGoal = submitGoalAndWait(goalPred);
    assertSame(learningGoal.getStatus(), GoalStatus.SUCCEEDED);

    goalPred = Factory.createPredicate("drive", "self:agent");
    Goal driveGoal = submitGoalAndWait(goalPred);
    goalPred = Factory.createPredicate("look", "self:agent");
    Goal lookGoal = submitGoalAndWait(goalPred);
    em.joinOnGoal(driveGoal.getId());
    assertSame(driveGoal.getStatus(), GoalStatus.SUCCEEDED);
    em.joinOnGoal(lookGoal.getId());
    assertSame(lookGoal.getStatus(), GoalStatus.SUCCEEDED);

    goalPred = Factory.createPredicate("updateActionLearning", "self:agent", "testAction()", "end");
    Goal endLearningGoal = submitGoalAndWait(goalPred);
    em.joinOnGoal(endLearningGoal.getId());
    em.joinOnGoal(learningGoal.getId());
    assertSame(learningGoal.getStatus(), GoalStatus.SUCCEEDED);

    goalPred = Factory.createPredicate("testAction", "self:agent");
    Goal newActionGoal = submitGoalAndWait(goalPred);
    em.joinOnGoal(newActionGoal.getId());
    assertSame(newActionGoal.getStatus(), GoalStatus.SUCCEEDED);
    assertSame(newActionGoal.getCurrentContext().getCommand(), "look");

    cancelAllGoals();
  }

  //ExecuteWhileLearning calls a method on the GMImpl, needs to be instantiated to work
//  @Test
//  public void testExecuteWhileLearning() {
//    Predicate goalPred = Factory.createPredicate("updateActionLearning", "self:agent", "testAction()", "start");
//    Goal learningGoal = submitGoalAndWait(goalPred);
//    assertSame(learningGoal.getStatus(), GoalStatus.SUCCEEDED);
//
//    goalPred = Factory.createPredicate("changeLearningExecution", "self:agent", "execute");
//    Goal executeWhileLearningGoal = submitGoalAndWait(goalPred);
//    assertSame(executeWhileLearningGoal.getStatus(), GoalStatus.SUCCEEDED);
//
//    goalPred = Factory.createPredicate("drive", "self:agent");
//    Goal driveGoal = submitGoalAndWait(goalPred);
//    goalPred = Factory.createPredicate("look", "self:agent");
//    Goal lookGoal = submitGoalAndWait(goalPred);
//    em.joinOnGoal(driveGoal.getId());
//    assertSame(driveGoal.getStatus(), GoalStatus.SUCCEEDED);
//    em.joinOnGoal(lookGoal.getId());
//    assertSame(lookGoal.getStatus(), GoalStatus.SUCCEEDED);
//
//    goalPred = Factory.createPredicate("changeLearningExecution", "self:agent", "stop");
//    Goal endExecuteWhileLearningGoal = submitGoalAndWait(goalPred);
//    assertSame(endExecuteWhileLearningGoal.getStatus(), GoalStatus.SUCCEEDED);
//
//    goalPred = Factory.createPredicate("updateActionLearning", "self:agent", "testAction()", "end");
//    Goal endLearningGoal = submitGoalAndWait(goalPred);
//    em.joinOnGoal(endLearningGoal.getId());
//    em.joinOnGoal(learningGoal.getId());
//    assertSame(learningGoal.getStatus(), GoalStatus.SUCCEEDED);
//
//    goalPred = Factory.createPredicate("testAction", "self:agent");
//    Goal newActionGoal = submitGoalAndWait(goalPred);
//    em.joinOnGoal(newActionGoal.getId());
//    assertSame(newActionGoal.getStatus(), GoalStatus.SUCCEEDED);
//    assertSame(newActionGoal.getCurrentContext().getCommand(), "look");
//
//    cancelAllGoals();
//  }

}

