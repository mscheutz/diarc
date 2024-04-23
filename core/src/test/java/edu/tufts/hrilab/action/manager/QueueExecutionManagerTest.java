/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.manager;

import edu.tufts.hrilab.action.ActionConstraints;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.execution.ExecutionType;
import edu.tufts.hrilab.action.execution.RootContext;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.goal.PendingGoal;
import edu.tufts.hrilab.action.goal.PriorityTier;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import org.junit.AfterClass;
import org.junit.BeforeClass;
import org.junit.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.HashSet;

import static org.junit.Assert.*;

public class QueueExecutionManagerTest {

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
    try {
      em = ExecutionManager.createInstance((Class<ExecutionManager>)Class.forName("edu.tufts.hrilab.action.manager.QueueExecutionManager"), sm, rootContext, "queueExecutionManagerTest.json",new ArrayList<>());
    } catch (ClassNotFoundException e) {
      throw new RuntimeException(e);
    }

    //TODO: Update actions to use new resources when integrated.
//    ActionResourceLock.motionLock = new ActionResourceLockLinear("motionLock");

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

  private Goal submitGoalAndWait(Predicate goalPred) {
    Goal g = em.submitGoal(goalPred);
    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      log.error(e.getMessage());
    }
    return g;
  }

  private Goal submitGoalAndWait(Predicate goalPred, ExecutionType executionType, long priority, PriorityTier priorityTier) {
    Goal g = em.submitGoal(goalPred, executionType, priority, priorityTier);
    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      log.error(e.getMessage());
    }
    return g;
  }

  private void cancelAllGoals() {
    log.info("[cancelAllGoals] cleaning up");
    for (PendingGoal pg: em.getPendingGoals()) {
      em.cancelGoal(pg.getGoal().getId());
    }
    for (Goal g: em.getActiveGoals()) {
      em.cancelGoal(g.getId());
    }
  }

  /**
   * Tests base queueing behavior. Goals of equal explicit priority should be
   * queued and subsequently executed in the order they are submitted once unblocked
   */
  @Test
  public void testQueueGoalsEqualPriority() {
    Predicate goalPred = Factory.createPredicate("freeze", "self:agent", "self:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);
    goalPred = Factory.createPredicate("drive", "self:agent");
    Goal driveGoal = submitGoalAndWait(goalPred);
    goalPred = Factory.createPredicate("look", "self:agent");
    Goal lookGoal = submitGoalAndWait(goalPred);
    assertSame(driveGoal.getStatus(), GoalStatus.PENDING);
    assertSame(lookGoal.getStatus(), GoalStatus.PENDING);
    submitGoalAndWait(Factory.createPredicate("endFreeze", "self:agent", "self:agent"));
    em.joinOnGoal(freezeGoal.getId());
    assertSame(freezeGoal.getStatus(), GoalStatus.SUCCEEDED);
    em.joinOnGoal(driveGoal.getId());
    assertSame(driveGoal.getStatus(), GoalStatus.SUCCEEDED);
    em.joinOnGoal(lookGoal.getId());
    assertSame(lookGoal.getStatus(), GoalStatus.SUCCEEDED);
    assertTrue(driveGoal.getStartTime() <= lookGoal.getStartTime());

    cancelAllGoals();
  }

  /**
   * Tests handling of priority tier when queueing goals. The higher priority
   * goal should be inserted in front of the lower one in the queue despite
   * being submitted second.
   */
  @Test
  public void testQueueGoalsOutOfOrderPriorityTier() {
    Predicate goalPred = Factory.createPredicate("freeze", "self:agent", "self:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);
    goalPred = Factory.createPredicate("drive", "self:agent");
    Goal driveGoal = submitGoalAndWait(goalPred, ExecutionType.ACT, 1L, PriorityTier.LOW);
    goalPred = Factory.createPredicate("look", "self:agent");
    Goal lookGoal = submitGoalAndWait(goalPred);
    assertSame(driveGoal.getStatus(), GoalStatus.PENDING);
    assertSame(lookGoal.getStatus(), GoalStatus.PENDING);
    submitGoalAndWait(Factory.createPredicate("endFreeze", "self:agent", "self:agent"));
    em.joinOnGoal(freezeGoal.getId());
    assertSame(freezeGoal.getStatus(), GoalStatus.SUCCEEDED);
    em.joinOnGoal(lookGoal.getId());
    assertSame(lookGoal.getStatus(), GoalStatus.SUCCEEDED);
    em.joinOnGoal(driveGoal.getId());
    assertSame(driveGoal.getStatus(), GoalStatus.SUCCEEDED);
    assertTrue(lookGoal.getStartTime() <= driveGoal.getStartTime());

    cancelAllGoals();
  }

  /**
   * Tests handling of priority value when queueing goals. The higher priority
   * goal should be inserted in front of the lower one in the queue despite
   * being submitted second.
   */
  @Test
  public void testQueueGoalsOutOfOrderPriority() {
    Predicate goalPred = Factory.createPredicate("freeze", "self:agent", "self:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);
    goalPred = Factory.createPredicate("drive", "self:agent");
    Goal driveGoal = submitGoalAndWait(goalPred, ExecutionType.ACT, 0L, PriorityTier.NORMAL);
    goalPred = Factory.createPredicate("look", "self:agent");
    Goal lookGoal = submitGoalAndWait(goalPred);
    assertSame(driveGoal.getStatus(), GoalStatus.PENDING);
    assertSame(lookGoal.getStatus(), GoalStatus.PENDING);
    submitGoalAndWait(Factory.createPredicate("endFreeze", "self:agent", "self:agent"));
    em.joinOnGoal(freezeGoal.getId());
    assertSame(freezeGoal.getStatus(), GoalStatus.SUCCEEDED);
    em.joinOnGoal(lookGoal.getId());
    assertSame(lookGoal.getStatus(), GoalStatus.SUCCEEDED);
    em.joinOnGoal(driveGoal.getId());
    assertSame(driveGoal.getStatus(), GoalStatus.SUCCEEDED);
    assertTrue(lookGoal.getStartTime() <= driveGoal.getStartTime());

    cancelAllGoals();
  }

  /**
   * Tests handling of goals which skip the queue
   */
  @Test
  public void testSkipQueueGoal() {
    Predicate goalPred = Factory.createPredicate("freeze", "self:agent", "self:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);
    goalPred = Factory.createPredicate("updateSettings", "self:agent");
    Goal updateSettingsGoal = submitGoalAndWait(goalPred);
    em.joinOnGoal(updateSettingsGoal.getId());
    assertSame(updateSettingsGoal.getStatus(), GoalStatus.SUCCEEDED);

    cancelAllGoals();
  }

  //Some of the below test may change when resource definitions are more granular
  /**
   * Tests handling of multiple goals within a branch of the group hierarchy.
   * Current resource assumptions dictate that goals for a team and a member of
   * that team conflict and cannot coexist.
   */
  @Test
  public void testSubmitGoalsForTeamAndMember() {
    Predicate goalPred = Factory.createPredicate("freeze", "self:agent", "self:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);
    goalPred = Factory.createPredicate("freeze", "agent3:agent", "agent3:agent");
    freezeGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.PENDING);

    cancelAllGoals();
  }

  /**
   * Tests handling of goals for different members of the same team.
   */
  @Test
  public void testSubmitGoalsForSiblings() {
    Predicate goalPred = Factory.createPredicate("freeze", "agent1:agent", "agent1:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);
    goalPred = Factory.createPredicate("freeze", "agent2:agent", "agent2:agent");
    freezeGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);

    cancelAllGoals();
  }

  /**
   * Tests handling of a goal being submitted which has higher priority than
   * the currently active goal. The current goal should have its execution
   * superseded and pushed back to pending. Upon completion of the superseder,
   * the original goal should be returned to active.
   */
  @Test
  public void testSupersedeGoal() {
    Predicate goalPred = Factory.createPredicate("freeze", "team1:agent", "team1:agent");
    Goal freeze1Goal = submitGoalAndWait(goalPred, ExecutionType.ACT, 1L, PriorityTier.LOW);
    assertSame(freeze1Goal.getStatus(), GoalStatus.ACTIVE);

    goalPred = Factory.createPredicate("freeze", "self:agent");
    Goal freeze2Goal = submitGoalAndWait(goalPred);
    assertSame(freeze2Goal.getStatus(), GoalStatus.ACTIVE);
    assertSame(freeze1Goal.getStatus(), GoalStatus.SUSPENDED);

    Goal endFreezeGoal = submitGoalAndWait(Factory.createPredicate("endFreeze", "self:agent", "self:agent"));
    em.joinOnGoal(endFreezeGoal.getId());
    em.joinOnGoal(freeze2Goal.getId());
    assertSame(freeze2Goal.getStatus(), GoalStatus.SUCCEEDED);
    assertSame(freeze1Goal.getStatus(), GoalStatus.ACTIVE);

    endFreezeGoal = submitGoalAndWait(Factory.createPredicate("endFreeze", "team1:agent", "team1:agent"));
    em.joinOnGoal(endFreezeGoal.getId());
    em.joinOnGoal(freeze1Goal.getId());
    assertSame(freeze1Goal.getStatus(), GoalStatus.SUCCEEDED);

    cancelAllGoals();
  }

  //TODO: With superseding of tasks working, we could actually do away with
  //  this assumption and tasks wouldn't be starved
  /**
   * Tests handling of resource conflicts existing for a newly submitted goal
   * only among higher priority pending goals, and not any active goals.
   * E.g. a goal for a sibling team member exists blocking a higher priority
   * team wide goal upon submission time. The submitted goal could be immediately
   * executed, but we don't want to allow the possibility for certain tasks to
   * be starved
   */
  @Test
  public void testQueueResourceConflictBlocks() {
    Predicate goalPred = Factory.createPredicate("freeze", "agent1:agent", "agent1:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);

    goalPred = Factory.createPredicate("grab", "team1:agent");
    Goal grabGoal = submitGoalAndWait(goalPred);
    assertSame(grabGoal.getStatus(), GoalStatus.PENDING);

    goalPred = Factory.createPredicate("drive", "agent2:agent");
    Goal driveGoal = submitGoalAndWait(goalPred);
    assertSame(driveGoal.getStatus(), GoalStatus.PENDING);

    cancelAllGoals();
  }

  //TODO: this goal could be removed if the above behavior is overturned
  /**
   * Tests that canceling a pending goal could potentially correctly allow a
   * lower priority pending goal to be executed immediately
   */
  @Test
  public void testCancelQueueResourceConflict() {
    Predicate goalPred = Factory.createPredicate("freeze", "agent1:agent", "agent1:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);

    goalPred = Factory.createPredicate("grab", "team1:agent");
    Goal grabGoal = submitGoalAndWait(goalPred);
    assertSame(grabGoal.getStatus(), GoalStatus.PENDING);

    goalPred = Factory.createPredicate("drive", "agent2:agent");
    Goal driveGoal = submitGoalAndWait(goalPred);
    assertSame(driveGoal.getStatus(), GoalStatus.PENDING);

    em.cancelGoal(grabGoal.getId());
    assertSame(grabGoal.getStatus(), GoalStatus.CANCELED);
    em.joinOnGoal(driveGoal.getId());
    assertSame(driveGoal.getStatus(), GoalStatus.SUCCEEDED);

    cancelAllGoals();
  }

  /**
   * Tests that canceling a goal from the pending collection is
   * handled properly
   */
  @Test
  public void testCancelPendingGoal() {
    Predicate goalPred = Factory.createPredicate("freeze", "team1:agent", "team1:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);
    goalPred = Factory.createPredicate("drive", "self:agent");
    Goal driveGoal = submitGoalAndWait(goalPred);
    goalPred = Factory.createPredicate("look", "self:agent");
    Goal lookGoal = submitGoalAndWait(goalPred);
    assertSame(driveGoal.getStatus(), GoalStatus.PENDING);
    assertSame(lookGoal.getStatus(), GoalStatus.PENDING);
    em.cancelGoal(driveGoal.getId());
    em.joinOnGoal(driveGoal.getId());
    assertSame(driveGoal.getStatus(), GoalStatus.CANCELED);
    submitGoalAndWait(Factory.createPredicate("endFreeze", "team1:agent", "team1:agent"));
    em.joinOnGoal(freezeGoal.getId());
    assertSame(freezeGoal.getStatus(), GoalStatus.SUCCEEDED);
    em.joinOnGoal(lookGoal.getId());
    assertSame(lookGoal.getStatus(), GoalStatus.SUCCEEDED);
    assertEquals(-1L, driveGoal.getStartTime());

    cancelAllGoals();
  }

  /**
   * Tests that canceling a superseded goal from the pending collection is
   * handled properly (is different than generally canceling a pending goal
   * because it will have a partially executed ActionInterpreter and SUSPENDED
   * status)
   */
  @Test
  public void testCancelSupersededGoal() {
    Predicate goalPred = Factory.createPredicate("freeze", "team1:agent", "team1:agent");
    Goal freeze1Goal = submitGoalAndWait(goalPred, ExecutionType.ACT, 1L, PriorityTier.LOW);
    assertSame(freeze1Goal.getStatus(), GoalStatus.ACTIVE);

    goalPred = Factory.createPredicate("freeze", "self:agent", "self:agent");
    Goal freeze2Goal = submitGoalAndWait(goalPred);
    assertSame(freeze2Goal.getStatus(), GoalStatus.ACTIVE);
    assertSame(freeze1Goal.getStatus(), GoalStatus.SUSPENDED);

    em.cancelGoal(freeze1Goal.getId());
    em.joinOnGoal(freeze1Goal.getId());
    assertSame(freeze1Goal.getStatus(), GoalStatus.CANCELED);
    //TODO: make sure goal is in pastGoals?

    cancelAllGoals();
  }

  /**
   * Tests canceling freeze (As of now a behavior on the EM itself alongside its
   * interruption behavior) works properly
   */
  @Test
  public void testCancelFreeze() {
    Predicate goalPred = Factory.createPredicate("freeze", "self:agent", "self:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);
    goalPred = Factory.createPredicate("drive", "self:agent");
    Goal driveGoal = submitGoalAndWait(goalPred);
    goalPred = Factory.createPredicate("look", "self:agent");
    Goal lookGoal = submitGoalAndWait(goalPred);
    assertSame(driveGoal.getStatus(), GoalStatus.PENDING);
    assertSame(lookGoal.getStatus(), GoalStatus.PENDING);
    Goal cancelGoal = submitGoalAndWait(Factory.createPredicate("cancelCurrentGoal", "self:agent"));
    em.joinOnGoal(cancelGoal.getId());
    em.joinOnGoal(freezeGoal.getId());
    assertSame(freezeGoal.getStatus(), GoalStatus.CANCELED);
    em.joinOnGoal(driveGoal.getId());
    assertSame(driveGoal.getStatus(), GoalStatus.SUCCEEDED);
    em.joinOnGoal(lookGoal.getId());
    assertSame(lookGoal.getStatus(), GoalStatus.SUCCEEDED);
    assertTrue(driveGoal.getStartTime() <= lookGoal.getStartTime());

    cancelAllGoals();
  }

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
