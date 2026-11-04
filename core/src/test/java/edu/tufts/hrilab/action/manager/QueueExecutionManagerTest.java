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
import edu.tufts.hrilab.action.goal.PriorityTier;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.util.Util;
import org.junit.AfterClass;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import static org.junit.Assert.*;

public class QueueExecutionManagerTest {

  private static QueueExecutionManager em;
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
      em = (QueueExecutionManager) ExecutionManager.createInstance((Class<ExecutionManager>)Class.forName("edu.tufts.hrilab.action.manager.QueueExecutionManager"), sm, rootContext, new ArrayList<>());
    } catch (ClassNotFoundException e) {
      throw new RuntimeException(e);
    }

    ActionDBEntry.Builder driveBuilder = new ActionDBEntry.Builder("drive");
    driveBuilder.build(true);

    ActionDBEntry.Builder lookBuilder = new ActionDBEntry.Builder("look");
    lookBuilder.build(true);

    ActionDBEntry.Builder speakBuilder = new ActionDBEntry.Builder("speak");
    speakBuilder.build(true);

    ActionDBEntry.Builder grabBuilder = new ActionDBEntry.Builder("grab");
    grabBuilder.build(true);

    ActionDBEntry.Builder updateSettingsBuilder = new ActionDBEntry.Builder("updateSettings");
    updateSettingsBuilder.build(true);
  }

  @AfterClass
  public static void destroy(){
    em.shutdown();
  }

  private Goal submitGoalAndWait(Predicate goalPred) {
    PriorityTier priorityTier = PriorityTier.NORMAL;
    long priority = 1;
    if (goalPred.getName().equals("freeze")) {
      priorityTier = PriorityTier.URGENT;
      priority = 9999;
    } else if (goalPred.getName().equals("endFreeze")) {
      priorityTier = PriorityTier.SKIPPENDING;
    } else if (goalPred.getName().equals("updateSettings")) {
      priorityTier = PriorityTier.SKIPPENDING;
    } else if (goalPred.getName().equals("cancelSystemGoals")) {
      priorityTier = PriorityTier.SKIPPENDING;
    } else if (goalPred.getName().equals("suspendSystemGoals")) {
      priorityTier = PriorityTier.SKIPPENDING;
    } else if (goalPred.getName().equals("resumeSystemGoals")) {
      priorityTier = PriorityTier.SKIPPENDING;
    } else if (goalPred.getName().equals("cancelAllPendingGoals")) {
      priorityTier = PriorityTier.SKIPPENDING;
    } else if (goalPred.getName().equals("cancelAllActiveGoals")) {
      priorityTier = PriorityTier.SKIPPENDING;
    } else if (goalPred.getName().equals("cancelAllCurrentGoals")) {
      priorityTier = PriorityTier.SKIPPENDING;
    } else if (goalPred.getName().equals("endActionLearning")) {
      priorityTier = PriorityTier.SKIPPENDING;
    }
    Goal g = Goal.createGoal(goalPred);
    g.setPriorityTier(priorityTier);
    g.setPriority(priority);

    g = em.submitGoal(g);
    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      log.error(e.getMessage());
    }
    return g;
  }

  private Goal submitGoalAndWait(Predicate goalPred, ExecutionType executionType, long priority, PriorityTier priorityTier) {
    Goal g = Goal.createGoal(goalPred);
    g.setPriority(priority);
    g.setPriorityTier(priorityTier);
    g.setExecutionType(executionType);
    em.submitGoal(g);
    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      log.error(e.getMessage());
    }
    return g;
  }

  private void cancelAllGoals() {
    log.info("[cancelAllGoals] cleaning up");
    em.cancelCurrentGoals();
  }

  /**
   * Tests base queueing behavior. Goals of equal explicit priority should be
   * queued and subsequently executed in the order they are submitted once unblocked
   */
  @Test
  public void testQueueGoalsEqualPriority() {
    Predicate goalPred = Factory.createPredicate("freeze", "self:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);
    goalPred = Factory.createPredicate("drive", "self:agent");
    Goal driveGoal = submitGoalAndWait(goalPred);
    goalPred = Factory.createPredicate("look", "self:agent");
    Goal lookGoal = submitGoalAndWait(goalPred);
    assertSame(driveGoal.getStatus(), GoalStatus.PENDING);
    assertSame(lookGoal.getStatus(), GoalStatus.PENDING);
    submitGoalAndWait(Factory.createPredicate("endFreeze", "self:agent"));
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
    Predicate goalPred = Factory.createPredicate("freeze", "self:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);
    goalPred = Factory.createPredicate("drive", "self:agent");
    Goal driveGoal = submitGoalAndWait(goalPred, ExecutionType.ACT, 1L, PriorityTier.LOW);
    goalPred = Factory.createPredicate("look", "self:agent");
    Goal lookGoal = submitGoalAndWait(goalPred);
    assertSame(driveGoal.getStatus(), GoalStatus.PENDING);
    assertSame(lookGoal.getStatus(), GoalStatus.PENDING);
    submitGoalAndWait(Factory.createPredicate("endFreeze", "self:agent"));
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
    Predicate goalPred = Factory.createPredicate("freeze", "self:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);
    goalPred = Factory.createPredicate("drive", "self:agent");
    Goal driveGoal = submitGoalAndWait(goalPred, ExecutionType.ACT, 0L, PriorityTier.NORMAL);
    goalPred = Factory.createPredicate("look", "self:agent");
    Goal lookGoal = submitGoalAndWait(goalPred);
    assertSame(driveGoal.getStatus(), GoalStatus.PENDING);
    assertSame(lookGoal.getStatus(), GoalStatus.PENDING);
    submitGoalAndWait(Factory.createPredicate("endFreeze", "self:agent"));
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
    Predicate goalPred = Factory.createPredicate("freeze", "self:agent");
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
    Predicate goalPred = Factory.createPredicate("freeze", "self:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);
    goalPred = Factory.createPredicate("freeze", "agent3:agent");
    freezeGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.PENDING);

    cancelAllGoals();
  }

  /**
   * Tests handling of goals for different members of the same team.
   */
  @Test
  public void testSubmitGoalsForSiblings() {
    Predicate goalPred = Factory.createPredicate("freeze", "agent1:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);
    goalPred = Factory.createPredicate("freeze", "agent2:agent");
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
    Predicate goalPred = Factory.createPredicate("freeze", "team1:agent");
    Goal freeze1Goal = submitGoalAndWait(goalPred, ExecutionType.ACT, 1L, PriorityTier.LOW);
    assertSame(GoalStatus.ACTIVE, freeze1Goal.getStatus());

    goalPred = Factory.createPredicate("freeze", "self:agent");
    Goal freeze2Goal = submitGoalAndWait(goalPred);
    assertSame(GoalStatus.ACTIVE, freeze2Goal.getStatus());
    assertSame(GoalStatus.SUSPENDED, freeze1Goal.getStatus());

    Goal endFreezeGoal = submitGoalAndWait(Factory.createPredicate("endFreeze", "self:agent"));
    em.joinOnGoal(endFreezeGoal.getId());
    em.joinOnGoal(freeze2Goal.getId());
    assertSame(GoalStatus.SUCCEEDED, freeze2Goal.getStatus());
    assertSame(GoalStatus.ACTIVE, freeze1Goal.getStatus());

    endFreezeGoal = submitGoalAndWait(Factory.createPredicate("endFreeze", "team1:agent"));
    em.joinOnGoal(endFreezeGoal.getId());
    em.joinOnGoal(freeze1Goal.getId());
    assertSame(GoalStatus.SUCCEEDED, freeze1Goal.getStatus());

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
  //TODO: this test causes things to hang when run as part of suite. Issue with shutdown/startup?
  @Ignore
  @Test
  public void testQueueResourceConflictBlocks() {
    Predicate goalPred = Factory.createPredicate("freeze", "agent1:agent");
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
    Predicate goalPred = Factory.createPredicate("freeze", "agent1:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);

    goalPred = Factory.createPredicate("grab", "self:agent");
    Goal grabGoal = submitGoalAndWait(goalPred);
    assertSame(grabGoal.getStatus(), GoalStatus.PENDING);

    goalPred = Factory.createPredicate("drive", "agent2:agent");
    Goal driveGoal = submitGoalAndWait(goalPred);
    assertSame(driveGoal.getStatus(), GoalStatus.PENDING);

    goalPred = Factory.createPredicate("look", "agent3:agent");
    Goal lookGoal = submitGoalAndWait(goalPred);
    assertSame(lookGoal.getStatus(), GoalStatus.PENDING);

    em.cancelGoal(grabGoal.getId());
    assertSame(grabGoal.getStatus(), GoalStatus.CANCELED);
    em.joinOnGoal(driveGoal.getId());
    assertSame(driveGoal.getStatus(), GoalStatus.SUCCEEDED);
    em.joinOnGoal(lookGoal.getId());
    assertSame(lookGoal.getStatus(), GoalStatus.SUCCEEDED);

    cancelAllGoals();
  }

  /**
   * Tests that canceling a goal from the pending collection is
   * handled properly
   */
  @Test
  public void testCancelPendingGoal() {
    Predicate goalPred = Factory.createPredicate("freeze", "team1:agent");
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
    submitGoalAndWait(Factory.createPredicate("endFreeze", "team1:agent"));
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
    Predicate goalPred = Factory.createPredicate("freeze", "team1:agent");
    Goal freeze1Goal = submitGoalAndWait(goalPred, ExecutionType.ACT, 1L, PriorityTier.LOW);
    assertSame(freeze1Goal.getStatus(), GoalStatus.ACTIVE);

    goalPred = Factory.createPredicate("freeze", "self:agent");
    Goal freeze2Goal = submitGoalAndWait(goalPred);
    assertSame(freeze2Goal.getStatus(), GoalStatus.ACTIVE);
    assertSame(freeze1Goal.getStatus(), GoalStatus.SUSPENDED);

    em.cancelGoal(freeze1Goal.getId());
    em.joinOnGoal(freeze1Goal.getId());
    assertSame(freeze1Goal.getStatus(), GoalStatus.CANCELED);
    assertNotNull(em.getPastGoal(freeze1Goal.getId()));

    cancelAllGoals();
  }

  /**
   * Tests canceling freeze (As of now a behavior on the EM itself alongside its
   * interruption behavior) works properly
   */
  @Test
  public void testCancelFreeze() {
    Predicate goalPred = Factory.createPredicate("freeze", "self:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);
    goalPred = Factory.createPredicate("drive", "self:agent");
    Goal driveGoal = submitGoalAndWait(goalPred);
    goalPred = Factory.createPredicate("look", "self:agent");
    Goal lookGoal = submitGoalAndWait(goalPred);
    assertSame(driveGoal.getStatus(), GoalStatus.PENDING);
    assertSame(lookGoal.getStatus(), GoalStatus.PENDING);
    em.cancelSystemGoals();
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
    Predicate goalPred = Factory.createPredicate("learnAction", "self:agent", "testAction()");
    Goal learningGoal = submitGoalAndWait(goalPred);
    assertSame(learningGoal.getStatus(), GoalStatus.ACTIVE);

    goalPred = Factory.createPredicate("drive", "self:agent");
    Goal driveGoal = submitGoalAndWait(goalPred);
    goalPred = Factory.createPredicate("look", "self:agent");
    Goal lookGoal = submitGoalAndWait(goalPred);
    em.joinOnGoal(driveGoal.getId());
    assertSame(driveGoal.getStatus(), GoalStatus.SUCCEEDED);
    em.joinOnGoal(lookGoal.getId());
    assertSame(lookGoal.getStatus(), GoalStatus.SUCCEEDED);

    goalPred = Factory.createPredicate("endActionLearning", "self:agent", "testAction()");
    Goal endLearningGoal = submitGoalAndWait(goalPred);
    em.joinOnGoal(endLearningGoal.getId());
    em.joinOnGoal(learningGoal.getId());
    assertSame(learningGoal.getStatus(), GoalStatus.SUCCEEDED);
    assertSame(endLearningGoal.getStatus(), GoalStatus.SUCCEEDED);

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

  /**
   * Tests handling of a goal being submitted which has higher priority than
   * the currently active goal. The current goal should have its execution
   * superseded and pushed back to pending. Upon completion of the superseder,
   * the original goal should be returned to the current goal with its original suspended status.
   */
  @Test
  public void testSupersedeSuspendedGoal() {
    Predicate goalPred = Factory.createPredicate("freeze", "team1:agent");
    Goal freeze1Goal = submitGoalAndWait(goalPred, ExecutionType.ACT, 1L, PriorityTier.LOW);
    assertSame(GoalStatus.ACTIVE, freeze1Goal.getStatus());
    em.suspendGoal(freeze1Goal.getId());
    em.joinOnGoal(freeze1Goal.getId(), 1000);
    assertSame(GoalStatus.SUSPENDED, freeze1Goal.getStatus());

    goalPred = Factory.createPredicate("freeze", "self:agent");
    Goal freeze2Goal = submitGoalAndWait(goalPred);
    assertSame( GoalStatus.ACTIVE, freeze2Goal.getStatus());
    assertSame(GoalStatus.SUSPENDED, freeze1Goal.getStatus());

    Goal endFreezeGoal = submitGoalAndWait(Factory.createPredicate("endFreeze", "self:agent"));
    em.joinOnGoal(endFreezeGoal.getId());
    em.joinOnGoal(freeze2Goal.getId());
    assertSame(GoalStatus.SUCCEEDED, freeze2Goal.getStatus());
    assertSame(GoalStatus.SUSPENDED, freeze1Goal.getStatus());
    em.resumeGoal(freeze1Goal.getId());
    assertSame(GoalStatus.ACTIVE, freeze1Goal.getStatus());

    endFreezeGoal = submitGoalAndWait(Factory.createPredicate("endFreeze", "team1:agent"));
    em.joinOnGoal(endFreezeGoal.getId());
    em.joinOnGoal(freeze1Goal.getId());
    assertSame( GoalStatus.SUCCEEDED, freeze1Goal.getStatus());

    cancelAllGoals();
  }

  @Test
  public void testCancelAllPending() {
    Predicate goalPred = Factory.createPredicate("freeze", "self:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);

    goalPred = Factory.createPredicate("drive", "self:agent");
    Goal driveGoal = submitGoalAndWait(goalPred);
    goalPred = Factory.createPredicate("look", "self:agent");
    Goal lookGoal = submitGoalAndWait(goalPred);
    assertSame(driveGoal.getStatus(), GoalStatus.PENDING);
    assertSame(lookGoal.getStatus(), GoalStatus.PENDING);

    em.cancelPendingGoals();
    em.joinOnGoal(driveGoal.getId());
    em.joinOnGoal(lookGoal.getId());
    assertSame(driveGoal.getStatus(), GoalStatus.CANCELED);
    assertSame(lookGoal.getStatus(), GoalStatus.CANCELED);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);

    cancelAllGoals();
  }

  @Test
  public void testCancelAllActive() {
    Predicate goalPred = Factory.createPredicate("freeze", "team1:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    goalPred = Factory.createPredicate("freeze", "agent3:agent");
    Goal freeze2Goal = submitGoalAndWait(goalPred);
    goalPred = Factory.createPredicate("drive", "agent1:agent");
    Goal driveGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);
    assertSame(freeze2Goal.getStatus(), GoalStatus.ACTIVE);
    assertSame(driveGoal.getStatus(), GoalStatus.PENDING);

    em.cancelActiveGoals();
    em.joinOnGoal(freezeGoal.getId());
    em.joinOnGoal(freeze2Goal.getId());
    em.joinOnGoal(driveGoal.getId());
    assertSame(freezeGoal.getStatus(), GoalStatus.CANCELED);
    assertSame(freeze2Goal.getStatus(), GoalStatus.CANCELED);
    assertSame(driveGoal.getStatus(), GoalStatus.SUCCEEDED);

    cancelAllGoals();
  }

  @Test
  public void testCancelAllCurrent() {
    Predicate goalPred = Factory.createPredicate("freeze", "team1:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    goalPred = Factory.createPredicate("freeze", "agent3:agent");
    Goal freeze2Goal = submitGoalAndWait(goalPred);
    goalPred = Factory.createPredicate("drive", "agent1:agent");
    Goal driveGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);
    assertSame(freeze2Goal.getStatus(), GoalStatus.ACTIVE);
    assertSame(driveGoal.getStatus(), GoalStatus.PENDING);

    em.cancelCurrentGoals();
    em.joinOnGoal(freezeGoal.getId());
    em.joinOnGoal(freeze2Goal.getId());
    em.joinOnGoal(driveGoal.getId());
    assertSame(freezeGoal.getStatus(), GoalStatus.CANCELED);
    assertSame(freeze2Goal.getStatus(), GoalStatus.CANCELED);
    assertSame(driveGoal.getStatus(), GoalStatus.CANCELED);

    cancelAllGoals();
  }

  @Test
  public void testGetGoalPredicates() {
    Predicate goalPred = Factory.createPredicate("freeze", "team1:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    Predicate goalPred2 = Factory.createPredicate("freeze", "agent3:agent");
    submitGoalAndWait(goalPred2);
    Predicate goalPred3 = Factory.createPredicate("look", "agent3:agent");
    submitGoalAndWait(goalPred3);
    Predicate goalPred4 = Factory.createPredicate("drive", "self:agent");
    submitGoalAndWait(goalPred4);
    Predicate goalPred5 = Factory.createPredicate("speak", "team1:agent");
    submitGoalAndWait(goalPred5);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);

    List<Predicate> expendedPendingList = new ArrayList<>();
    expendedPendingList.add(goalPred3);
    expendedPendingList.add(goalPred4);
    expendedPendingList.add(goalPred5);
    List<Predicate> pendingList = new ArrayList<>();
    em.getPendingGoals().forEach(g -> pendingList.add(g.getGoalPredicate()));
    assertEquals(expendedPendingList, pendingList);

    List<Goal> agent3Goals = em.getSystemGoals(Factory.createSymbol("agent3:agent"));
    assertEquals(agent3Goals.size(), 1);
    assertEquals(agent3Goals.get(0).getGoalPredicate(), goalPred2);
    assertEquals(agent3Goals.get(0).getStatus(), GoalStatus.ACTIVE);

    List<Goal> selfGoals = em.getSystemGoals(Factory.createSymbol("self:agent"));
    assertEquals(selfGoals.size(), 2);
    assertEquals(selfGoals.get(0).getStatus(), GoalStatus.ACTIVE);
    assertEquals(selfGoals.get(1).getStatus(), GoalStatus.ACTIVE);
    assertEquals(selfGoals.get(1).getGoalPredicate(), goalPred);
    assertEquals(selfGoals.get(0).getGoalPredicate(), goalPred2);

    assertSame(em.getNextGoal().getGoalPredicate(), goalPred3);
    assertSame(em.getNextGoal(Factory.createSymbol("team1:agent")).getGoalPredicate(), goalPred5);
    assertNull(em.getNextGoal(Factory.createSymbol("agent1:agent")));

    cancelAllGoals();
  }

  @Test
  public void testCancelSystemGoals() {
    Predicate goalPred = Factory.createPredicate("freeze", "agent1:agent");
    Goal freezeGoal1 = submitGoalAndWait(goalPred);
    goalPred = Factory.createPredicate("freeze", "agent2:agent");
    Goal freezeGoal2 = submitGoalAndWait(goalPred);
    assertSame(GoalStatus.ACTIVE, freezeGoal1.getStatus());
    assertSame(GoalStatus.ACTIVE, freezeGoal2.getStatus());

    em.cancelSystemGoals(Factory.createSymbol("agent1:agent"));
    em.joinOnGoal(freezeGoal1.getId(), 2000);
    assertSame(GoalStatus.CANCELED, freezeGoal1.getStatus());
    assertSame(GoalStatus.ACTIVE, freezeGoal2.getStatus());

    em.cancelSystemGoals(Factory.createSymbol("team1:agent"));
    em.joinOnGoal(freezeGoal2.getId(), 2000);
    assertSame(GoalStatus.CANCELED, freezeGoal2.getStatus());

    goalPred = Factory.createPredicate("freeze", "self:agent");
    Goal freezeGoal3 = submitGoalAndWait(goalPred);
    assertSame(GoalStatus.ACTIVE, freezeGoal3.getStatus());
    em.cancelSystemGoals(Factory.createSymbol("team1:agent"));
    assertSame(GoalStatus.CANCELED, freezeGoal2.getStatus());

    cancelAllGoals();
  }

  //Changed how this works. If you want this functionality, you should just use Freeze. Otherwise, could
  //  write a 'suspendAllSystemGoals' variation
  @Ignore
  @Test
  public void testSuspendResumeSystemGoals() {
    Predicate goalPred = Factory.createPredicate("freeze", "agent1:agent");
    Goal freezeGoal1 = submitGoalAndWait(goalPred);
    goalPred = Factory.createPredicate("freeze", "agent2:agent");
    Goal freezeGoal2 = submitGoalAndWait(goalPred);
    assertSame(GoalStatus.ACTIVE, freezeGoal1.getStatus());
    assertSame(GoalStatus.ACTIVE, freezeGoal2.getStatus());

    em.suspendSystemGoals(Factory.createSymbol("agent1:agent"));
    em.joinOnGoal(freezeGoal1.getId(), 2000);
    assertSame(GoalStatus.SUSPENDED, freezeGoal1.getStatus());
    assertSame(GoalStatus.ACTIVE, freezeGoal2.getStatus());

    em.suspendSystemGoals(Factory.createSymbol("team1:agent"));
    em.joinOnGoal(freezeGoal2.getId(), 2000);
    assertSame(GoalStatus.SUSPENDED, freezeGoal2.getStatus());

    em.resumeSystemGoals(Factory.createSymbol("team1:agent"));
    assertSame(GoalStatus.ACTIVE, freezeGoal1.getStatus());
    assertSame(GoalStatus.ACTIVE, freezeGoal2.getStatus());

    em.cancelGoal(freezeGoal1.getId());
    em.cancelGoal(freezeGoal2.getId());

    goalPred = Factory.createPredicate("freeze", "team1:agent");
    Goal freezeGoal4 = submitGoalAndWait(goalPred);
    em.suspendSystemGoals(Factory.createSymbol("agent2:agent"));
    assertSame(GoalStatus.ACTIVE, freezeGoal4.getStatus());

    cancelAllGoals();
  }

  @Test
  public void testFreeze() {
    Predicate goalPred = Factory.createPredicate("freeze", "team1:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    goalPred = Factory.createPredicate("endFreeze", "agent2:agent");
    Goal endFreezeGoal = submitGoalAndWait(goalPred);
    Util.Sleep(2000);
    assertSame(GoalStatus.ACTIVE, freezeGoal.getStatus());
    assertSame(GoalStatus.SUCCEEDED, endFreezeGoal.getStatus());

    cancelAllGoals();
  }

  //TODO: Make test in some domain where we suspend on the last step of a script, where that step is uninterruptible
  //        (want to make sure we properly finish execution of the script and fail to suspend when the last step is complete)
}
