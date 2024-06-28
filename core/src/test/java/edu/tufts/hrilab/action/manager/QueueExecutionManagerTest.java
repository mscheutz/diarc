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
import org.junit.AfterClass;
import org.junit.BeforeClass;
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
    PriorityTier priorityTier = PriorityTier.NORMAL;
    long priority = 1;
    if (goalPred.getName().equals("freeze")) {
      priorityTier = PriorityTier.URGENT;
      priority = 9999;
    } else if (goalPred.getName().equals("endFreeze")) {
      priorityTier = PriorityTier.SKIPPENDING;
    } else if (goalPred.getName().equals("updateSettings")) {
      priorityTier = PriorityTier.SKIPPENDING;
    } else if (goalPred.getName().equals("cancelSystemGoal")) {
      priorityTier = PriorityTier.SKIPPENDING;
    } else if (goalPred.getName().equals("suspendSystemGoal")) {
      priorityTier = PriorityTier.SKIPPENDING;
    } else if (goalPred.getName().equals("resumeSystemGoal")) {
      priorityTier = PriorityTier.SKIPPENDING;
    } else if (goalPred.getName().equals("cancelAllPendingGoals")) {
      priorityTier = PriorityTier.SKIPPENDING;
    } else if (goalPred.getName().equals("cancelAllActiveGoals")) {
      priorityTier = PriorityTier.SKIPPENDING;
    } else if (goalPred.getName().equals("cancelAllCurrentGoals")) {
      priorityTier = PriorityTier.SKIPPENDING;
    }
    Goal g = new Goal(goalPred);
    g.setPriorityTier(priorityTier);
    g.setPriority(priority);

    g = em.submitGoal(g, ExecutionType.ACT);
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
    log.info("[cancelAllGoals] cleaning up");
    em.cancelAllCurrentGoals();
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
    assertNotNull(em.getPastGoal(freeze1Goal.getId()));

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
    Goal cancelGoal = submitGoalAndWait(Factory.createPredicate("cancelSystemGoal", "self:agent"));
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

  /**
   * Tests handling of a goal being submitted which has higher priority than
   * the currently active goal. The current goal should have its execution
   * superseded and pushed back to pending. Upon completion of the superseder,
   * the original goal should be returned to the current goal with its original suspended status.
   */
  @Test
  public void testSupersedeSuspendedGoal() {
    Predicate goalPred = Factory.createPredicate("freeze", "team1:agent", "team1:agent");
    Goal freeze1Goal = submitGoalAndWait(goalPred, ExecutionType.ACT, 1L, PriorityTier.LOW);
    assertSame(freeze1Goal.getStatus(), GoalStatus.ACTIVE);
    em.suspendGoal(freeze1Goal.getId());
    assertSame(freeze1Goal.getStatus(), GoalStatus.SUSPENDED);

    goalPred = Factory.createPredicate("freeze", "self:agent", "self:agent");
    Goal freeze2Goal = submitGoalAndWait(goalPred);
    assertSame(freeze2Goal.getStatus(), GoalStatus.ACTIVE);
    assertSame(freeze1Goal.getStatus(), GoalStatus.SUSPENDED);

    Goal endFreezeGoal = submitGoalAndWait(Factory.createPredicate("endFreeze", "self:agent", "self:agent"));
    em.joinOnGoal(endFreezeGoal.getId());
    em.joinOnGoal(freeze2Goal.getId());
    assertSame(freeze2Goal.getStatus(), GoalStatus.SUCCEEDED);
    assertSame(freeze1Goal.getStatus(), GoalStatus.SUSPENDED);
    em.resumeGoal(freeze1Goal.getId());
    assertSame(freeze1Goal.getStatus(), GoalStatus.ACTIVE);

    endFreezeGoal = submitGoalAndWait(Factory.createPredicate("endFreeze", "team1:agent", "team1:agent"));
    em.joinOnGoal(endFreezeGoal.getId());
    em.joinOnGoal(freeze1Goal.getId());
    assertSame(freeze1Goal.getStatus(), GoalStatus.SUCCEEDED);

    cancelAllGoals();
  }

  @Test
  public void testCancelAllPending() {
    Predicate goalPred = Factory.createPredicate("freeze", "self:agent", "self:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);

    goalPred = Factory.createPredicate("drive", "self:agent");
    Goal driveGoal = submitGoalAndWait(goalPred);
    goalPred = Factory.createPredicate("look", "self:agent");
    Goal lookGoal = submitGoalAndWait(goalPred);
    assertSame(driveGoal.getStatus(), GoalStatus.PENDING);
    assertSame(lookGoal.getStatus(), GoalStatus.PENDING);

    em.cancelAllPendingGoals();
    em.joinOnGoal(driveGoal.getId());
    em.joinOnGoal(lookGoal.getId());
    assertSame(driveGoal.getStatus(), GoalStatus.CANCELED);
    assertSame(lookGoal.getStatus(), GoalStatus.CANCELED);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);

    cancelAllGoals();
  }

  @Test
  public void testCancelAllActive() {
    Predicate goalPred = Factory.createPredicate("freeze", "team1:agent", "team1:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    goalPred = Factory.createPredicate("freeze", "agent3:agent", "agent3:agent");
    Goal freeze2Goal = submitGoalAndWait(goalPred);
    goalPred = Factory.createPredicate("drive", "agent1:agent");
    Goal driveGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);
    assertSame(freeze2Goal.getStatus(), GoalStatus.ACTIVE);
    assertSame(driveGoal.getStatus(), GoalStatus.PENDING);

    em.cancelAllActiveGoals();
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
    Predicate goalPred = Factory.createPredicate("freeze", "team1:agent", "team1:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    goalPred = Factory.createPredicate("freeze", "agent3:agent", "agent3:agent");
    Goal freeze2Goal = submitGoalAndWait(goalPred);
    goalPred = Factory.createPredicate("drive", "agent1:agent");
    Goal driveGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);
    assertSame(freeze2Goal.getStatus(), GoalStatus.ACTIVE);
    assertSame(driveGoal.getStatus(), GoalStatus.PENDING);

    em.cancelAllCurrentGoals();
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
    Predicate goalPred = Factory.createPredicate("freeze", "team1:agent", "team1:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    Predicate goalPred2 = Factory.createPredicate("freeze", "agent3:agent", "agent3:agent");
    submitGoalAndWait(goalPred2);
    Predicate goalPred3 = Factory.createPredicate("look", "agent3:agent");
    submitGoalAndWait(goalPred3);
    Predicate goalPred4 = Factory.createPredicate("drive", "self:agent");
    submitGoalAndWait(goalPred4);
    Predicate goalPred5 = Factory.createPredicate("speak", "team1:agent");
    submitGoalAndWait(goalPred5);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);

    List<Predicate> pendingList = new ArrayList<>();
    pendingList.add(goalPred3);
    pendingList.add(goalPred4);
    pendingList.add(goalPred5);
    assertEquals(em.getPendingGoalsPredicates(), pendingList);

    List<Predicate> systemList = new ArrayList<>();
    systemList.add(Factory.createPredicate("ACTIVE",goalPred2));
    assertEquals(em.getActiveGoalsPredicates(Factory.createSymbol("agent3:agent")), systemList);

    systemList.add(Factory.createPredicate("ACTIVE",goalPred));
    assertEquals(em.getActiveGoalsPredicates(Factory.createSymbol("self:agent")), systemList);

    assertSame(em.getNextGoalPredicate(), goalPred3);
    assertSame(em.getNextGoalPredicate(Factory.createSymbol("team1:agent")), goalPred5);
    assertEquals(em.getNextGoalPredicate(Factory.createSymbol("agent1:agent")), Factory.createPredicate("none()"));

    cancelAllGoals();
  }

  @Test
  public void testcancelSystemGoal() {
    Predicate goalPred = Factory.createPredicate("freeze", "agent1:agent", "agent1:agent");
    Goal freezeGoal1 = submitGoalAndWait(goalPred);
    goalPred = Factory.createPredicate("freeze", "agent2:agent", "agent2:agent");
    Goal freezeGoal2 = submitGoalAndWait(goalPred);
    assertSame(freezeGoal1.getStatus(), GoalStatus.ACTIVE);
    assertSame(freezeGoal2.getStatus(), GoalStatus.ACTIVE);

    em.cancelSystemGoal(Factory.createSymbol("agent1:agent"));
    assertSame(freezeGoal1.getStatus(), GoalStatus.CANCELED);
    assertSame(freezeGoal2.getStatus(), GoalStatus.ACTIVE);

    em.cancelSystemGoal(Factory.createSymbol("team1:agent"));
    assertSame(freezeGoal2.getStatus(), GoalStatus.CANCELED);

    goalPred = Factory.createPredicate("freeze", "self:agent", "self:agent");
    Goal freezeGoal3 = submitGoalAndWait(goalPred);
    assertSame(freezeGoal3.getStatus(), GoalStatus.ACTIVE);
    em.cancelSystemGoal(Factory.createSymbol("team1:agent"));
    assertSame(freezeGoal2.getStatus(), GoalStatus.CANCELED);

    cancelAllGoals();
  }

  @Test
  public void testSuspendresumeSystemGoal() {
    Predicate goalPred = Factory.createPredicate("freeze", "agent1:agent", "agent1:agent");
    Goal freezeGoal1 = submitGoalAndWait(goalPred);
    goalPred = Factory.createPredicate("freeze", "agent2:agent", "agent2:agent");
    Goal freezeGoal2 = submitGoalAndWait(goalPred);
    assertSame(freezeGoal1.getStatus(), GoalStatus.ACTIVE);
    assertSame(freezeGoal2.getStatus(), GoalStatus.ACTIVE);

    em.suspendSystemGoal(Factory.createSymbol("agent1:agent"));
    assertSame(freezeGoal1.getStatus(), GoalStatus.SUSPENDED);
    assertSame(freezeGoal2.getStatus(), GoalStatus.ACTIVE);

    em.suspendSystemGoal(Factory.createSymbol("team1:agent"));
    assertSame(freezeGoal2.getStatus(), GoalStatus.SUSPENDED);

    em.resumeSystemGoal(Factory.createSymbol("team1:agent"));
    assertSame(freezeGoal1.getStatus(), GoalStatus.ACTIVE);
    assertSame(freezeGoal2.getStatus(), GoalStatus.ACTIVE);

    em.cancelGoal(freezeGoal1.getId());
    em.cancelGoal(freezeGoal2.getId());

    goalPred = Factory.createPredicate("freeze", "team1:agent", "team1:agent");
    Goal freezeGoal4 = submitGoalAndWait(goalPred);
    em.suspendSystemGoal(Factory.createSymbol("agent2:agent"));
    assertSame(freezeGoal4.getStatus(), GoalStatus.ACTIVE);

    cancelAllGoals();
  }

  @Test
  public void testFreeze() {
    Predicate goalPred = Factory.createPredicate("freeze", "team1:agent", "team1:agent");
    Goal freezeGoal = submitGoalAndWait(goalPred);
    goalPred = Factory.createPredicate("endFreeze", "agent2:agent", "agent2:agent");
    Goal endfreezeGoal = submitGoalAndWait(goalPred);
    assertSame(freezeGoal.getStatus(), GoalStatus.ACTIVE);
    assertSame(endfreezeGoal.getStatus(), GoalStatus.SUCCEEDED);

    cancelAllGoals();
  }

  //TODO: Make test in some domain where we suspend on the last step of a script, where that step is uninterruptible
  //        (want to make sure we properly finish execution of the script and fail to suspend when the last step is complete)
}
