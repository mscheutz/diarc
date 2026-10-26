package edu.tufts.hrilab.action.manager;

import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalInfo;
import edu.tufts.hrilab.action.goal.GoalSchedulingExplanation;
import edu.tufts.hrilab.action.goal.GoalSchedulingExplanation.Location;
import edu.tufts.hrilab.action.goal.GoalSchedulingExplanation.Reason;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.goal.PriorityTier;
import edu.tufts.hrilab.action.goal.schedule.ScheduledGoal;
import edu.tufts.hrilab.action.learning.ActionLearningStatus;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import org.junit.Test;
import org.quartz.Scheduler;

import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.util.Date;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.CompletableFuture;

import static org.junit.Assert.*;

public class GoalSchedulingExplanationTest {
  private static final Symbol SELF = Factory.createSymbol("self");

  private static class TestManager extends ExecutionManager {
    TestManager() {
      rootAgent = SELF;
      agentHierarchy = Map.of(SELF, Set.of(SELF));
      agentTeams.put(SELF, new AgentTeam(SELF, this));
    }

    @Override
    protected void onPendingGoalUpdated(Goal goal, int index, UpdateType type) {
    }

    @Override
    protected void onActiveGoalUpdated(Goal goal, GoalStatus status, UpdateType type) {
    }

    @Override
    protected boolean executeGoal(Goal goal) {
      getAgentTeam(SELF).addGoal(goal, new CompletableFuture<>());
      goal.setAsStarted();
      return true;
    }

    Resource resource(String name) {
      return new Resource(getAgentTeam(SELF), Factory.createSymbol(name));
    }
  }

  private Goal goal(long priority, Set<Resource> required, Set<Resource> held) {
    Goal goal = Goal.createGoal(Factory.createPredicate("test(self)"));
    goal.setPriority(priority);
    goal.setRequiredResources(required);
    goal.setHeldResources(held);
    return goal;
  }

  @Test
  public void unknownGoalIsExplicitAndHasNoFabricatedMetadata() {
    GoalSchedulingExplanation result = new TestManager().explainGoalScheduling(Long.MAX_VALUE);
    assertEquals(Location.UNKNOWN, result.location);
    assertEquals(java.util.List.of(Reason.UNKNOWN_GOAL), result.reasons);
    assertNull(result.getGoalInfo());
    assertEquals(-1, result.queuePosition);
  }

  @Test
  public void readyGoalReportsPriorityQueuePositionAndClaimModesWithoutChangingState() {
    TestManager manager = new TestManager();
    Resource resource = manager.resource("freeze");
    Goal first = goal(10, Set.of(resource), Set.of());
    Goal second = goal(1, Set.of(resource), Set.of());
    second.setPriorityTier(PriorityTier.LOW);
    manager.submitGoal(first);
    manager.submitGoal(second);
    GoalSchedulingExplanation result = manager.explainGoalScheduling(second.getId());
    assertEquals(Location.PENDING, result.location);
    assertEquals(1, result.queuePosition);
    assertEquals(java.util.List.of(Reason.READY), result.reasons);
    assertEquals(TestManager.class.getName(), result.policy);
    assertEquals(PriorityTier.LOW, result.getGoalInfo().priorityTier);
    assertEquals("self", result.claims.get(0).provider);
    assertEquals("freeze", result.claims.get(0).resource);
    assertTrue(result.claims.get(0).required);
    assertFalse(result.claims.get(0).held);
    assertEquals(2, manager.getPendingGoals().size());
    assertEquals(0, second.getInfo().statusVersion);
    assertTrue(resource.isAvailable());
    assertTrue(resource.getUsers().isEmpty());
  }

  @Test
  public void activeRequiredOnlyUsersExplainWhyExclusiveGoalIsBlocked() {
    TestManager manager = new TestManager();
    Resource resource = manager.resource("learning");
    Goal reader = goal(10, Set.of(resource), Set.of());
    manager.submitGoal(reader);
    manager.activateValidPendingGoals();
    Goal writer = goal(1, Set.of(), Set.of(resource));
    manager.submitGoal(writer);
    GoalSchedulingExplanation result = manager.explainGoalScheduling(writer.getId());
    assertTrue(result.reasons.contains(Reason.ACTIVE_RESOURCE_CONFLICT));
    assertEquals(1, result.blockers.size());
    GoalSchedulingExplanation.Blocker blocker = result.blockers.get(0);
    assertEquals(reader.getId(), blocker.getGoalInfo().gid);
    assertEquals(Location.ACTIVE, blocker.location);
    assertEquals(GoalStatus.ACTIVE, blocker.getGoalInfo().status);
    assertTrue(blocker.resources.get(0).goalHolds);
    assertFalse(blocker.resources.get(0).blockerHolds);
    assertEquals("learning", blocker.resources.get(0).resource);
  }

  @Test
  public void exclusiveHolderExplainsWhyRequiredOnlyGoalIsBlocked() {
    TestManager manager = new TestManager();
    Resource resource = manager.resource("freeze");
    Goal holder = goal(10, Set.of(), Set.of(resource));
    manager.submitGoal(holder);
    manager.activateValidPendingGoals();
    Goal candidate = goal(1, Set.of(resource), Set.of());
    manager.submitGoal(candidate);
    GoalSchedulingExplanation result = manager.explainGoalScheduling(candidate.getId());
    assertTrue(result.reasons.contains(Reason.ACTIVE_RESOURCE_CONFLICT));
    assertEquals(holder.getId(), result.blockers.get(0).getGoalInfo().gid);
    assertFalse(result.blockers.get(0).resources.get(0).goalHolds);
    assertTrue(result.blockers.get(0).resources.get(0).blockerHolds);
  }

  @Test
  public void onlyConflictingHigherPriorityPendingGoalsAreReported() {
    TestManager manager = new TestManager();
    Resource resource = manager.resource("shared");
    Goal higher = goal(10, Set.of(resource), Set.of());
    Goal candidate = goal(5, Set.of(), Set.of(resource));
    Goal lower = goal(1, Set.of(), Set.of(resource));
    manager.submitGoal(higher);
    manager.submitGoal(candidate);
    manager.submitGoal(lower);
    GoalSchedulingExplanation result = manager.explainGoalScheduling(candidate.getId());
    assertEquals(java.util.List.of(Reason.HIGHER_PRIORITY_PENDING_CONFLICT), result.reasons);
    assertEquals(1, result.blockers.size());
    assertEquals(higher.getId(), result.blockers.get(0).getGoalInfo().gid);
    assertEquals(Location.PENDING, result.blockers.get(0).location);
    assertTrue(manager.resourceConflictInPending(candidate, result.queuePosition));
  }

  @Test
  public void sameResourceNameFromDifferentProvidersIsNotAConflict() {
    TestManager manager = new TestManager();
    Resource first = manager.resource("same");
    AgentTeam other = new AgentTeam(Factory.createSymbol("other"), manager);
    Resource second = new Resource(other, Factory.createSymbol("same"));
    Goal higher = goal(10, Set.of(), Set.of(first));
    Goal candidate = goal(1, Set.of(), Set.of(second));
    manager.submitGoal(higher);
    manager.submitGoal(candidate);
    GoalSchedulingExplanation result = manager.explainGoalScheduling(candidate.getId());
    assertEquals(java.util.List.of(Reason.READY), result.reasons);
    assertTrue(result.blockers.isEmpty());
    assertEquals("other", result.claims.get(0).provider);
  }

  @Test
  public void activeExplanationUsesAcquiredClaimsAndIsDetachedFromLiveGoal() {
    TestManager manager = new TestManager();
    Resource resource = manager.resource("acquired");
    Goal goal = goal(1, Set.of(resource), Set.of(resource));
    manager.submitGoal(goal);
    manager.activateValidPendingGoals();
    goal.setRequiredResources(Set.of());
    goal.setHeldResources(Set.of());
    GoalSchedulingExplanation result = manager.explainGoalScheduling(goal.getId());
    assertEquals(Location.ACTIVE, result.location);
    assertEquals(java.util.List.of(Reason.EXECUTING), result.reasons);
    assertEquals("acquired", result.claims.get(0).resource);
    GoalInfo snapshot = result.getGoalInfo();
    snapshot.status = GoalStatus.FAILED;
    assertEquals(GoalStatus.ACTIVE, result.getGoalInfo().status);
    goal.setStatus(GoalStatus.SUCCEEDED);
    assertEquals(GoalStatus.ACTIVE, result.getGoalInfo().status);
    GoalSchedulingExplanation past = manager.explainGoalScheduling(goal.getId());
    assertEquals(Location.PAST, past.location);
    assertEquals(java.util.List.of(Reason.TERMINATED), past.reasons);
    assertTrue(past.claims.isEmpty());
  }

  @Test
  public void invalidActorFailureCanBeExplainedWithoutComputingInvalidClaims() {
    TestManager manager = new TestManager();
    Goal invalid = Goal.createGoal(Factory.createPredicate("test(missing)"));
    manager.submitGoal(invalid);
    GoalSchedulingExplanation result = manager.explainGoalScheduling(invalid.getId());
    assertEquals(Location.PAST, result.location);
    assertEquals(GoalStatus.FAILED, result.getGoalInfo().status);
    assertFalse(result.getGoalInfo().failConditions.getValue());
  }

  @Test
  public void learningAndSuspensionHaveExplicitReasons() {
    TestManager manager = new TestManager();
    Goal suspended = goal(1, Set.of(), Set.of());
    manager.submitGoal(suspended);
    suspended.setStatus(GoalStatus.SUSPENDED);
    assertTrue(manager.explainGoalScheduling(suspended.getId()).reasons.contains(Reason.SUSPENDED));
    manager.agentTeams.put(SELF, new AgentTeam(SELF, manager) {
      @Override
      protected ActionLearningStatus getLearningStatus() {
        return ActionLearningStatus.ACTIVE;
      }

      @Override
      protected boolean shouldIgnore(Goal goal) {
        return false;
      }

      @Override
      protected void addLearningGoal(Goal goal) {
      }
    });
    Goal taught = goal(1, Set.of(), Set.of());
    manager.submitGoal(taught);
    GoalSchedulingExplanation result = manager.explainGoalScheduling(taught.getId());
    assertEquals(Location.LEARNING, result.location);
    assertEquals(java.util.List.of(Reason.LEARNING), result.reasons);
  }

  @Test(expected = UnsupportedOperationException.class)
  public void explanationCollectionsCannotBeMutated() {
    new TestManager().explainGoalScheduling(-1).reasons.clear();
  }

  @Test
  public void detailedConflictsAndSchedulingPredicateAgreeForAllClaimModes() {
    TestManager manager = new TestManager();
    Resource resource = manager.resource("r");
    for (int first = 0; first < 4; first++) {
      for (int second = 0; second < 4; second++) {
        ResourceClaims left = new ResourceClaims((first & 1) != 0 ? Set.of(resource) : Set.of(),
                (first & 2) != 0 ? Set.of(resource) : Set.of());
        ResourceClaims right = new ResourceClaims((second & 1) != 0 ? Set.of(resource) : Set.of(),
                (second & 2) != 0 ? Set.of(resource) : Set.of());
        assertEquals(left.conflictsWith(right), !left.conflictingResources(right).isEmpty());
        assertEquals(left.conflictingResources(right), right.conflictingResources(left));
      }
    }
  }

  @Test
  public void inFlightTerminationIsNotReportedAsReady() {
    TestManager manager = new TestManager() {
      @Override
      public void goalStatusChanged(GoalInfo info) {
      }
    };
    Goal goal = goal(1, Set.of(), Set.of());
    manager.submitGoal(goal);
    goal.setStatus(GoalStatus.CANCELED);
    GoalSchedulingExplanation result = manager.explainGoalScheduling(goal.getId());
    assertEquals(Location.PENDING, result.location);
    assertEquals(java.util.List.of(Reason.TERMINATING), result.reasons);
  }

  @Test
  public void registeredInterpreterBeforeStartIsNotReportedAsExecuting() {
    TestManager manager = new TestManager();
    Goal goal = goal(1, Set.of(), Set.of());
    manager.submitGoal(goal);
    manager.pendingGoals.clear();
    manager.getAgentTeam(SELF).addGoal(goal, new CompletableFuture<>());
    GoalSchedulingExplanation result = manager.explainGoalScheduling(goal.getId());
    assertEquals(Location.ACTIVE, result.location);
    assertEquals(java.util.List.of(Reason.STARTING), result.reasons);
  }

  @Test(timeout = 3000)
  public void scheduledDefinitionIsDistinguishedFromAnExecutingOccurrence() throws Exception {
    TestManager manager = new TestManager();
    ScheduledGoalManager scheduled = new ScheduledGoalManager(manager);
    Field field = ExecutionManager.class.getDeclaredField("scheduledGoalManager");
    field.setAccessible(true);
    field.set(manager, scheduled);
    Field schedulerField = ScheduledGoalManager.class.getDeclaredField("goalScheduler");
    schedulerField.setAccessible(true);
    Scheduler original = (Scheduler) schedulerField.get(scheduled);
    try {
      Scheduler proxy = (Scheduler) Proxy.newProxyInstance(getClass().getClassLoader(),
              new Class<?>[]{Scheduler.class}, (object, method, args) -> {
                if (method.getName().equals("scheduleJob")) {
                  return new Date();
                }
                throw new AssertionError(method.getName());
              });
      schedulerField.set(scheduled, proxy);
      Goal goal = new ScheduledGoal(Factory.createPredicate("scheduled(test(self),start(1,DAYS))"));
      manager.submitGoal(goal);
      GoalSchedulingExplanation result = manager.explainGoalScheduling(goal.getId());
      assertEquals(Location.SCHEDULED, result.location);
      assertEquals(java.util.List.of(Reason.SCHEDULED), result.reasons);
      assertEquals(1, result.getGoalInfo().statusVersion);
      assertTrue(result.claims.isEmpty());
    } finally {
      original.shutdown();
    }
  }
}
