package edu.tufts.hrilab.action.manager;

import edu.tufts.hrilab.action.execution.ExecutionType;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.goal.PendingGoal;
import edu.tufts.hrilab.action.goal.PriorityTier;
import edu.tufts.hrilab.action.goal.schedule.ScheduledGoal;
import edu.tufts.hrilab.action.learning.ActionLearningStatus;
import edu.tufts.hrilab.action.listener.GoalListener;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import org.junit.Test;

import java.util.Map;
import java.util.Set;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.Assert.*;

public class StageOneExecutionManagerTest {
  private static final Symbol SELF = Factory.createSymbol("self");

  private static class TestManager extends ExecutionManager {
    int admissions;
    int removals;
    int executions;

    TestManager() {
      initialize(this);
    }

    @Override
    protected void onPendingGoalUpdated(Goal goal, int index, UpdateType type) {
      if (type == UpdateType.ADDED) {
        admissions++;
      } else if (type == UpdateType.REMOVED) {
        assertNull(getPendingGoal(goal.getId()));
        assertTrue(getActiveGoal(goal.getId()) == goal || getPastGoal(goal.getId()) == goal);
      }
    }

    @Override
    protected void onActiveGoalUpdated(Goal goal, GoalStatus status, UpdateType type) {
      if (type == UpdateType.REMOVED) {
        removals++;
        assertNull(getActiveGoal(goal.getId()));
        assertSame(goal, getPastGoal(goal.getId()));
      }
    }

    @Override
    protected boolean executeGoal(Goal goal) {
      executions++;
      getAgentTeam(SELF).addGoal(goal, new CompletableFuture<>());
      goal.setAsStarted();
      return true;
    }
  }

  private static void initialize(ExecutionManager manager) {
    manager.rootAgent = SELF;
    manager.agentHierarchy = Map.of(SELF, Set.of(SELF));
    manager.agentTeams.put(SELF, new AgentTeam(SELF, manager));
  }

  private static class LearningTeam extends AgentTeam {
    int handoffs;

    LearningTeam(ExecutionManager manager) {
      super(SELF, manager);
    }

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
      handoffs++;
    }
  }

  private Goal goal(Set<Resource> required, Set<Resource> held) {
    Goal goal = Goal.createGoal(Factory.createPredicate("test(self)"));
    goal.setRequiredResources(required);
    goal.setHeldResources(held);
    return goal;
  }

  private Resource resource(String name) {
    return new Resource(null, Factory.createSymbol(name));
  }

  @Test
  public void pendingCompletionIsCommittedBeforeExternalNotification() {
    TestManager manager = new TestManager();
    Goal goal = goal(Set.of(), Set.of());
    AtomicInteger notifications = new AtomicInteger();
    goal.addListener(info -> {
      assertNull(manager.getPendingGoal(goal.getId()));
      assertSame(goal, manager.getPastGoal(goal.getId()));
      notifications.incrementAndGet();
    });
    manager.submitGoal(goal);
    goal.setStatus(GoalStatus.CANCELED);
    assertEquals(1, notifications.get());
    assertEquals(0, manager.executions);
  }

  @Test(timeout = 3000)
  public void concurrentDuplicateSubmissionAndTerminalResubmissionAreIdempotent() throws Exception {
    TestManager manager = new TestManager();
    Goal goal = goal(Set.of(), Set.of());
    ExecutorService executor = Executors.newFixedThreadPool(2);
    try {
      Future<Goal> first = executor.submit(() -> manager.submitGoal(goal));
      Future<Goal> second = executor.submit(() -> manager.submitGoal(goal));
      assertSame(goal, first.get(1, TimeUnit.SECONDS));
      assertSame(goal, second.get(1, TimeUnit.SECONDS));
      assertEquals(1, manager.admissions);
      assertEquals(1, manager.getPendingGoals().size());
      goal.setStatus(GoalStatus.CANCELED);
      assertSame(goal, manager.submitGoal(goal));
      assertEquals(1, manager.admissions);
      assertTrue(manager.getPendingGoals().isEmpty());
    } finally {
      executor.shutdownNow();
    }
  }

  @Test
  public void invalidActorIsInHistoryBeforeFailureIsPublished() {
    TestManager manager = new TestManager();
    Goal goal = Goal.createGoal(Factory.createPredicate("test(missing)"));
    goal.addListener(info -> assertSame(goal, manager.getPastGoal(goal.getId())));
    manager.submitGoal(goal);
    assertEquals(GoalStatus.FAILED, goal.getStatus());
    assertSame(goal, manager.getGoal(goal.getId()));
  }

  @Test
  public void requiredUsersShareButBlockExclusiveClaimsUntilBothFinish() {
    TestManager manager = new TestManager();
    Resource resource = resource("shared");
    Goal first = goal(Set.of(resource), Set.of());
    Goal second = goal(Set.of(resource), Set.of());
    Goal writer = goal(Set.of(), Set.of(resource));
    manager.submitGoal(first);
    manager.submitGoal(second);
    manager.activateValidPendingGoals();
    assertEquals(Set.of(first, second), resource.getUsers());
    assertEquals(Set.of(first, second), manager.getResourceConflictingActiveGoals(writer));
    manager.submitGoal(writer);
    manager.activateValidPendingGoals();
    assertNotNull(manager.getPendingGoal(writer.getId()));
    first.setStatus(GoalStatus.SUCCEEDED);
    manager.activateValidPendingGoals();
    assertNotNull(manager.getPendingGoal(writer.getId()));
    second.setStatus(GoalStatus.SUCCEEDED);
    manager.activateValidPendingGoals();
    assertSame(writer, resource.getHolder());
    assertSame(writer, manager.getActiveGoal(writer.getId()));
  }

  @Test
  public void failedAllocationDoesNotAcquireSubsetOrStartExecution() {
    TestManager manager = new TestManager();
    Resource free = resource("free");
    Resource occupied = resource("occupied");
    Goal holder = goal(Set.of(), Set.of(occupied));
    occupied.setHolder(holder);
    Goal candidate = goal(Set.of(), Set.of(free, occupied));
    manager.submitGoal(candidate);
    manager.transferGoalToActive(candidate);
    assertTrue(free.isAvailable());
    assertTrue(free.getUsers().isEmpty());
    assertSame(holder, occupied.getHolder());
    assertNotNull(manager.getPendingGoal(candidate.getId()));
    assertEquals(0, manager.executions);
  }

  @Test
  public void releaseUsesAcquiredSnapshotAndDuplicateCompletionCannotReleaseNewOwner() {
    TestManager manager = new TestManager();
    Resource original = resource("original");
    Resource other = resource("other");
    Goal goal = goal(Set.of(original), Set.of(original));
    manager.submitGoal(goal);
    manager.activateValidPendingGoals();
    Goal otherOwner = goal(Set.of(), Set.of(other));
    other.setHolder(otherOwner);
    goal.setRequiredResources(Set.of(other));
    goal.setHeldResources(Set.of(other));
    goal.addListener(info -> {
      assertTrue(original.isAvailable());
      assertNull(manager.getActiveGoal(goal.getId()));
      assertSame(goal, manager.getPastGoal(goal.getId()));
    });
    goal.setStatus(GoalStatus.SUCCEEDED);
    assertSame(otherOwner, other.getHolder());
    Goal next = goal(Set.of(original), Set.of(original));
    manager.submitGoal(next);
    manager.activateValidPendingGoals();
    manager.goalStatusChanged(goal.getInfo());
    assertSame(next, original.getHolder());
    assertEquals(1, manager.removals);
  }

  @Test
  public void pendingPrefixScanSkipsActualEntriesAndHonorsHigherPriorityClaims() {
    TestManager manager = new TestManager();
    Resource blocked = resource("blocked");
    Goal holder = goal(Set.of(), Set.of(blocked));
    blocked.setHolder(holder);
    Goal first = goal(Set.of(blocked), Set.of());
    first.setPriority(10);
    Goal second = goal(Set.of(), Set.of());
    manager.submitGoal(first);
    manager.submitGoal(second);
    assertEquals(1, manager.activateNextValidPendingGoal(1));
    assertSame(second, manager.getActiveGoal(second.getId()));
    assertNotNull(manager.getPendingGoal(first.getId()));
  }

  @Test
  public void resourceIdentityIncludesProviderNotJustName() {
    Resource first = resource("same");
    Resource second = resource("same");
    assertFalse(new ResourceClaims(Set.of(), Set.of(first))
            .conflictsWith(new ResourceClaims(Set.of(), Set.of(second))));
    assertTrue(new ResourceClaims(Set.of(first), Set.of())
            .conflictsWith(new ResourceClaims(Set.of(), Set.of(first))));
    assertFalse(new ResourceClaims(Set.of(first), Set.of())
            .conflictsWith(new ResourceClaims(Set.of(first), Set.of())));
  }

  @Test
  public void singleGoalClaimsDoNotDependOnGetterOrderOrAliasCaches() {
    SingleGoalExecutionManager manager = new SingleGoalExecutionManager();
    initialize(manager);
    Goal first = Goal.createGoal(Factory.createPredicate("test(self)"));
    Set<Resource> held = manager.getHeldResourcesForGoal(first);
    Set<Resource> required = manager.getRequiredResourcesForGoal(first);
    assertEquals(held, required);
    assertNotSame(held, required);
    assertTrue(required.contains(manager.getAgentTeam(SELF).getResource(SELF)));
    Goal second = Goal.createGoal(Factory.createPredicate("test(self)"));
    assertEquals(required, manager.getRequiredResourcesForGoal(second));
    assertEquals(held, manager.getHeldResourcesForGoal(second));
  }

  @Test
  public void normalMetricAndScheduledSubmissionsSharePreparationWithoutTradeLookup() {
    TestManager manager = new TestManager();
    manager.configureGoalPriorities(predicate -> PriorityTier.URGENT, predicate -> 42L);
    Predicate predicate = Factory.createPredicate("test(self)");
    Predicate metric = Factory.createPredicate("minimize(cost)");
    Symbol submitter = Factory.createSymbol("speaker");
    Goal normal = manager.submitGoal(predicate, ExecutionType.SIMULATE, null, null, submitter, null);
    Goal measured = manager.submitGoal(predicate, ExecutionType.SIMULATE, null, null, submitter, metric);
    assertEquals(normal.getPriority(), measured.getPriority());
    assertEquals(normal.getPriorityTier(), measured.getPriorityTier());
    assertEquals(42, measured.getPriority());
    assertEquals(metric, measured.getMetric());
    ScheduledGoal schedule = new ScheduledGoal(Factory.createPredicate("scheduled(test(self),period(1,DAYS))"));
    schedule.setExecutionType(ExecutionType.SIMULATE);
    schedule.setSubmitter(submitter);
    schedule.setMetric(metric);
    Goal occurrence = schedule.getNextGoalToSubmit();
    manager.submitScheduledOccurrence(schedule, occurrence);
    assertEquals(42, occurrence.getPriority());
    assertEquals(PriorityTier.LOW, occurrence.getPriorityTier());
    assertEquals(ExecutionType.SIMULATE, occurrence.getExecutionType());
    assertEquals(submitter, occurrence.getSubmitter());
    assertEquals(metric, occurrence.getMetric());
    assertEquals(2, occurrence.getScheduledState());
    assertSame(occurrence, manager.getGoal(occurrence.getId()));
  }

  @Test
  public void explicitPrioritiesAndSkipPendingPolicyArePreserved() {
    TestManager manager = new TestManager();
    manager.configureGoalPriorities(predicate -> PriorityTier.SKIPPENDING, predicate -> 17L);
    Goal direct = goal(Set.of(), Set.of());
    direct.setPriorityTier(PriorityTier.HIGH);
    direct.setPriority(99);
    manager.submitGoal(direct);
    assertEquals(PriorityTier.HIGH, direct.getPriorityTier());
    assertEquals(99, direct.getPriority());
    ScheduledGoal schedule = new ScheduledGoal(Factory.createPredicate("scheduled(test(self),period(1,DAYS))"));
    Goal occurrence = schedule.getNextGoalToSubmit();
    manager.submitScheduledOccurrence(schedule, occurrence);
    assertEquals(PriorityTier.SKIPPENDING, occurrence.getPriorityTier());
    Goal explicit = manager.submitGoal(Factory.createPredicate("test(self)"), ExecutionType.ACT,
            PriorityTier.HIGH, 100L, SELF, null);
    assertEquals(PriorityTier.HIGH, explicit.getPriorityTier());
    assertEquals(100, explicit.getPriority());
  }

  @Test
  public void failedScheduledPreparationIsVisibleAndDoesNotBlockFutureOccurrences() {
    TestManager manager = new TestManager();
    manager.configureGoalPriorities(predicate -> {
      throw new IllegalStateException("Priority configuration failed");
    }, predicate -> 1L);
    ScheduledGoal schedule = new ScheduledGoal(Factory.createPredicate("scheduled(test(self),period(1,DAYS))"));
    Goal occurrence = schedule.getNextGoalToSubmit();
    try {
      manager.submitScheduledOccurrence(schedule, occurrence);
      fail("Expected priority preparation failure");
    } catch (IllegalStateException expected) {
      assertEquals("Priority configuration failed", expected.getMessage());
    }
    assertEquals(GoalStatus.FAILED, occurrence.getStatus());
    assertSame(occurrence, manager.getPastGoal(occurrence.getId()));
    assertTrue(manager.getPendingGoals().isEmpty());
    assertNotNull(schedule.getNextGoalToSubmit());
  }

  @Test
  public void learningHandoffCanReturnToExecutionExactlyOnce() {
    TestManager manager = new TestManager();
    LearningTeam team = new LearningTeam(manager);
    manager.agentTeams.put(SELF, team);
    Goal goal = goal(Set.of(), Set.of());
    manager.submitGoal(goal);
    manager.submitGoal(goal);
    assertEquals(1, team.handoffs);
    assertEquals(0, manager.admissions);
    assertNull(manager.getPastGoal(goal.getId()));
    assertSame(goal, manager.getGoal(goal.getId()));
    goal.setIsLearningGoal(true);
    manager.submitGoal(goal);
    manager.submitGoal(goal);
    assertEquals(1, team.handoffs);
    assertEquals(1, manager.admissions);
    manager.activateValidPendingGoals();
    assertEquals(1, manager.executions);
  }

  @Test
  public void canceledLearningHandoffCannotReturnToExecution() {
    TestManager manager = new TestManager();
    manager.agentTeams.put(SELF, new LearningTeam(manager));
    Goal goal = goal(Set.of(), Set.of());
    manager.submitGoal(goal);
    assertTrue(manager.cancelGoal(goal.getId()));
    assertEquals(GoalStatus.CANCELED, goal.getStatus());
    assertSame(goal, manager.getPastGoal(goal.getId()));
    goal.setIsLearningGoal(true);
    manager.submitGoal(goal);
    assertEquals(0, manager.admissions);
  }

  @Test(timeout = 2000)
  public void startupFailureReleasesClaimsAndNotifiesRemovedPendingEntry() {
    TestManager manager = new TestManager() {
      @Override
      protected boolean executeGoal(Goal goal) {
        throw new IllegalStateException("Startup failed");
      }
    };
    Resource resource = resource("startup");
    Goal goal = goal(Set.of(), Set.of(resource));
    manager.submitGoal(goal);
    PendingGoal pending = manager.getPendingGoal(goal.getId());
    try {
      manager.transferGoalToActive(goal);
      fail("Expected startup failure");
    } catch (IllegalStateException expected) {
      assertEquals("Startup failed", expected.getMessage());
    }
    pending.waitForNoLongerPending();
    assertEquals(GoalStatus.FAILED, goal.getStatus());
    assertSame(goal, manager.getPastGoal(goal.getId()));
    assertTrue(resource.isAvailable());
    assertTrue(resource.getUsers().isEmpty());
  }

  @Test
  public void requiredOnlyCompletionAutomaticallyUnblocksExclusiveWaiter() {
    QueueExecutionManager manager = new QueueExecutionManager() {
      @Override
      protected boolean executeGoal(Goal goal) {
        getAgentTeam(SELF).addGoal(goal, new CompletableFuture<>());
        goal.setAsStarted();
        return true;
      }
    };
    initialize(manager);
    Resource resource = resource("shared");
    Goal reader = goal(Set.of(resource), Set.of());
    Goal writer = goal(Set.of(), Set.of(resource));
    manager.submitGoal(reader);
    manager.submitGoal(writer);
    assertEquals(GoalStatus.PENDING, writer.getStatus());
    reader.setRequiredResources(Set.of());
    reader.setStatus(GoalStatus.SUCCEEDED);
    assertEquals(GoalStatus.ACTIVE, writer.getStatus());
    assertSame(writer, resource.getHolder());
    assertNull(manager.getPendingGoal(writer.getId()));
  }

  @Test
  public void rejectedSuspensionDoesNotReleaseResourcesOrStartReplacement() {
    QueueExecutionManager manager = new QueueExecutionManager() {
      @Override
      protected boolean executeGoal(Goal goal) {
        getAgentTeam(SELF).addGoal(goal, new CompletableFuture<>());
        goal.setAsStarted();
        return true;
      }
    };
    initialize(manager);
    Resource resource = resource("uninterruptible");
    Goal running = new Goal(Factory.createPredicate("test(self)")) {
      @Override
      public boolean suspend() {
        return false;
      }
    };
    running.setRequiredResources(Set.of());
    running.setHeldResources(Set.of(resource));
    Goal replacement = goal(Set.of(), Set.of(resource));
    replacement.setPriority(100);
    manager.submitGoal(running);
    manager.submitGoal(replacement);
    assertSame(running, resource.getHolder());
    assertSame(running, manager.getActiveGoal(running.getId()));
    assertNotNull(manager.getPendingGoal(replacement.getId()));
    assertEquals(GoalStatus.PENDING, replacement.getStatus());
    running.setStatus(GoalStatus.SUCCEEDED);
    assertSame(replacement, resource.getHolder());
    assertEquals(GoalStatus.ACTIVE, replacement.getStatus());
  }

  @Test
  public void completionDuringOwnershipRegistrationAlreadyHasDiscoverableGoal() {
    TestManager manager = new TestManager();
    Goal goal = new Goal(Factory.createPredicate("test(self)")) {
      @Override
      public void setLifecycleListener(GoalListener listener) {
        super.setLifecycleListener(listener);
        setStatus(GoalStatus.SUCCEEDED);
      }
    };
    goal.addListener(info -> assertSame(goal, manager.getPastGoal(goal.getId())));
    manager.submitGoal(goal);
    assertSame(goal, manager.getPastGoal(goal.getId()));
    assertTrue(manager.getPendingGoals().isEmpty());
    assertEquals(0, manager.admissions);
  }

  @Test
  public void rejectedSecondOwnerDoesNotRetainGoalInItsRegistry() {
    TestManager owner = new TestManager();
    TestManager other = new TestManager();
    Goal goal = goal(Set.of(), Set.of());
    owner.submitGoal(goal);
    try {
      other.submitGoal(goal);
      fail("Expected ownership rejection");
    } catch (IllegalStateException expected) {
      assertTrue(expected.getMessage().contains("lifecycle manager"));
    }
    assertNull(other.getGoal(goal.getId()));
    assertSame(goal, owner.getGoal(goal.getId()));
  }

  @Test
  public void invalidExecutionTreeDoesNotLeaveARegisteredUnfinishedGoal() {
    TestManager manager = new TestManager();
    Goal goal = goal(Set.of(), Set.of());
    try {
      manager.submitGoalContext(goal, null);
      fail("Expected invalid execution tree to fail");
    } catch (NullPointerException expected) {
      assertEquals(GoalStatus.FAILED, goal.getStatus());
    }
    assertSame(goal, manager.getPastGoal(goal.getId()));
    assertNull(manager.getActiveGoal(goal.getId()));
    assertTrue(manager.getPendingGoals().isEmpty());
  }
}
