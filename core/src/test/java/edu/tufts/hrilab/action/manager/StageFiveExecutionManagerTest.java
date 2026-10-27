package edu.tufts.hrilab.action.manager;

import edu.tufts.hrilab.action.ActionConstraints;
import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.action.execution.RootContext;
import edu.tufts.hrilab.action.goal.*;
import edu.tufts.hrilab.action.goal.schedule.ScheduledGoal;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import org.junit.Test;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.Assert.*;

public class StageFiveExecutionManagerTest {
  private static final Symbol SELF = Factory.createSymbol("self");

  static class TestManager extends QueueExecutionManager {
    final AtomicInteger executions = new AtomicInteger();

    TestManager() {
      rootAgent = SELF;
      agentHierarchy = Map.of(SELF, Set.of(SELF));
      agentTeams.put(SELF, new AgentTeam(SELF, this));
    }

    @Override
    protected boolean executeGoal(Goal goal) {
      executions.incrementAndGet();
      goal.setAsStarted();
      return true;
    }
  }

  static void installInfrastructure(ExecutionManager manager) throws Exception {
    StateMachine state = StateMachine.createTestStateMachine(Set.of());
    setField(manager, "sm", state);
    setField(manager, "rootContext", new RootContext(new ActionConstraints(), state));
    setField(manager, "scheduledGoalManager", new ScheduledGoalManager(manager));
  }

  static void setField(Object target, String name, Object value) throws Exception {
    Field field = ExecutionManager.class.getDeclaredField(name);
    field.setAccessible(true);
    field.set(target, value);
  }

  static Goal goal(Resource resource, long priority) {
    Goal goal = Goal.createGoal(Factory.createPredicate("test(self)"));
    goal.setPriority(priority);
    goal.setRequiredResources(Set.of());
    goal.setHeldResources(resource == null ? Set.of() : Set.of(resource));
    return goal;
  }

  private Resource resource() {
    return new Resource(null, Factory.createSymbol("shared"));
  }

  @Test
  public void admissionLimitIsAtomicAndDuplicateSubmissionDoesNotConsumeCapacity() {
    TestManager manager = new TestManager();
    manager.configureGoalLimits(2, 0);
    Resource resource = resource();
    Goal first = goal(resource, 100);
    Goal second = goal(resource, 1);
    manager.submitGoal(first);
    manager.submitGoal(second);
    assertSame(first, manager.submitGoal(first));
    Goal rejected = goal(resource, 1);
    try {
      manager.submitGoal(rejected);
      fail("Expected admission rejection");
    } catch (RejectedExecutionException expected) {
      assertTrue(expected.getMessage().contains("limit"));
    }
    assertNull(manager.getGoal(rejected.getId()));
    assertEquals(GoalStatus.PENDING, rejected.getStatus());
    assertSame(first, resource.getHolder());
    assertTrue(manager.cancelGoal(second.getId()));
    manager.submitGoal(rejected);
    assertNotNull(manager.getPendingGoal(rejected.getId()));
  }

  @Test(timeout = 5000)
  public void concurrentAdmissionCannotOvershootConfiguredCapacity() throws Exception {
    TestManager manager = new TestManager();
    manager.configureGoalLimits(3, 0);
    Resource resource = resource();
    ExecutorService callers = Executors.newFixedThreadPool(6);
    try {
      List<Future<Boolean>> results = new ArrayList<>();
      for (int i = 0; i < 30; i++) {
        results.add(callers.submit(() -> {
          try {
            manager.submitGoal(goal(resource, 1));
            return true;
          } catch (RejectedExecutionException expected) {
            return false;
          }
        }));
      }
      int accepted = 0;
      for (Future<Boolean> result : results) {
        if (result.get(2, TimeUnit.SECONDS)) {
          accepted++;
        }
      }
      assertEquals(3, accepted);
      assertEquals(2, manager.getPendingGoals().size());
      assertEquals(1, manager.getAgentTeam(SELF).getActiveGoals().size());
      assertEquals(Set.of(resource.getHolder()), resource.getUsers());
    } finally {
      callers.shutdownNow();
    }
  }

  @Test
  public void contextSubmissionCannotBypassAdmissionLimit() {
    TestManager manager = new TestManager();
    manager.configureGoalLimits(1, 0);
    manager.submitGoal(goal(null, 1));
    Goal rejected = goal(null, 1);
    try {
      manager.submitGoalContext(rejected, null);
      fail("Expected context admission rejection");
    } catch (RejectedExecutionException expected) {
      assertNull(manager.getGoal(rejected.getId()));
    }
  }

  @Test(timeout = 3000)
  public void queuedContinuationReservesCapacityBeforeItIsAdmitted() throws Exception {
    TestManager manager = new TestManager();
    manager.configureGoalLimits(1, 0);
    Goal reserved = goal(null, 1);
    CountDownLatch entered = new CountDownLatch(1);
    CountDownLatch release = new CountDownLatch(1);
    manager.runScheduledContinuation(reserved, () -> {
      entered.countDown();
      try {
        assertTrue(release.await(2, TimeUnit.SECONDS));
      } catch (InterruptedException e) {
        throw new AssertionError(e);
      }
    });
    try {
      assertTrue(entered.await(1, TimeUnit.SECONDS));
      try {
        manager.submitGoal(goal(null, 1));
        fail("Queued continuation must reserve capacity");
      } catch (RejectedExecutionException expected) {
        assertEquals(0, manager.executions.get());
      }
    } finally {
      release.countDown();
    }
    assertEquals(GoalStatus.UNKNOWN, manager.joinOnGoal(reserved.getId(), 1000));
    manager.submitGoal(goal(null, 1));
    assertEquals(1, manager.executions.get());
  }

  @Test
  public void rejectedContinuationIsTerminalAndDoesNotLeaveAnUnboundedReservation() {
    TestManager manager = new TestManager();
    manager.configureGoalLimits(1, 0);
    manager.submitGoal(goal(null, 1));
    Goal rejected = goal(null, 1);
    manager.runScheduledContinuation(rejected, () -> fail("Rejected continuation ran"));
    assertEquals(GoalStatus.FAILED, rejected.getStatus());
    assertNull(manager.getGoal(rejected.getId()));
  }

  @Test(timeout = 5000)
  public void scheduledOccurrencesRespectLimitsWithoutStrandingTheDefinition() throws Exception {
    TestManager manager = new TestManager();
    installInfrastructure(manager);
    try {
      manager.configureGoalLimits(1, 0);
      ScheduledGoal definition = new ScheduledGoal(
              Factory.createPredicate("scheduled(test(self),start(1,DAYS),period(1,DAYS))"));
      manager.submitGoal(definition);
      Goal occurrence = definition.getNextGoalToSubmit();
      manager.submitScheduledOccurrence(definition, occurrence);
      assertEquals(GoalStatus.FAILED, occurrence.getStatus());
      assertEquals(GoalStatus.ACTIVE, definition.getStatus());
      assertNotNull(definition.getNextGoalToSubmit());
      assertNull(manager.getGoal(occurrence.getId()));
    } finally {
      manager.shutdown();
    }
  }

  @Test
  public void countBoundEvictsOldestEligibleHistoryButKeepsLiveGoalsAndNewestResults() {
    TestManager manager = new TestManager();
    manager.configureGoalLimits(0, 2);
    Goal live = goal(null, 1);
    manager.submitGoal(live);
    List<Goal> completed = new ArrayList<>();
    for (int i = 0; i < 5; i++) {
      Goal goal = goal(null, 1);
      manager.submitGoal(goal);
      goal.setStatus(GoalStatus.SUCCEEDED);
      completed.add(goal);
    }
    assertEquals(2, manager.getPastGoals().size());
    assertNull(manager.getGoal(completed.get(0).getId()));
    assertSame(completed.get(4), manager.getGoal(completed.get(4).getId()));
    assertSame(live, manager.getActiveGoal(live.getId()));
    manager.configureGoalLimits(0, 1);
    assertEquals(1, manager.getPastGoals().size());
  }

  @Test
  public void reducingLiveLimitDoesNotCancelExistingWork() {
    TestManager manager = new TestManager();
    Goal first = goal(null, 1);
    Goal second = goal(null, 1);
    manager.submitGoal(first);
    manager.submitGoal(second);
    manager.configureGoalLimits(1, 0);
    assertEquals(GoalStatus.ACTIVE, first.getStatus());
    assertEquals(GoalStatus.ACTIVE, second.getStatus());
    try {
      manager.submitGoal(goal(null, 1));
      fail("Expected reduced admission limit");
    } catch (RejectedExecutionException expected) {
      assertEquals(2, manager.executions.get());
    }
  }

  @Test(timeout = 5000)
  public void shutdownTimeoutPreservesClaimsUntilAnExplicitLateAcknowledgment() throws Exception {
    TestManager manager = new TestManager();
    installInfrastructure(manager);
    Resource resource = resource();
    Goal uncooperative = new Goal(Factory.createPredicate("test(self)")) {
      @Override
      public boolean cancel() {
        return true;
      }
    };
    uncooperative.setRequiredResources(Set.of());
    uncooperative.setHeldResources(Set.of(resource));
    manager.submitGoal(uncooperative);
    try {
      long start = System.nanoTime();
      GoalManagerShutdownReport report = manager.shutdownWithTimeout(20, 20);
      assertTrue(TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - start) < 1500);
      assertEquals(GoalManagerShutdownReport.Outcome.TIMED_OUT, report.outcome);
      assertTrue(report.interruptionRequested);
      assertEquals(1, report.getUnfinishedGoals().size());
      report.getUnfinishedGoals().get(0).status = GoalStatus.FAILED;
      assertEquals(GoalStatus.ACTIVE, uncooperative.getStatus());
      assertSame(uncooperative, resource.getHolder());
      try {
        manager.submitGoal(goal(null, 1));
        fail("Shutdown must close admission");
      } catch (RejectedExecutionException expected) {
        assertSame(uncooperative, resource.getHolder());
      }
    } finally {
      uncooperative.setStatus(GoalStatus.CANCELED);
      assertEquals(GoalManagerShutdownReport.Outcome.COMPLETE,
              manager.shutdownWithTimeout(2000, 0).outcome);
    }
    assertTrue(resource.isAvailable());
  }

  @Test(timeout = 5000)
  public void escalationInterruptsCancellationHooksWithoutCompletingTheirWorkPrematurely() throws Exception {
    TestManager manager = new TestManager();
    installInfrastructure(manager);
    CountDownLatch entered = new CountDownLatch(1);
    CountDownLatch interrupted = new CountDownLatch(1);
    CountDownLatch release = new CountDownLatch(1);
    Resource resource = resource();
    Goal blocked = new Goal(Factory.createPredicate("test(self)")) {
      @Override
      public boolean cancel() {
        entered.countDown();
        boolean done = false;
        while (!done) {
          try {
            release.await();
            done = true;
          } catch (InterruptedException e) {
            interrupted.countDown();
          }
        }
        setStatus(GoalStatus.CANCELED);
        return true;
      }
    };
    blocked.setRequiredResources(Set.of());
    blocked.setHeldResources(Set.of(resource));
    manager.submitGoal(blocked);
    try {
      GoalManagerShutdownReport report = manager.shutdownWithTimeout(100, 100);
      assertTrue(entered.await(1, TimeUnit.SECONDS));
      assertTrue(interrupted.await(1, TimeUnit.SECONDS));
      assertEquals(GoalManagerShutdownReport.Outcome.TIMED_OUT, report.outcome);
      assertSame(blocked, resource.getHolder());
      assertEquals(GoalStatus.ACTIVE, blocked.getStatus());
    } finally {
      release.countDown();
      assertEquals(GoalManagerShutdownReport.Outcome.COMPLETE,
              manager.shutdownWithTimeout(2000, 0).outcome);
    }
  }

  @Test(timeout = 3000)
  public void workerInterruptionIsNotFutureCancellationOrACompletionAcknowledgment() throws Exception {
    GoalManagerExecutor executor = new GoalManagerExecutor();
    CountDownLatch entered = new CountDownLatch(1);
    CountDownLatch interrupted = new CountDownLatch(1);
    CountDownLatch release = new CountDownLatch(1);
    Future<?> future = executor.submit(() -> {
      entered.countDown();
      boolean done = false;
      while (!done) {
        try {
          release.await();
          done = true;
        } catch (InterruptedException e) {
          interrupted.countDown();
        }
      }
    });
    try {
      assertTrue(entered.await(1, TimeUnit.SECONDS));
      executor.requestInterruption();
      assertTrue(interrupted.await(1, TimeUnit.SECONDS));
      assertFalse(future.isDone());
      assertFalse(future.isCancelled());
    } finally {
      release.countDown();
      future.get(1, TimeUnit.SECONDS);
      executor.shutdown();
    }
  }

  @Test(timeout = 5000)
  public void componentDoesNotJoinBeforeApplyingItsConfiguredShutdownBudget() throws Exception {
    TestManager manager = new TestManager();
    installInfrastructure(manager);
    Goal goal = new Goal(Factory.createPredicate("test(self)")) {
      @Override
      public boolean cancel() {
        return true;
      }
    };
    goal.setRequiredResources(Set.of());
    goal.setHeldResources(Set.of());
    manager.submitGoal(goal);
    GoalManagerComponent component = new GoalManagerComponent();
    Field field = GoalManagerComponent.class.getDeclaredField("em");
    field.setAccessible(true);
    field.set(component, manager);
    component.configureShutdownTimeouts(20, 20);
    try {
      component.shutdownComponent();
      assertEquals(GoalStatus.ACTIVE, goal.getStatus());
    } finally {
      goal.setStatus(GoalStatus.CANCELED);
      assertEquals(GoalManagerShutdownReport.Outcome.COMPLETE,
              manager.shutdownWithTimeout(2000, 0).outcome);
    }
  }

  @Test(expected = IllegalArgumentException.class)
  public void negativeGoalBoundsAreRejected() {
    new TestManager().configureGoalLimits(-1, 0);
  }

  @Test(expected = IllegalArgumentException.class)
  public void negativeShutdownBudgetsAreRejectedWithoutStartingShutdown() {
    new TestManager().shutdownWithTimeout(0, -1);
  }

  @Test(timeout = 5000)
  public void escalationInterruptsSlowStartupButCannotDispatchItAfterShutdown() throws Exception {
    CountDownLatch entered = new CountDownLatch(1);
    CountDownLatch interrupted = new CountDownLatch(1);
    CountDownLatch release = new CountDownLatch(1);
    QueueExecutionManager manager = new QueueExecutionManager() {
      @Override
      protected boolean executeGoal(Goal goal) {
        entered.countDown();
        boolean released = false;
        while (!released) {
          try {
            release.await();
            released = true;
          } catch (InterruptedException e) {
            interrupted.countDown();
          }
        }
        return super.executeGoal(goal);
      }
    };
    manager.rootAgent = SELF;
    manager.agentHierarchy = Map.of(SELF, Set.of(SELF));
    manager.agentTeams.put(SELF, new AgentTeam(SELF, manager));
    installInfrastructure(manager);
    Resource resource = resource();
    Goal goal = goal(resource, 1);
    ExecutorService worker = Executors.newSingleThreadExecutor();
    try {
      Future<?> submitted = worker.submit(() -> manager.submitGoal(goal));
      assertTrue(entered.await(1, TimeUnit.SECONDS));
      assertEquals(GoalManagerShutdownReport.Outcome.TIMED_OUT,
              manager.shutdownWithTimeout(20, 20).outcome);
      assertTrue(interrupted.await(1, TimeUnit.SECONDS));
      assertFalse(submitted.isDone());
      assertSame(goal, resource.getHolder());
      release.countDown();
      submitted.get(2, TimeUnit.SECONDS);
      assertEquals(GoalStatus.CANCELED, goal.getStatus());
      assertEquals(-1, goal.getStartTime());
      assertTrue(resource.isAvailable());
    } finally {
      release.countDown();
      worker.shutdownNow();
      assertEquals(GoalManagerShutdownReport.Outcome.COMPLETE,
              manager.shutdownWithTimeout(2000, 0).outcome);
    }
  }

  @Test(timeout = 3000)
  public void historyBoundNeverEvictsATerminalGoalWhoseStartupIsStillRunning() throws Exception {
    CountDownLatch entered = new CountDownLatch(1);
    CountDownLatch release = new CountDownLatch(1);
    Goal slow = goal(null, 1);
    TestManager manager = new TestManager() {
      @Override
      protected boolean executeGoal(Goal goal) {
        if (goal == slow) {
          entered.countDown();
          try {
            assertTrue(release.await(2, TimeUnit.SECONDS));
          } catch (InterruptedException e) {
            throw new AssertionError(e);
          }
        }
        return super.executeGoal(goal);
      }
    };
    manager.configureGoalLimits(0, 1);
    ExecutorService worker = Executors.newSingleThreadExecutor();
    try {
      Future<?> submitted = worker.submit(() -> manager.submitGoal(slow));
      assertTrue(entered.await(1, TimeUnit.SECONDS));
      slow.setStatus(GoalStatus.FAILED);
      Goal finished = goal(null, 1);
      manager.submitGoal(finished);
      finished.setStatus(GoalStatus.SUCCEEDED);
      assertSame(slow, manager.getGoal(slow.getId()));
      assertEquals(GoalStatus.FAILED, slow.getStatus());
      release.countDown();
      submitted.get(1, TimeUnit.SECONDS);
      assertTrue(manager.getPastGoals().size() <= 1);
    } finally {
      release.countDown();
      worker.shutdownNow();
    }
  }
}
