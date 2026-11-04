package edu.tufts.hrilab.action.manager;

import edu.tufts.hrilab.action.ActionConstraints;
import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.execution.RootContext;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalInfo;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.goal.PendingGoal;
import edu.tufts.hrilab.fol.Factory;
import org.junit.Test;

import java.lang.reflect.Field;
import java.util.List;
import java.util.Set;
import java.util.concurrent.AbstractExecutorService;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.RejectedExecutionException;
import java.util.concurrent.TimeUnit;

import static org.junit.Assert.*;

public class ExecutionManagerRegressionTest {
  private static class InlineExecutor extends AbstractExecutorService {
    private boolean shutdown;

    @Override
    public void execute(Runnable command) {
      command.run();
    }

    @Override
    public void shutdown() {
      shutdown = true;
    }

    @Override
    public List<Runnable> shutdownNow() {
      shutdown();
      return List.of();
    }

    @Override
    public boolean isShutdown() {
      return shutdown;
    }

    @Override
    public boolean isTerminated() {
      return shutdown;
    }

    @Override
    public boolean awaitTermination(long timeout, TimeUnit unit) {
      return shutdown;
    }
  }

  private static class TestManager extends ExecutionManager {
    Goal knownGoal;
    CountDownLatch transferring;

    @Override
    public Goal getGoal(long gid) {
      return knownGoal != null && knownGoal.getId() == gid ? knownGoal : null;
    }

    @Override
    protected void onPendingGoalUpdated(Goal goal, int index, UpdateType type) {
    }

    @Override
    protected void transferGoalToPastGoals(Goal goal) {
      if (transferring != null) {
        transferring.countDown();
      }
      super.transferGoalToPastGoals(goal);
    }
  }

  private Goal goal(long priority, Set<Resource> required, Set<Resource> held) {
    Goal goal = Goal.createGoal(Factory.createSymbol("self"), Factory.createPredicate("regression(self)"));
    goal.setPriority(priority);
    goal.setRequiredResources(required);
    goal.setHeldResources(held);
    return goal;
  }

  @Test
  public void pendingScanExcludesCandidateAndLowerPriorityGoals() {
    TestManager manager = new TestManager();
    Resource first = new Resource(null, Factory.createSymbol("first"));
    Resource second = new Resource(null, Factory.createSymbol("second"));
    Goal higher = goal(10, Set.of(first), Set.of(first));
    Goal candidate = goal(5, Set.of(second), Set.of(second));
    Goal lower = goal(1, Set.of(second), Set.of(second));
    manager.pendingGoals.add(new PendingGoal(higher));
    manager.pendingGoals.add(new PendingGoal(candidate));
    manager.pendingGoals.add(new PendingGoal(lower));
    assertFalse(manager.resourceConflictInPending(candidate, 0));
    assertFalse(manager.resourceConflictInPending(candidate, 1));
  }

  @Test
  public void pendingConflictsUseHeldResourcesInBothDirections() {
    TestManager manager = new TestManager();
    Resource resource = new Resource(null, Factory.createSymbol("shared"));
    Goal higher = goal(10, Set.of(resource), Set.of());
    manager.pendingGoals.add(new PendingGoal(higher));
    assertFalse(manager.resourceConflictInPending(goal(1, Set.of(resource), Set.of()), 1));
    assertTrue(manager.resourceConflictInPending(goal(1, Set.of(), Set.of(resource)), 1));
    manager.pendingGoals.clear();
    manager.pendingGoals.add(new PendingGoal(goal(10, Set.of(), Set.of(resource))));
    assertTrue(manager.resourceConflictInPending(goal(1, Set.of(resource), Set.of()), 1));
    assertTrue(manager.resourceConflictInPending(goal(1, Set.of(), Set.of(resource)), 1));
  }

  @Test(timeout = 2000)
  public void statusWaitUsesMilliseconds() {
    TestManager manager = new TestManager();
    Goal goal = goal(1, Set.of(), Set.of());
    long start = System.nanoTime();
    assertNull(manager.waitForNextGoalStatusUpdate(goal, 20, GoalStatus.PENDING));
    long elapsed = TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - start);
    assertTrue("Wait returned prematurely: " + elapsed, elapsed >= 10);
    assertTrue("Milliseconds interpreted as seconds: " + elapsed, elapsed < 1000);
  }

  @Test
  public void unknownGoalStatusWaitReturnsNull() {
    TestManager manager = new TestManager();
    assertNull(manager.waitForNextGoalStatusUpdate(Long.MAX_VALUE, 10));
    assertNull(manager.waitForNextGoalStatusUpdate(Long.MAX_VALUE, 10, GoalStatus.ACTIVE));
  }

  @Test(timeout = 2000)
  public void statusWaitReturnsUpdatesAndHonorsExplicitInterrupts() throws Exception {
    TestManager manager = new TestManager();
    Goal goal = goal(1, Set.of(), Set.of());
    manager.knownGoal = goal;
    goal.addListener(manager);
    ExecutorService executor = Executors.newSingleThreadExecutor();
    try {
      Future<GoalStatus> update = executor.submit(
              () -> manager.waitForNextGoalStatusUpdate(goal, 0, GoalStatus.PENDING));
      goal.setStatus(GoalStatus.ACTIVE);
      assertEquals(GoalStatus.ACTIVE, update.get(1, TimeUnit.SECONDS));
      manager.interruptWaitForNextGoalStatusUpdate(goal.getId());
      assertNull(manager.waitForNextGoalStatusUpdate(goal, 0, GoalStatus.ACTIVE));
      goal.setStatus(GoalStatus.SUCCEEDED);
      assertNull(manager.waitForNextGoalStatusUpdate(goal.getId(), 10));
      assertEquals(GoalStatus.SUCCEEDED,
              manager.waitForNextGoalStatusUpdate(goal.getId(), 10, GoalStatus.ACTIVE));
    } finally {
      executor.shutdownNow();
    }
  }

  @Test(timeout = 2000)
  public void immediateInterpreterFailureReleasesResourcesAndRemovesActiveGoal() throws Exception {
    TestManager manager = new TestManager();
    Resource resource = new Resource(null, Factory.createSymbol("exclusive"));
    Goal goal = goal(1, Set.of(resource), Set.of(resource));
    manager.knownGoal = goal;
    AgentTeam team = new AgentTeam(goal.getActor(), manager);
    manager.agentTeams.put(goal.getActor(), team);
    manager.rootAgent = goal.getActor();
    resource.setHolder(goal);
    Field executorField = ExecutionManager.class.getDeclaredField("executor");
    executorField.setAccessible(true);
    executorField.set(manager, new InlineExecutor());
    Context context = new Context(new RootContext(new ActionConstraints(), null), null, "failing") {
      @Override
      public void doStep() {
        assertSame(goal, manager.getActiveGoal(goal.getId()));
        throw new IllegalStateException("Immediate interpreter failure");
      }

      @Override
      public Context copy(Context parent) {
        throw new UnsupportedOperationException();
      }
    };
    manager.submitGoalContext(goal, context);
    assertEquals(ActionStatus.FAIL, context.getStatus());
    assertEquals(GoalStatus.FAILED, goal.getStatus());
    assertNull(manager.getActiveGoal(goal.getId()));
    assertSame(goal, manager.getPastGoal(goal.getId()));
    assertTrue(resource.isAvailable());
    assertNull(resource.getHolder());
    assertTrue(team.getActiveGoals().isEmpty());
  }

  @Test(timeout = 2000)
  public void rejectedInterpreterSubmissionDoesNotLeaveAnActiveGoal() throws Exception {
    TestManager manager = new TestManager();
    Resource resource = new Resource(null, Factory.createSymbol("exclusive"));
    Goal goal = goal(1, Set.of(resource), Set.of(resource));
    manager.knownGoal = goal;
    manager.agentTeams.put(goal.getActor(), new AgentTeam(goal.getActor(), manager));
    resource.setHolder(goal);
    ExecutorService executor = Executors.newSingleThreadExecutor();
    executor.shutdown();
    Field executorField = ExecutionManager.class.getDeclaredField("executor");
    executorField.setAccessible(true);
    executorField.set(manager, executor);
    Context context = new Context(new RootContext(new ActionConstraints(), null), null, "rejected") {
      @Override
      public Context copy(Context parent) {
        throw new UnsupportedOperationException();
      }
    };
    try {
      manager.submitGoalContext(goal, context);
      fail("Expected rejected interpreter submission");
    } catch (RejectedExecutionException expected) {
      assertTrue(executor.isShutdown());
    }
    assertEquals(GoalStatus.FAILED, goal.getStatus());
    assertNull(manager.getActiveGoal(goal.getId()));
    assertSame(goal, manager.getPastGoal(goal.getId()));
    assertTrue(resource.isAvailable());
  }

  @Test(timeout = 3000)
  public void completionDoesNotInvertPendingAndActiveLocks() throws Exception {
    TestManager manager = new TestManager();
    Goal goal = goal(1, Set.of(), Set.of());
    manager.knownGoal = goal;
    manager.pendingGoals.add(new PendingGoal(goal));
    goal.setStatus(GoalStatus.SUCCEEDED);
    GoalInfo info = goal.getInfo();
    CountDownLatch pendingHeld = new CountDownLatch(1);
    manager.transferring = new CountDownLatch(1);
    ExecutorService executor = Executors.newFixedThreadPool(2, runnable -> {
      Thread thread = new Thread(runnable);
      thread.setDaemon(true);
      return thread;
    });
    try {
      Future<?> preemption = executor.submit(() -> {
        synchronized (manager.pendingGoalsLock) {
          pendingHeld.countDown();
          try {
            assertTrue(manager.transferring.await(1, TimeUnit.SECONDS));
          } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new AssertionError(e);
          }
          synchronized (manager.goalsLock) {
            assertTrue(Thread.holdsLock(manager.pendingGoalsLock));
          }
        }
      });
      assertTrue(pendingHeld.await(1, TimeUnit.SECONDS));
      Future<?> completion = executor.submit(() -> manager.goalStatusChanged(info));
      preemption.get(1, TimeUnit.SECONDS);
      completion.get(1, TimeUnit.SECONDS);
      assertTrue(manager.pendingGoals.isEmpty());
      assertEquals(goal, manager.getPastGoal(goal.getId()));
    } finally {
      executor.shutdownNow();
    }
  }
}
