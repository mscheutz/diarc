package edu.tufts.hrilab.action.manager;

import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalInfo;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.goal.GoalWaitResult;
import edu.tufts.hrilab.fol.Factory;
import org.junit.Test;

import java.lang.reflect.Field;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.Lock;

import static org.junit.Assert.*;

public class GoalVersionedWaitTest {
  private static class TestManager extends ExecutionManager {
    final Map<Long, Goal> goals = new ConcurrentHashMap<>();

    @Override
    public Goal getGoal(long gid) {
      return goals.get(gid);
    }

    Goal addGoal() {
      Goal goal = Goal.createGoal(Factory.createPredicate("test(self)"));
      goals.put(goal.getId(), goal);
      goal.addListener(this);
      return goal;
    }
  }

  @Test
  public void versionsDistinguishRoundTripsAndRepeatedStatusNotifications() {
    TestManager manager = new TestManager();
    Goal goal = manager.addGoal();
    assertEquals(0, goal.getInfo().statusVersion);
    goal.setAsStarted();
    long version = goal.getInfo().statusVersion;
    goal.setStatus(GoalStatus.SUSPENDED);
    goal.setStatus(GoalStatus.ACTIVE);
    GoalWaitResult result = manager.waitForGoalUpdate(goal.getId(), version, 10);
    assertEquals(GoalWaitResult.Outcome.UPDATED, result.outcome);
    assertEquals(GoalStatus.ACTIVE, result.getGoalInfo().status);
    assertEquals(version + 2, result.getGoalInfo().statusVersion);
    goal.setStatus(GoalStatus.ACTIVE);
    assertEquals(version + 3, goal.getInfo().statusVersion);
    result.getGoalInfo().statusVersion = 999;
    assertEquals(version + 2, result.getGoalInfo().statusVersion);
  }

  @Test(timeout = 2000)
  public void terminalWaitsReturnImmediatelyAndDuplicateCompletionDoesNotAdvanceVersion() {
    TestManager manager = new TestManager();
    Goal goal = manager.addGoal();
    goal.setStatus(GoalStatus.SUCCEEDED);
    long version = goal.getInfo().statusVersion;
    goal.setStatus(GoalStatus.CANCELED);
    assertEquals(version, goal.getInfo().statusVersion);
    GoalWaitResult result = manager.waitForGoalUpdate(goal.getId(), version, 0);
    assertEquals(GoalWaitResult.Outcome.TERMINATED, result.outcome);
    assertEquals(GoalStatus.SUCCEEDED, result.getGoalInfo().status);
    assertNull(manager.waitForNextGoalStatusUpdate(goal.getId(), 0));
    assertEquals(GoalStatus.SUCCEEDED,
            manager.waitForNextGoalStatusUpdate(goal.getId(), 0, GoalStatus.SUCCEEDED));
  }

  @Test(timeout = 3000)
  public void allVersionedAndLegacyWaitersReceiveOneTransition() throws Exception {
    TestManager manager = new TestManager();
    Goal goal = manager.addGoal();
    ExecutorService executor = Executors.newFixedThreadPool(3);
    try {
      Future<GoalWaitResult> first = executor.submit(() -> manager.waitForGoalUpdate(goal.getId(), 0, 0));
      Future<GoalWaitResult> second = executor.submit(() -> manager.waitForGoalUpdate(goal.getId(), 0, 0));
      Future<GoalStatus> legacy = executor.submit(() -> manager.waitForNextGoalStatusUpdate(goal.getId(), 0));
      awaitWaiters(manager, goal.getId(), 3);
      goal.setAsStarted();
      assertEquals(GoalWaitResult.Outcome.UPDATED, first.get(1, TimeUnit.SECONDS).outcome);
      assertEquals(GoalWaitResult.Outcome.UPDATED, second.get(1, TimeUnit.SECONDS).outcome);
      assertEquals(GoalStatus.ACTIVE, legacy.get(1, TimeUnit.SECONDS));
      assertEquals(0, waitStateCount(manager));
    } finally {
      executor.shutdownNow();
    }
  }

  @Test(timeout = 3000)
  public void explicitInterruptBroadcastsWithoutPoisoningLaterWaiters() throws Exception {
    TestManager manager = new TestManager();
    Goal goal = manager.addGoal();
    ExecutorService executor = Executors.newFixedThreadPool(3);
    try {
      Future<GoalWaitResult> first = executor.submit(() -> manager.waitForGoalUpdate(goal.getId(), 0, 0));
      Future<GoalWaitResult> second = executor.submit(() -> manager.waitForGoalUpdate(goal.getId(), 0, 0));
      Future<GoalStatus> legacy = executor.submit(() -> manager.waitForNextGoalStatusUpdate(goal.getId(), 0));
      awaitWaiters(manager, goal.getId(), 3);
      manager.interruptWaitForNextGoalStatusUpdate(goal.getId());
      assertEquals(GoalWaitResult.Outcome.INTERRUPTED, first.get(1, TimeUnit.SECONDS).outcome);
      assertEquals(GoalWaitResult.Outcome.INTERRUPTED, second.get(1, TimeUnit.SECONDS).outcome);
      assertNull(legacy.get(1, TimeUnit.SECONDS));
      assertEquals(GoalWaitResult.Outcome.TIMED_OUT, manager.waitForGoalUpdate(goal.getId(), 0, 10).outcome);
      assertEquals(GoalStatus.PENDING, goal.getStatus());
      assertEquals(0, waitStateCount(manager));
    } finally {
      executor.shutdownNow();
    }
  }

  @Test(timeout = 2000)
  public void interruptBeforeRegistrationIsConsumedOnceForCompatibility() throws Exception {
    TestManager manager = new TestManager();
    Goal goal = manager.addGoal();
    manager.interruptWaitForNextGoalStatusUpdate(goal.getId());
    assertNull(manager.waitForNextGoalStatusUpdate(goal.getId(), 0));
    assertEquals(GoalWaitResult.Outcome.TIMED_OUT, manager.waitForGoalUpdate(goal.getId(), 0, 10).outcome);
    assertEquals(0, waitStateCount(manager));
  }

  @Test(timeout = 2000)
  public void timeoutIsMillisecondsAndDoesNotCancelGoal() {
    TestManager manager = new TestManager();
    Goal goal = manager.addGoal();
    long start = System.nanoTime();
    GoalWaitResult result = manager.waitForGoalUpdate(goal.getId(), 0, 25);
    long elapsed = TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - start);
    assertEquals(GoalWaitResult.Outcome.TIMED_OUT, result.outcome);
    assertTrue("Premature timeout: " + elapsed, elapsed >= 15);
    assertTrue("Milliseconds treated as seconds: " + elapsed, elapsed < 1000);
    assertEquals(GoalStatus.PENDING, result.getGoalInfo().status);
    assertEquals(GoalStatus.PENDING, goal.getStatus());
  }

  @Test(timeout = 3000)
  public void unrelatedAndSpuriousSignalsDoNotSatisfyVersionPredicate() throws Exception {
    TestManager manager = new TestManager();
    Goal goal = manager.addGoal();
    Goal unrelated = manager.addGoal();
    ExecutorService executor = Executors.newSingleThreadExecutor();
    try {
      Future<GoalWaitResult> future = executor.submit(() -> manager.waitForGoalUpdate(goal.getId(), 0, 60));
      awaitWaiters(manager, goal.getId(), 1);
      for (int i = 0; i < 50; i++) {
        unrelated.setStatus(GoalStatus.ACTIVE);
        manager.goalStatusChanged(goal.getInfo());
      }
      assertEquals(GoalWaitResult.Outcome.TIMED_OUT, future.get(1, TimeUnit.SECONDS).outcome);
    } finally {
      executor.shutdownNow();
    }
  }

  @Test
  public void alreadyInterruptedThreadReturnsExplicitOutcomeAndKeepsFlag() {
    TestManager manager = new TestManager();
    Goal goal = manager.addGoal();
    Thread.currentThread().interrupt();
    try {
      assertEquals(GoalWaitResult.Outcome.INTERRUPTED, manager.waitForGoalUpdate(goal.getId(), 0, 0).outcome);
      assertTrue(Thread.currentThread().isInterrupted());
    } finally {
      Thread.interrupted();
    }
  }

  @Test(timeout = 3000)
  public void interruptingBlockedThreadRestoresInterruptFlag() throws Exception {
    TestManager manager = new TestManager();
    Goal goal = manager.addGoal();
    AtomicBoolean interrupted = new AtomicBoolean();
    AtomicBoolean correctOutcome = new AtomicBoolean();
    Thread waiter = new Thread(() -> {
      correctOutcome.set(manager.waitForGoalUpdate(goal.getId(), 0, 0).outcome == GoalWaitResult.Outcome.INTERRUPTED);
      interrupted.set(Thread.currentThread().isInterrupted());
    });
    waiter.setDaemon(true);
    waiter.start();
    try {
      awaitWaiters(manager, goal.getId(), 1);
      waiter.interrupt();
      waiter.join(1000);
      assertFalse(waiter.isAlive());
      assertTrue(correctOutcome.get());
      assertTrue(interrupted.get());
    } finally {
      waiter.interrupt();
    }
  }

  @Test
  public void unknownGoalHasExplicitOutcome() {
    GoalWaitResult result = new TestManager().waitForGoalUpdate(Long.MAX_VALUE, 0, 0);
    assertEquals(GoalWaitResult.Outcome.UNKNOWN_GOAL, result.outcome);
    assertNull(result.getGoalInfo());
  }

  @Test(timeout = 2000)
  public void legacyEnumWaitIgnoresRepeatedSameStatusWhileVersionedWaitDetectsIt() {
    TestManager manager = new TestManager();
    Goal goal = manager.addGoal();
    goal.setStatus(GoalStatus.PENDING);
    assertEquals(GoalWaitResult.Outcome.UPDATED, manager.waitForGoalUpdate(goal.getId(), 0, 10).outcome);
    assertNull(manager.waitForNextGoalStatusUpdate(goal.getId(), 10, GoalStatus.PENDING));
  }

  @Test(expected = IllegalArgumentException.class)
  public void negativeTimeoutIsRejected() {
    new TestManager().waitForGoalUpdate(1, 0, -1);
  }

  @Test(expected = IllegalArgumentException.class)
  public void negativeVersionIsRejected() {
    new TestManager().waitForGoalUpdate(1, -1, 0);
  }

  private static void awaitWaiters(ExecutionManager manager, long gid, int count) throws Exception {
    long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(1);
    while (System.nanoTime() < deadline) {
      Lock lock = waitLock(manager);
      lock.lock();
      try {
        Object state = waitStates(manager).get(gid);
        if (state != null) {
          Field field = state.getClass().getDeclaredField("waiters");
          field.setAccessible(true);
          if (field.getInt(state) == count) {
            return;
          }
        }
      } finally {
        lock.unlock();
      }
      Thread.yield();
    }
    fail("Waiters did not register");
  }

  private static Lock waitLock(ExecutionManager manager) throws Exception {
    Field field = ExecutionManager.class.getDeclaredField("goalStatusUpdateLock");
    field.setAccessible(true);
    return (Lock) field.get(manager);
  }

  private static Map<?, ?> waitStates(ExecutionManager manager) throws Exception {
    Field field = ExecutionManager.class.getDeclaredField("goalWaitStates");
    field.setAccessible(true);
    return (Map<?, ?>) field.get(manager);
  }

  private static int waitStateCount(ExecutionManager manager) throws Exception {
    Lock lock = waitLock(manager);
    lock.lock();
    try {
      return waitStates(manager).size();
    } finally {
      lock.unlock();
    }
  }
}
