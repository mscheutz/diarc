package edu.tufts.hrilab.action.goal;

import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.fol.Factory;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.Assert.*;

public class GoalLifecycleTest {
  private Goal goal() {
    return Goal.createGoal(Factory.createPredicate("test(self)"));
  }

  @Test
  public void managerBookkeepingPrecedesPreviouslyRegisteredListeners() {
    Goal goal = goal();
    List<String> events = new ArrayList<>();
    goal.addListener(info -> events.add("external"));
    goal.setLifecycleListener(info -> events.add("manager"));
    goal.setStatus(GoalStatus.SUCCEEDED);
    assertEquals(List.of("manager", "external"), events);
  }

  @Test
  public void notificationsDoNotExposeLiveInfoOrOtherListenersSnapshots() {
    Goal goal = goal();
    List<GoalInfo> snapshots = new ArrayList<>();
    goal.addListener(info -> info.status = GoalStatus.FAILED);
    goal.addListener(snapshots::add);
    goal.setAsStarted();
    goal.setStatus(GoalStatus.SUCCEEDED);
    assertEquals(GoalStatus.ACTIVE, snapshots.get(0).status);
    assertEquals(-1, snapshots.get(0).end);
    assertEquals(GoalStatus.SUCCEEDED, snapshots.get(1).status);
    assertEquals(GoalStatus.SUCCEEDED, goal.getStatus());
  }

  @Test(timeout = 3000)
  public void competingTerminalTransitionsCommitOnceIncludingFailureMetadata() throws Exception {
    ExecutorService executor = Executors.newFixedThreadPool(2);
    try {
      for (int i = 0; i < 50; i++) {
        Goal goal = goal();
        AtomicInteger notifications = new AtomicInteger();
        goal.addListener(info -> notifications.incrementAndGet());
        CountDownLatch start = new CountDownLatch(1);
        ConditionJustification failure = new ConditionJustification(false);
        Future<?> canceled = executor.submit(() -> {
          await(start);
          goal.setStatus(GoalStatus.CANCELED);
        });
        Future<?> failed = executor.submit(() -> {
          await(start);
          goal.setFailureStatus(GoalStatus.FAILED, failure);
        });
        start.countDown();
        canceled.get(1, TimeUnit.SECONDS);
        failed.get(1, TimeUnit.SECONDS);
        assertEquals(1, notifications.get());
        assertTrue(goal.getEndTime() >= 0);
        if (goal.getStatus() == GoalStatus.FAILED) {
          assertSame(failure, goal.getFailConditions());
        } else {
          assertEquals(GoalStatus.CANCELED, goal.getStatus());
          assertNull(goal.getFailConditions());
        }
        long end = goal.getEndTime();
        goal.setFailureStatus(GoalStatus.FAILED, new ConditionJustification(false));
        assertEquals(end, goal.getEndTime());
        assertEquals(1, notifications.get());
      }
    } finally {
      executor.shutdownNow();
    }
  }

  @Test
  public void startTimeSurvivesSuspendResumeAndTerminalGoalsCannotRestart() {
    Goal goal = goal();
    assertTrue(goal.setAsStarted());
    long start = goal.getStartTime();
    goal.setStatus(GoalStatus.SUSPENDED);
    assertTrue(goal.setAsStarted());
    assertEquals(start, goal.getStartTime());
    goal.setStatus(GoalStatus.SUCCEEDED);
    assertFalse(goal.setAsStarted());
    assertEquals(GoalStatus.SUCCEEDED, goal.getStatus());
  }

  @Test
  public void listenersCanRegisterDuringNotification() {
    Goal goal = goal();
    AtomicInteger added = new AtomicInteger();
    goal.addListener(info -> goal.addListener(next -> added.incrementAndGet()));
    goal.setStatus(GoalStatus.ACTIVE);
    assertEquals(0, added.get());
    goal.setStatus(GoalStatus.SUCCEEDED);
    assertEquals(1, added.get());
  }

  @Test
  public void nonfailureStatusDoesNotInstallFailureMetadataOrNotifyTwice() {
    Goal goal = goal();
    AtomicInteger events = new AtomicInteger();
    goal.addListener(info -> events.incrementAndGet());
    goal.setFailureStatus(GoalStatus.ACTIVE, new ConditionJustification(false));
    assertEquals(GoalStatus.ACTIVE, goal.getStatus());
    assertNull(goal.getFailConditions());
    assertEquals(1, events.get());
  }

  @Test(expected = IllegalArgumentException.class)
  public void activeGoalCannotReturnToUnstartedStatus() {
    Goal goal = goal();
    goal.setAsStarted();
    goal.setStatus(GoalStatus.PENDING);
  }

  private static void await(CountDownLatch latch) {
    try {
      latch.await();
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      throw new AssertionError(e);
    }
  }
}
