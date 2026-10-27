package edu.tufts.hrilab.action.manager;

import edu.tufts.hrilab.action.ActionConstraints;
import edu.tufts.hrilab.action.ActionInterpreter;
import edu.tufts.hrilab.action.execution.Context;
import edu.tufts.hrilab.action.execution.RootContext;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalInfo;
import edu.tufts.hrilab.action.goal.GoalSchedulingExplanation;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.goal.PriorityTier;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import org.junit.Test;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.RejectedExecutionException;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Consumer;

import static org.junit.Assert.*;

public class StageThreeExecutionManagerTest {
  private static final Symbol SELF = Factory.createSymbol("self");

  private static class ControlledGoal extends Goal {
    int suspensionRequests;
    int cancellationRequests;
    int resumes;
    Runnable onSuspend = () -> {};
    Runnable onCancel = () -> {};
    boolean cancelImmediately;

    ControlledGoal(Resource resource, long priority) {
      super(Factory.createPredicate("controlled(self)"));
      setRequiredResources(Set.of());
      setHeldResources(resource == null ? Set.of() : Set.of(resource));
      setPriority(priority);
    }

    @Override
    public boolean suspend() {
      suspensionRequests++;
      onSuspend.run();
      return true;
    }

    @Override
    public boolean cancel() {
      cancellationRequests++;
      onCancel.run();
      if (cancelImmediately) {
        setStatus(GoalStatus.CANCELED);
      }
      return true;
    }

    @Override
    public boolean resume() {
      resumes++;
      setStatus(GoalStatus.ACTIVE);
      return true;
    }
  }

  private static class TestManager extends QueueExecutionManager {
    Consumer<Goal> startup = goal -> {};
    AtomicInteger executions = new AtomicInteger();

    TestManager() {
      rootAgent = SELF;
      agentHierarchy = Map.of(SELF, Set.of(SELF));
      agentTeams.put(SELF, new AgentTeam(SELF, this));
    }

    @Override
    protected boolean executeGoal(Goal goal) {
      assertFalse(Thread.holdsLock(pendingGoalsLock));
      assertFalse(Thread.holdsLock(resourceLock));
      assertFalse(Thread.holdsLock(goalsLock));
      assertFalse(Thread.currentThread().getName().equals("goal-manager-coordinator"));
      startup.accept(goal);
      Context context = new Context(new RootContext(new ActionConstraints(), null), null, "controlled") {
        @Override
        public Context copy(Context parent) {
          throw new UnsupportedOperationException();
        }
      };
      ActionInterpreter.createInterpreterFromContext(goal, context);
      executions.incrementAndGet();
      goal.setAsStarted();
      return true;
    }
  }

  private Resource resource() {
    return new Resource(null, Factory.createSymbol("shared"));
  }

  @Test(timeout = 3000)
  public void preemptionKeepsClaimsUntilAcknowledgmentAndThenResumesThePreviousInterpreter() {
    TestManager manager = new TestManager();
    Resource resource = resource();
    ControlledGoal previous = new ControlledGoal(resource, 1);
    ControlledGoal replacement = new ControlledGoal(resource, 100);
    manager.submitGoal(previous);
    manager.submitGoal(replacement);
    assertEquals(1, previous.suspensionRequests);
    assertSame(previous, resource.getHolder());
    assertSame(previous, manager.getActiveGoal(previous.getId()));
    assertNotNull(manager.getPendingGoal(replacement.getId()));
    ControlledGoal unrelated = new ControlledGoal(null, 1);
    manager.submitGoal(unrelated);
    assertEquals(GoalStatus.ACTIVE, unrelated.getStatus());
    previous.setStatus(GoalStatus.SUSPENDED);
    assertSame(replacement, resource.getHolder());
    assertNotNull(manager.getPendingGoal(previous.getId()));
    replacement.setStatus(GoalStatus.SUCCEEDED);
    assertSame(previous, resource.getHolder());
    assertEquals(GoalStatus.ACTIVE, previous.getStatus());
    assertEquals(1, previous.resumes);
    assertEquals(3, manager.executions.get());
  }

  @Test(timeout = 3000)
  public void kioskCancellationKeepsClaimsUntilTerminalAcknowledgment() {
    TestManager manager = new TestManager();
    Resource resource = resource();
    ControlledGoal kiosk = new ControlledGoal(resource, 1);
    kiosk.setPriorityTier(PriorityTier.KIOSK);
    ControlledGoal replacement = new ControlledGoal(resource, 100);
    manager.submitGoal(kiosk);
    manager.submitGoal(replacement);
    assertEquals(1, kiosk.cancellationRequests);
    assertSame(kiosk, resource.getHolder());
    kiosk.setStatus(GoalStatus.CANCELED);
    assertSame(replacement, resource.getHolder());
    assertSame(kiosk, manager.getPastGoal(kiosk.getId()));
    assertNull(manager.getPendingGoal(kiosk.getId()));
  }

  @Test(timeout = 3000)
  public void interruptionHooksCanWaitForManagerCallsOnAnotherThread() {
    TestManager manager = new TestManager();
    Resource resource = resource();
    ControlledGoal previous = new ControlledGoal(resource, 1);
    ControlledGoal unrelated = new ControlledGoal(null, 1);
    ExecutorService worker = Executors.newSingleThreadExecutor();
    try {
      previous.onSuspend = () -> {
        try {
          assertSame(unrelated, worker.submit(() -> manager.submitGoal(unrelated)).get(1, TimeUnit.SECONDS));
          assertEquals(GoalStatus.ACTIVE, unrelated.getStatus());
        } catch (Exception e) {
          throw new AssertionError(e);
        }
      };
      manager.submitGoal(previous);
      manager.submitGoal(new ControlledGoal(resource, 100));
      assertEquals(1, previous.suspensionRequests);
    } finally {
      worker.shutdownNow();
    }
  }

  @Test(timeout = 3000)
  public void slowStartupDoesNotBlockStateQueriesSubmissionsOrCancellation() throws Exception {
    TestManager manager = new TestManager();
    Resource resource = resource();
    ControlledGoal slow = new ControlledGoal(resource, 1);
    slow.cancelImmediately = true;
    ControlledGoal replacement = new ControlledGoal(resource, 100);
    CountDownLatch entered = new CountDownLatch(1);
    CountDownLatch release = new CountDownLatch(1);
    manager.startup = goal -> {
      if (goal == slow) {
        entered.countDown();
        try {
          assertTrue(release.await(2, TimeUnit.SECONDS));
        } catch (InterruptedException e) {
          throw new AssertionError(e);
        }
      }
    };
    ExecutorService worker = Executors.newSingleThreadExecutor();
    try {
      Future<Goal> submission = worker.submit(() -> manager.submitGoal(slow));
      assertTrue(entered.await(1, TimeUnit.SECONDS));
      assertEquals(List.of(GoalSchedulingExplanation.Reason.STARTING),
              manager.explainGoalScheduling(slow.getId()).reasons);
      manager.submitGoal(replacement);
      assertTrue(manager.cancelGoal(slow.getId()));
      assertSame(slow, resource.getHolder());
      manager.submitGoal(new ControlledGoal(null, 1));
      release.countDown();
      submission.get(1, TimeUnit.SECONDS);
      assertEquals(GoalStatus.CANCELED, slow.getStatus());
      assertSame(replacement, resource.getHolder());
    } finally {
      release.countDown();
      worker.shutdownNow();
    }
  }

  @Test(timeout = 3000)
  public void canceledReplacementDoesNotStrandAnAcknowledgedSuspension() {
    TestManager manager = new TestManager();
    Resource resource = resource();
    ControlledGoal previous = new ControlledGoal(resource, 1);
    ControlledGoal replacement = new ControlledGoal(resource, 100);
    replacement.cancelImmediately = true;
    manager.submitGoal(previous);
    manager.submitGoal(replacement);
    assertTrue(manager.cancelGoal(replacement.getId()));
    previous.setStatus(GoalStatus.SUSPENDED);
    assertSame(previous, resource.getHolder());
    assertEquals(GoalStatus.ACTIVE, previous.getStatus());
    assertEquals(1, previous.resumes);
    assertNull(manager.getPendingGoal(replacement.getId()));
  }

  @Test(timeout = 3000)
  public void manuallySuspendedPreemptedGoalDoesNotAutomaticallyResume() {
    TestManager manager = new TestManager();
    Resource resource = resource();
    ControlledGoal previous = new ControlledGoal(resource, 1);
    ControlledGoal replacement = new ControlledGoal(resource, 100);
    manager.submitGoal(previous);
    manager.submitGoal(replacement);
    assertTrue(manager.suspendGoal(previous.getId()));
    previous.setStatus(GoalStatus.SUSPENDED);
    replacement.setStatus(GoalStatus.SUCCEEDED);
    assertEquals(GoalStatus.SUSPENDED, previous.getStatus());
    assertEquals(0, previous.resumes);
    assertTrue(manager.resumeGoal(previous.getId()));
    assertEquals(GoalStatus.ACTIVE, previous.getStatus());
  }

  @Test(timeout = 3000)
  public void resumingAQueuedInterpreterWaitsForItsClaimsToBeAvailable() {
    TestManager manager = new TestManager();
    Resource resource = resource();
    ControlledGoal previous = new ControlledGoal(resource, 1);
    ControlledGoal replacement = new ControlledGoal(resource, 100);
    manager.submitGoal(previous);
    manager.submitGoal(replacement);
    previous.setStatus(GoalStatus.SUSPENDED);
    GoalInfo oldSuspension = previous.getInfo();
    assertTrue(manager.resumeGoal(previous.getId()));
    assertSame(replacement, resource.getHolder());
    assertEquals(0, previous.resumes);
    replacement.setStatus(GoalStatus.SUCCEEDED);
    assertSame(previous, resource.getHolder());
    assertEquals(1, previous.resumes);
    manager.goalStatusChanged(oldSuspension);
    assertNull(manager.getPendingGoal(previous.getId()));
    assertEquals(GoalStatus.ACTIVE, previous.getStatus());
  }

  @Test(timeout = 3000)
  public void terminalListenersRunAfterCommitAndCanWaitForOtherManagerCalls() {
    TestManager manager = new TestManager();
    Resource resource = resource();
    ControlledGoal goal = new ControlledGoal(resource, 1);
    ExecutorService worker = Executors.newSingleThreadExecutor();
    try {
      goal.addListener(info -> {
        if (info.status.isTerminated()) {
          assertSame(goal, manager.getPastGoal(goal.getId()));
          assertFalse(Thread.currentThread().getName().equals("goal-manager-coordinator"));
          try {
            ControlledGoal next = new ControlledGoal(resource, 1);
            assertSame(next, worker.submit(() -> manager.submitGoal(next)).get(1, TimeUnit.SECONDS));
            assertSame(next, resource.getHolder());
          } catch (Exception e) {
            throw new AssertionError(e);
          }
        }
      });
      manager.submitGoal(goal);
      goal.setStatus(GoalStatus.SUCCEEDED);
    } finally {
      worker.shutdownNow();
    }
  }

  @Test
  public void latePendingNotificationsCannotRegressAnExecutingGoal() {
    Goal goal = Goal.createGoal(Factory.createPredicate("test(self)"));
    assertTrue(goal.notifyIfStatus(GoalStatus.PENDING));
    goal.setAsStarted();
    long version = goal.getInfo().statusVersion;
    assertFalse(goal.notifyIfStatus(GoalStatus.PENDING));
    assertEquals(GoalStatus.ACTIVE, goal.getStatus());
    assertEquals(version, goal.getInfo().statusVersion);
  }

  @Test(timeout = 5000)
  public void startupCancellationIsCheckedBeforeRealInterpreterDispatch() throws Exception {
    CountDownLatch entered = new CountDownLatch(1);
    CountDownLatch release = new CountDownLatch(1);
    QueueExecutionManager manager = new QueueExecutionManager() {
      @Override
      protected boolean executeGoal(Goal goal) {
        entered.countDown();
        try {
          assertTrue(release.await(2, TimeUnit.SECONDS));
        } catch (InterruptedException e) {
          throw new AssertionError(e);
        }
        return super.executeGoal(goal);
      }
    };
    manager.rootAgent = SELF;
    manager.agentHierarchy = Map.of(SELF, Set.of(SELF));
    manager.agentTeams.put(SELF, new AgentTeam(SELF, manager));
    StateMachine state = StateMachine.createTestStateMachine(Set.of());
    Field stateField = ExecutionManager.class.getDeclaredField("sm");
    stateField.setAccessible(true);
    stateField.set(manager, state);
    Field root = ExecutionManager.class.getDeclaredField("rootContext");
    root.setAccessible(true);
    root.set(manager, new RootContext(new ActionConstraints(), state));
    Goal goal = Goal.createGoal(Factory.createPredicate("test(self)"));
    Resource resource = resource();
    goal.setRequiredResources(Set.of());
    goal.setHeldResources(Set.of(resource));
    ExecutorService worker = Executors.newSingleThreadExecutor();
    try {
      Future<Goal> submission = worker.submit(() -> manager.submitGoal(goal));
      assertTrue(entered.await(1, TimeUnit.SECONDS));
      assertTrue(manager.cancelGoal(goal.getId()));
      assertSame(goal, resource.getHolder());
      release.countDown();
      submission.get(2, TimeUnit.SECONDS);
      assertEquals(GoalStatus.CANCELED, goal.getStatus());
      assertEquals(-1, goal.getStartTime());
      assertTrue(resource.isAvailable());
      assertSame(goal, manager.getPastGoal(goal.getId()));
    } finally {
      release.countDown();
      worker.shutdownNow();
      state.shutdown();
    }
  }

  @Test(timeout = 5000)
  public void shutdownWaitsForSupersededInterpretersAndRejectsNewAdmission() throws Exception {
    TestManager manager = new TestManager();
    StateMachine state = StateMachine.createTestStateMachine(Set.of());
    ScheduledGoalManager scheduled = new ScheduledGoalManager(manager);
    Field stateField = ExecutionManager.class.getDeclaredField("sm");
    stateField.setAccessible(true);
    stateField.set(manager, state);
    Field scheduledField = ExecutionManager.class.getDeclaredField("scheduledGoalManager");
    scheduledField.setAccessible(true);
    scheduledField.set(manager, scheduled);
    Resource resource = resource();
    ControlledGoal previous = new ControlledGoal(resource, 1);
    ControlledGoal replacement = new ControlledGoal(resource, 100);
    CountDownLatch cancellationRequested = new CountDownLatch(2);
    previous.onCancel = cancellationRequested::countDown;
    replacement.onCancel = cancellationRequested::countDown;
    ExecutorService worker = Executors.newSingleThreadExecutor();
    boolean stopped = false;
    try {
      manager.submitGoal(previous);
      manager.submitGoal(replacement);
      previous.setStatus(GoalStatus.SUSPENDED);
      Future<?> shutdown = worker.submit(manager::shutdown);
      assertTrue(cancellationRequested.await(1, TimeUnit.SECONDS));
      assertFalse(shutdown.isDone());
      previous.setStatus(GoalStatus.CANCELED);
      replacement.setStatus(GoalStatus.CANCELED);
      shutdown.get(2, TimeUnit.SECONDS);
      stopped = true;
      assertTrue(resource.isAvailable());
      assertTrue(manager.getPendingGoals().isEmpty());
      try {
        manager.submitGoal(new ControlledGoal(null, 1));
        fail("Expected closed admission");
      } catch (RejectedExecutionException expected) {
        assertTrue(manager.getPendingGoals().isEmpty());
      }
    } finally {
      previous.setStatus(GoalStatus.CANCELED);
      replacement.setStatus(GoalStatus.CANCELED);
      if (!stopped) {
        scheduled.shutdown();
        state.shutdown();
      }
      worker.shutdownNow();
    }
  }

  @Test(timeout = 3000)
  public void failedInterruptionDoesNotPreventALaterPreemptionRequest() {
    TestManager manager = new TestManager();
    Resource resource = resource();
    ControlledGoal previous = new ControlledGoal(resource, 1);
    previous.onSuspend = () -> {
      throw new IllegalStateException("Interruption hook failed");
    };
    manager.submitGoal(previous);
    try {
      manager.submitGoal(new ControlledGoal(resource, 100));
      fail("Expected interruption failure");
    } catch (IllegalStateException expected) {
      assertEquals("Interruption hook failed", expected.getMessage());
    }
    assertSame(previous, resource.getHolder());
    previous.onSuspend = () -> {};
    ControlledGoal next = new ControlledGoal(resource, 200);
    manager.submitGoal(next);
    assertEquals(2, previous.suspensionRequests);
    previous.setStatus(GoalStatus.SUSPENDED);
    assertSame(next, resource.getHolder());
  }

  @Test(timeout = 5000)
  public void concurrentAdmissionsAndCompletionsNeverShareAnExclusiveClaim() throws Exception {
    TestManager manager = new TestManager();
    Resource resource = resource();
    ExecutorService callers = Executors.newFixedThreadPool(6);
    try {
      List<Future<?>> submissions = new ArrayList<>();
      for (int i = 0; i < 30; i++) {
        ControlledGoal goal = new ControlledGoal(resource, 1);
        submissions.add(callers.submit(() -> manager.submitGoal(goal)));
      }
      for (Future<?> submission : submissions) {
        submission.get(2, TimeUnit.SECONDS);
      }
      assertEquals(1, manager.getAgentTeam(SELF).getActiveGoals().size());
      assertEquals(29, manager.getPendingGoals().size());
      for (int i = 0; i < 30; i++) {
        Goal holder = resource.getHolder();
        assertNotNull(holder);
        assertEquals(Set.of(holder), resource.getUsers());
        assertSame(holder, manager.getActiveGoal(holder.getId()));
        holder.setStatus(GoalStatus.SUCCEEDED);
      }
      assertTrue(resource.isAvailable());
      assertTrue(resource.getUsers().isEmpty());
      assertTrue(manager.getPendingGoals().isEmpty());
      assertEquals(30, manager.getPastGoals().size());
      assertEquals(30, manager.executions.get());
    } finally {
      callers.shutdownNow();
    }
  }
}
