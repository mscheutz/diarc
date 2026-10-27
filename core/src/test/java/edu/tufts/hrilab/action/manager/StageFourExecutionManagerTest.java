package edu.tufts.hrilab.action.manager;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.action.ActionConstraints;
import edu.tufts.hrilab.action.execution.RootContext;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalInfo;
import edu.tufts.hrilab.action.goal.GoalSchedulingExplanation;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.goal.PriorityTier;
import edu.tufts.hrilab.action.goal.schedule.ScheduledGoal;
import edu.tufts.hrilab.action.state.StateMachine;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import org.junit.Test;

import java.io.*;
import java.lang.reflect.Field;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeoutException;

import static org.junit.Assert.*;

public class StageFourExecutionManagerTest {
  private static final Symbol SELF = Factory.createSymbol("self");

  private static class TestManager extends QueueExecutionManager {
    volatile long millis = System.currentTimeMillis();
    volatile long nanos;
    boolean realTime;
    int executions;

    TestManager() {
      rootAgent = SELF;
      agentHierarchy = Map.of(SELF, Set.of(SELF));
      agentTeams.put(SELF, new AgentTeam(SELF, this));
    }

    @Override
    protected long schedulingTimeMillis() {
      return realTime ? System.currentTimeMillis() : millis;
    }

    @Override
    protected long schedulingTimeNanos() {
      return nanos;
    }

    void advance(long elapsedMillis) {
      millis += elapsedMillis;
      nanos += TimeUnit.MILLISECONDS.toNanos(elapsedMillis);
      refreshSchedulingPolicies();
    }

    @Override
    protected boolean executeGoal(Goal goal) {
      executions++;
      goal.setAsStarted();
      return true;
    }
  }

  private Goal goal(Resource resource, long priority) {
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
  public void expiredAdmissionNeverStartsAndPublishesFailureAfterCleanup() {
    TestManager manager = new TestManager();
    Goal goal = goal(null, 1);
    goal.setStartDeadline(manager.millis);
    goal.addListener(info -> {
      assertEquals(GoalStatus.FAILED, info.status);
      assertSame(goal, manager.getPastGoal(goal.getId()));
    });
    manager.submitGoal(goal);
    assertEquals(0, manager.executions);
    assertEquals(GoalStatus.FAILED, goal.getStatus());
    assertTrue(goal.getInfo().failConditions.toString().contains("startDeadlineMissed"));
    assertEquals(-1, goal.getInfo().start);
  }

  @Test
  public void exactDeadlineExpiresBlockedGoalWithoutReleasingAnotherGoalsClaims() {
    TestManager manager = new TestManager();
    Resource resource = resource();
    Goal blocker = goal(resource, 100);
    manager.submitGoal(blocker);
    Goal pending = goal(resource, 1);
    pending.setStartDeadline(manager.millis + 100);
    manager.submitGoal(pending);
    manager.advance(99);
    assertEquals(GoalStatus.PENDING, pending.getStatus());
    manager.advance(1);
    assertEquals(GoalStatus.FAILED, pending.getStatus());
    assertNull(manager.getPendingGoal(pending.getId()));
    assertSame(blocker, resource.getHolder());
    assertEquals(1, manager.executions);
  }

  @Test
  public void runningGoalIsUnaffectedByItsDeadline() {
    TestManager manager = new TestManager();
    Resource resource = resource();
    Goal running = goal(resource, 1);
    running.setStartDeadline(manager.millis + 100);
    manager.submitGoal(running);
    manager.advance(1000);
    assertEquals(GoalStatus.ACTIVE, running.getStatus());
    assertSame(running, resource.getHolder());
  }

  @Test
  public void suspendedPreviouslyStartedGoalDoesNotExpire() {
    TestManager manager = new TestManager();
    Goal goal = goal(null, 1);
    goal.setStartDeadline(manager.millis + 100);
    manager.submitGoal(goal);
    goal.setStatus(GoalStatus.SUSPENDED);
    manager.advance(1000);
    assertEquals(GoalStatus.SUSPENDED, goal.getStatus());
  }

  @Test(timeout = 3000)
  public void realTimerExpiresPendingGoalWithoutNewSubmissionsOrManualTicks() throws Exception {
    TestManager manager = new TestManager();
    manager.realTime = true;
    Resource resource = resource();
    manager.submitGoal(goal(resource, 100));
    Goal pending = goal(resource, 1);
    pending.setStartDeadline(System.currentTimeMillis() + 150);
    manager.submitGoal(pending);
    manager.waitForGoalUpdate(pending.getId(), pending.getInfo().statusVersion, 2000);
    assertEquals(GoalStatus.FAILED, pending.getStatus());
    assertSame(pending, manager.getPastGoal(pending.getId()));
  }

  @Test
  public void agingIsDisabledByDefault() {
    TestManager manager = new TestManager();
    Resource resource = resource();
    manager.submitGoal(goal(resource, 100));
    Goal old = goal(resource, 1);
    manager.submitGoal(old);
    manager.advance(1000);
    Goal recent = goal(resource, 2);
    manager.submitGoal(recent);
    assertEquals(List.of(recent, old), manager.getPendingGoals());
    assertEquals(1, manager.explainGoalScheduling(old.getId()).effectivePriority);
  }

  @Test
  public void agingReordersQueueAndRetainsCreditAtActivationWithoutMutatingBasePriority() {
    TestManager manager = new TestManager();
    Resource resource = resource();
    Goal blocker = goal(resource, 100);
    manager.submitGoal(blocker);
    manager.configurePriorityAging(10);
    try {
      Goal old = goal(resource, 1);
      manager.submitGoal(old);
      manager.advance(100);
      Goal recent = goal(resource, 5);
      manager.submitGoal(recent);
      assertEquals(List.of(old, recent), manager.getPendingGoals());
      GoalSchedulingExplanation explanation = manager.explainGoalScheduling(old.getId());
      assertEquals(11, explanation.effectivePriority);
      assertEquals(10, explanation.agingIntervalMillis);
      assertEquals(1, explanation.getGoalInfo().priority);
      assertSame(blocker, resource.getHolder());
      blocker.setStatus(GoalStatus.SUCCEEDED);
      assertSame(old, resource.getHolder());
      assertEquals(11, manager.explainGoalScheduling(old.getId()).effectivePriority);
      assertEquals(GoalStatus.PENDING, recent.getStatus());
    } finally {
      manager.configurePriorityAging(0);
    }
  }

  @Test
  public void agingNeverCrossesTiersAndSaturatesWithoutOverflow() {
    TestManager manager = new TestManager();
    Resource resource = resource();
    Goal blocker = goal(resource, Long.MAX_VALUE);
    blocker.setPriorityTier(PriorityTier.URGENT);
    manager.submitGoal(blocker);
    manager.configurePriorityAging(1);
    try {
      Goal old = goal(resource, Long.MAX_VALUE - 1);
      old.setPriorityTier(PriorityTier.LOW);
      manager.submitGoal(old);
      manager.advance(100);
      Goal normal = goal(resource, Long.MIN_VALUE);
      manager.submitGoal(normal);
      assertEquals(List.of(normal, old), manager.getPendingGoals());
      assertEquals(Long.MAX_VALUE, manager.explainGoalScheduling(old.getId()).effectivePriority);
      blocker.setStatus(GoalStatus.SUCCEEDED);
      assertSame(normal, resource.getHolder());
    } finally {
      manager.configurePriorityAging(0);
    }
  }

  @Test
  public void disablingAgingRebuildsTheQueueInLegacyOrder() {
    TestManager manager = new TestManager();
    Resource resource = resource();
    manager.submitGoal(goal(resource, 100));
    manager.configurePriorityAging(10);
    Goal old = goal(resource, 1);
    manager.submitGoal(old);
    manager.advance(100);
    Goal recent = goal(resource, 5);
    manager.submitGoal(recent);
    assertEquals(List.of(old, recent), manager.getPendingGoals());
    manager.configurePriorityAging(0);
    assertEquals(List.of(recent, old), manager.getPendingGoals());
    assertTrue(manager.cancelGoal(old.getId()));
    assertNull(manager.getPendingGoal(old.getId()));
  }

  @Test
  public void deadlineServiceUsesConfiguredSubmissionDefaultsAndCopiesMetadata() throws Exception {
    TestManager manager = new TestManager();
    manager.configureGoalPriorities(predicate -> PriorityTier.HIGH, predicate -> 23);
    GoalManagerComponent component = new GoalManagerComponent();
    Field field = GoalManagerComponent.class.getDeclaredField("em");
    field.setAccessible(true);
    field.set(component, manager);
    long id = component.submitGoalWithDeadline(Factory.createPredicate("test(self)"), manager.millis);
    GoalInfo info = manager.getGoal(id).getInfo();
    assertEquals(PriorityTier.HIGH, info.priorityTier);
    assertEquals(23, info.priority);
    ByteArrayOutputStream bytes = new ByteArrayOutputStream();
    try (ObjectOutputStream output = new ObjectOutputStream(bytes)) {
      output.writeObject(info);
    }
    try (ObjectInputStream input = new ObjectInputStream(new ByteArrayInputStream(bytes.toByteArray()))) {
      GoalInfo restored = (GoalInfo) input.readObject();
      assertEquals(info.startDeadline, restored.startDeadline);
    }
    component.configurePriorityAging(10);
    component.configurePriorityAging(0);
  }

  @Test(expected = IllegalArgumentException.class)
  public void negativeAgingIntervalIsRejected() {
    new TestManager().configurePriorityAging(-1);
  }

  @Test(expected = IllegalArgumentException.class)
  public void negativeDeadlineIsRejected() {
    goal(null, 1).setStartDeadline(-1);
  }

  @Test(expected = IllegalStateException.class)
  public void deadlineCannotBeChangedAfterAdmission() {
    TestManager manager = new TestManager();
    Goal goal = goal(null, 1);
    manager.submitGoal(goal);
    goal.setStartDeadline(manager.millis);
  }

  @Test(expected = IllegalArgumentException.class)
  public void definitionDeadlinesAreRejectedInsteadOfSilentlyApplyingToOccurrences() {
    new ScheduledGoal(Factory.createPredicate("scheduled(test(self),period(1,DAYS))")).setStartDeadline(1);
  }

  @Test(timeout = 5000)
  public void startupPastDeadlineNeverDispatchesARealInterpreter() throws Exception {
    CountDownLatch entered = new CountDownLatch(1);
    CountDownLatch release = new CountDownLatch(1);
    QueueExecutionManager manager = new QueueExecutionManager() {
      volatile long now = System.currentTimeMillis();

      @Override
      protected boolean executeGoal(Goal goal) {
        entered.countDown();
        try {
          assertTrue(release.await(2, TimeUnit.SECONDS));
        } catch (InterruptedException e) {
          throw new AssertionError(e);
        }
        now = goal.getInfo().startDeadline;
        return super.executeGoal(goal);
      }

      @Override
      protected long schedulingTimeMillis() {
        return now;
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
    Goal goal = goal(resource(), 1);
    goal.setStartDeadline(System.currentTimeMillis() + 60_000);
    ExecutorService worker = Executors.newSingleThreadExecutor();
    try {
      Future<Goal> submission = worker.submit(() -> manager.submitGoal(goal));
      assertTrue(entered.await(1, TimeUnit.SECONDS));
      assertEquals(GoalSchedulingExplanation.Reason.STARTING,
              manager.explainGoalScheduling(goal.getId()).reasons.get(0));
      release.countDown();
      submission.get(2, TimeUnit.SECONDS);
      assertEquals(GoalStatus.FAILED, goal.getStatus());
      assertEquals(-1, goal.getStartTime());
      assertTrue(goal.getInfo().failConditions.toString().contains("startDeadlineMissed"));
      assertSame(goal, manager.getPastGoal(goal.getId()));
    } finally {
      release.countDown();
      worker.shutdownNow();
      state.shutdown();
    }
  }

  @Test(timeout = 3000)
  public void backgroundAgingUpdatesOrderWithoutAnyNewSubmission() throws Exception {
    TestManager manager = new TestManager();
    Resource resource = resource();
    manager.submitGoal(goal(resource, 1000));
    manager.configurePriorityAging(100);
    try {
      Goal old = goal(resource, 1);
      manager.submitGoal(old);
      manager.nanos = TimeUnit.MILLISECONDS.toNanos(50);
      Goal recent = goal(resource, 2);
      manager.submitGoal(recent);
      assertEquals(List.of(recent, old), manager.getPendingGoals());
      manager.nanos = TimeUnit.MILLISECONDS.toNanos(100);
      long end = System.nanoTime() + TimeUnit.SECONDS.toNanos(2);
      while (manager.getPendingGoals().get(0) != old && System.nanoTime() < end) {
        Thread.sleep(10);
      }
      assertEquals(List.of(old, recent), manager.getPendingGoals());
      assertEquals(2, manager.explainGoalScheduling(old.getId()).effectivePriority);
    } finally {
      manager.configurePriorityAging(0);
    }
  }

  @Test(timeout = 3000)
  public void joinsWaitForCatchUpAdmissionAndRespectTimeouts() throws Exception {
    TestManager manager = new TestManager();
    Goal occurrence = goal(null, 1);
    CountDownLatch entered = new CountDownLatch(1);
    CountDownLatch release = new CountDownLatch(1);
    manager.runScheduledContinuation(occurrence, () -> {
      entered.countDown();
      try {
        assertTrue(release.await(2, TimeUnit.SECONDS));
      } catch (InterruptedException e) {
        throw new AssertionError(e);
      }
      manager.submitGoal(occurrence);
      occurrence.setStatus(GoalStatus.SUCCEEDED);
    });
    ExecutorService worker = Executors.newSingleThreadExecutor();
    try {
      assertTrue(entered.await(1, TimeUnit.SECONDS));
      assertEquals(GoalStatus.PENDING, manager.joinOnGoal(occurrence.getId(), 25));
      Future<GoalStatus> joined = worker.submit(() -> manager.joinOnGoal(occurrence.getId()));
      try {
        joined.get(25, TimeUnit.MILLISECONDS);
        fail("Join returned before catch-up admission");
      } catch (TimeoutException expected) {
        assertNull(manager.getGoal(occurrence.getId()));
      }
      release.countDown();
      assertEquals(GoalStatus.SUCCEEDED, joined.get(1, TimeUnit.SECONDS));
    } finally {
      release.countDown();
      worker.shutdownNow();
    }
  }

  @Test(timeout = 5000)
  public void shutdownDrainsQueuedCatchUpAndPreventsItsLateAdmission() throws Exception {
    TestManager manager = new TestManager();
    StateMachine state = StateMachine.createTestStateMachine(Set.of());
    ScheduledGoalManager scheduled = new ScheduledGoalManager(manager);
    Field stateField = ExecutionManager.class.getDeclaredField("sm");
    stateField.setAccessible(true);
    stateField.set(manager, state);
    Field scheduledField = ExecutionManager.class.getDeclaredField("scheduledGoalManager");
    scheduledField.setAccessible(true);
    scheduledField.set(manager, scheduled);
    ScheduledGoal definition = new ScheduledGoal(Factory.createPredicate("scheduled(test(self),period(1,DAYS))"));
    Goal occurrence = definition.getNextGoalToSubmit();
    CountDownLatch entered = new CountDownLatch(1);
    CountDownLatch release = new CountDownLatch(1);
    manager.runScheduledContinuation(occurrence, () -> {
      entered.countDown();
      try {
        assertTrue(release.await(2, TimeUnit.SECONDS));
      } catch (InterruptedException e) {
        throw new AssertionError(e);
      }
      manager.submitScheduledOccurrence(definition, occurrence);
    });
    ExecutorService worker = Executors.newSingleThreadExecutor();
    boolean stopped = false;
    try {
      assertTrue(entered.await(1, TimeUnit.SECONDS));
      Future<?> shutdown = worker.submit(manager::shutdown);
      try {
        shutdown.get(50, TimeUnit.MILLISECONDS);
        fail("Shutdown did not drain queued catch-up");
      } catch (TimeoutException expected) {
        assertNull(manager.getGoal(occurrence.getId()));
      }
      release.countDown();
      shutdown.get(2, TimeUnit.SECONDS);
      stopped = true;
      assertEquals(GoalStatus.CANCELED, occurrence.getStatus());
      assertEquals(0, manager.executions);
    } finally {
      release.countDown();
      worker.shutdownNow();
      if (!stopped) {
        scheduled.shutdown();
        state.shutdown();
      }
    }
  }

  @Test
  public void optInActionsAreCallableThroughTrade() throws Exception {
    TestManager manager = new TestManager();
    GoalManagerComponent component = new GoalManagerComponent();
    Field field = GoalManagerComponent.class.getDeclaredField("em");
    field.setAccessible(true);
    field.set(component, manager);
    TRADE.registerAllServices(component, List.of());
    try {
      long id = TRADE.getAvailableService(new TRADEServiceConstraints()
                      .name("submitGoalWithDeadline").argTypes(Predicate.class, Long.class))
              .call(Long.class, Factory.createPredicate("test(self)"), manager.millis);
      assertEquals(GoalStatus.FAILED, manager.getGoal(id).getStatus());
      TRADE.getAvailableService(new TRADEServiceConstraints()
                      .name("configurePriorityAging").argTypes(Long.class))
              .call(void.class, 10L);
    } finally {
      manager.configurePriorityAging(0);
      TRADE.deregister(component);
    }
  }
}
