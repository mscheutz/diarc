/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.learning;

import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.Condition;
import edu.tufts.hrilab.action.ConditionType;
import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.Observable;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.manager.ExecutionManager;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import org.junit.Test;

import java.lang.reflect.Method;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.CyclicBarrier;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

import static org.junit.Assert.*;

public class ActionLearningConcurrencyHardeningTest {
  private static Predicate signature(String name) {
    return Factory.createPredicate(name, "self:agent", "?object");
  }

  private static Goal taughtGoal(String command) {
    return Goal.createGoal(Factory.createPredicate(command, "self:agent", "?object"));
  }

  private static void await(CountDownLatch latch, String message) throws InterruptedException {
    assertTrue(message, latch.await(3, TimeUnit.SECONDS));
  }

  @Test(timeout = 20000)
  public void finalizationBoundaryRejectsLateInstructions() throws Exception {
    ExecutionManager manager = new ExecutionManager();
    ActionLearning learning = new ActionLearning(manager, false);
    Predicate action = signature("finalizationBoundary_" + System.nanoTime());

    ExecutorService executor = Executors.newFixedThreadPool(2);
    Future<Boolean> learnFuture = executor.submit(() -> learning.learnAction(action));
    try {
      learning.waitForActionLearningStart(action);
      learning.addGoal(taughtGoal("beforeClose"));

      CountDownLatch hookEntered = new CountDownLatch(1);
      CountDownLatch releaseHook = new CountDownLatch(1);
      learning.setBeforeFinalizationValidationHookForTest(() -> {
        hookEntered.countDown();
        try {
          await(releaseHook, "Timed out waiting to release finalization hook.");
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          throw new RuntimeException(e);
        }
      });

      Future<Boolean> endFuture = executor.submit(() -> learning.endActionLearning(action));
      await(hookEntered, "Timed out waiting for finalization boundary.");
      learning.addGoal(taughtGoal("afterClose"));
      releaseHook.countDown();

      assertTrue(endFuture.get(3, TimeUnit.SECONDS));
      assertTrue(learnFuture.get(3, TimeUnit.SECONDS));
    } finally {
      learning.setBeforeFinalizationValidationHookForTest(null);
      executor.shutdownNow();
    }

    ActionDBEntry learned = Database.getActionDB().getAction(action.getName());
    assertNotNull(learned);
    List<String> commands = learned.getEventSpecs().stream().map(EventSpec::getCommand).toList();
    assertEquals(List.of("beforeClose"), commands);
  }

  @Test(timeout = 20000)
  public void concurrentFinalizationRequestsAreSerialized() throws Exception {
    ExecutionManager manager = new ExecutionManager();
    ActionLearning learning = new ActionLearning(manager, false);
    Predicate action = signature("concurrentFinalize_" + System.nanoTime());

    ExecutorService executor = Executors.newFixedThreadPool(2);
    Future<Boolean> learnFuture = executor.submit(() -> learning.learnAction(action));
    try {
      learning.waitForActionLearningStart(action);
      learning.addGoal(taughtGoal("step"));

      CountDownLatch hookEntered = new CountDownLatch(1);
      CountDownLatch releaseHook = new CountDownLatch(1);
      learning.setBeforeFinalizationValidationHookForTest(() -> {
        hookEntered.countDown();
        try {
          await(releaseHook, "Timed out waiting to release finalization hook.");
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          throw new RuntimeException(e);
        }
      });

      Future<Boolean> firstFinalize = executor.submit(() -> learning.endActionLearning(action));
      await(hookEntered, "Timed out waiting for first finalization to enter.");
      boolean secondFinalize = learning.endActionLearning(action);
      releaseHook.countDown();

      assertTrue(firstFinalize.get(3, TimeUnit.SECONDS));
      assertFalse("Concurrent second finalization should be rejected while session is closing.", secondFinalize);
      assertTrue(learnFuture.get(3, TimeUnit.SECONDS));
    } finally {
      learning.setBeforeFinalizationValidationHookForTest(null);
      executor.shutdownNow();
    }
  }

  @Test(timeout = 20000)
  public void concurrentQueueStartupCreatesSingleWorker() throws Exception {
    ExecutionManager manager = new ExecutionManager();
    ActionLearning learning = new ActionLearning(manager, false);
    Predicate action = signature("singleWorkerStartup_" + System.nanoTime());
    long baselineWorkers = Thread.getAllStackTraces().keySet().stream()
        .filter(t -> t.isAlive() && "action-learning-goal-queue".equals(t.getName()))
        .count();

    ExecutorService learnExecutor = Executors.newSingleThreadExecutor();
    Future<Boolean> learnFuture = learnExecutor.submit(() -> learning.learnAction(action));
    ExecutorService raceExecutor = Executors.newFixedThreadPool(8);
    try {
      learning.waitForActionLearningStart(action);
      CyclicBarrier barrier = new CyclicBarrier(9);
      List<Future<?>> calls = new java.util.ArrayList<>();
      for (int i = 0; i < 8; i++) {
        calls.add(raceExecutor.submit(() -> {
          barrier.await(3, TimeUnit.SECONDS);
          learning.startQueuingGoals();
          return null;
        }));
      }
      barrier.await(3, TimeUnit.SECONDS);
      for (Future<?> call : calls) {
        call.get(3, TimeUnit.SECONDS);
      }
      assertTrue("Queue worker should reach running state.",
          learning.waitForQueueWorkerRunningForTest(3000));

      Thread worker = learning.getQueueWorkerForTest();
      assertNotNull(worker);
      assertTrue(worker.isAlive());
      long aliveWorkers = Thread.getAllStackTraces().keySet().stream()
          .filter(t -> t.isAlive() && "action-learning-goal-queue".equals(t.getName()))
          .count();
      assertEquals("Exactly one worker should be started for this session.",
          baselineWorkers + 1, aliveWorkers);

      assertTrue(learning.endActionLearning(action));
      assertTrue(learnFuture.get(3, TimeUnit.SECONDS));
    } finally {
      raceExecutor.shutdownNow();
      learnExecutor.shutdownNow();
    }
  }

  @Test(timeout = 20000)
  public void workerStopsWhenIdleAndRestartsAfterResume() throws Exception {
    ExecutionManager manager = new ExecutionManager();
    ActionLearning learning = new ActionLearning(manager, false);
    Predicate action = signature("workerLifecycle_" + System.nanoTime());

    ExecutorService executor = Executors.newSingleThreadExecutor();
    Future<Boolean> learnFuture = executor.submit(() -> learning.learnAction(action));
    try {
      learning.waitForActionLearningStart(action);
      assertTrue("Queue worker should reach running state.",
          learning.waitForQueueWorkerRunningForTest(3000));
      Thread firstWorker = learning.getQueueWorkerForTest();
      assertNotNull(firstWorker);
      assertTrue(firstWorker.isAlive());

      assertTrue(learning.pauseActionLearning(action));
      assertTrue("Queue worker should stop after idle pause.",
          learning.waitForQueueWorkerStoppedForTest(3000));

      assertTrue(learning.resumeActionLearning(action));
      assertTrue("Queue worker should restart after resume.",
          learning.waitForQueueWorkerRunningForTest(3000));
      Thread restartedWorker = learning.getQueueWorkerForTest();
      assertNotNull(restartedWorker);
      assertTrue(restartedWorker.isAlive());
      assertNotSame("Restart should create a fresh worker thread.", firstWorker, restartedWorker);

      learning.addGoal(taughtGoal("afterResume"));
      assertTrue(learning.endActionLearning(action));
      assertTrue(learnFuture.get(3, TimeUnit.SECONDS));
    } finally {
      executor.shutdownNow();
    }
  }

  @Test(timeout = 20000)
  public void nestedSessionKeepsInstructionOwnershipAcrossTransition() throws Exception {
    ExecutionManager manager = new ExecutionManager();
    ActionLearning learning = new ActionLearning(manager, false);
    String parentName = "nestedOwnerParent_" + System.nanoTime();
    String childName = parentName + "_child";
    Predicate parent = signature(parentName);
    Predicate child = signature(childName);

    ExecutorService executor = Executors.newFixedThreadPool(2);
    Future<Boolean> parentFuture = executor.submit(() -> learning.learnAction(parent));
    try {
      learning.waitForActionLearningStart(parent);
      CountDownLatch firstQueuedEntered = new CountDownLatch(1);
      CountDownLatch releaseFirstQueued = new CountDownLatch(1);
      AtomicBoolean first = new AtomicBoolean(true);
      learning.setBeforeQueuedGoalProcessingHookForTest(() -> {
        if (first.compareAndSet(true, false)) {
          firstQueuedEntered.countDown();
          try {
            await(releaseFirstQueued, "Timed out waiting to release first queued goal.");
          } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new RuntimeException(e);
          }
        }
      });

      learning.addGoal(taughtGoal("parentPending"));
      await(firstQueuedEntered, "Timed out waiting for parent queued instruction.");

      Future<Boolean> childFuture = executor.submit(() -> learning.learnAction(child));
      learning.waitForActionLearningStart(child);
      learning.addGoal(taughtGoal("childStep"));
      releaseFirstQueued.countDown();

      assertTrue(learning.endActionLearning(child));
      learning.addGoal(taughtGoal("parentAfterChild"));
      assertTrue(learning.endActionLearning(parent));
      assertTrue(childFuture.get(3, TimeUnit.SECONDS));
      assertTrue(parentFuture.get(3, TimeUnit.SECONDS));
    } finally {
      learning.setBeforeQueuedGoalProcessingHookForTest(null);
      executor.shutdownNow();
    }

    ActionDBEntry parentEntry = Database.getActionDB().getAction(parentName);
    ActionDBEntry childEntry = Database.getActionDB().getAction(childName);
    assertNotNull(parentEntry);
    assertNotNull(childEntry);
    assertEquals(List.of("parentPending", "parentAfterChild"),
        parentEntry.getEventSpecs().stream().map(EventSpec::getCommand).toList());
    assertEquals(List.of("childStep"),
        childEntry.getEventSpecs().stream().map(EventSpec::getCommand).toList());
  }

  @Test
  public void semanticEquivalenceComparesRoleDefaultsAndConditionMetadata() throws Exception {
    ActionLearning learning = new ActionLearning(null, false);
    Method method = ActionLearning.class.getDeclaredMethod("semanticallyEquivalent", ActionDBEntry.class, ActionDBEntry.class);
    method.setAccessible(true);

    ActionDBEntry sameA = buildSemanticProbeEntry("sameDefaults", "3", Observable.TRUE);
    ActionDBEntry sameB = buildSemanticProbeEntry("sameDefaults", "3", Observable.TRUE);
    assertTrue((Boolean) method.invoke(learning, sameA, sameB));

    ActionDBEntry differentDefault = buildSemanticProbeEntry("sameDefaults", "5", Observable.TRUE);
    assertFalse((Boolean) method.invoke(learning, sameA, differentDefault));

    ActionDBEntry differentConditionMetadata = buildSemanticProbeEntry("sameDefaults", "3", Observable.FALSE);
    assertFalse((Boolean) method.invoke(learning, sameA, differentConditionMetadata));
  }

  private static ActionDBEntry buildSemanticProbeEntry(String name, String defaultValue, Observable observable) {
    ActionDBEntry.Builder builder = new ActionDBEntry.Builder(name);
    builder.addRoles(List.of(
        new ActionBinding.Builder("?actor", Symbol.class).setSemanticType("agent").build(),
        defaultValue == null
            ? new ActionBinding.Builder("?value", Integer.class).setSemanticType("number").build()
            : new ActionBinding.Builder("?value", Integer.class).setSemanticType("number").setDefaultValue(defaultValue).build()
    ));
    builder.addCondition(new Condition(Factory.createPredicate("ready", Factory.createVariable("?value")), ConditionType.PRE, observable));
    builder.addEventSpec(new EventSpec.Builder(EventSpec.EventType.ACTION)
        .setActor("?actor")
        .setCommand("probe")
        .addInputArg("?value")
        .build());
    return builder.build(false);
  }
}
