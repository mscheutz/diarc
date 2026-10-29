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
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
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
  private static List<ActionLearningStatus> recordStatusNotifications(ActionLearning learning) throws Exception {
    Field field = ActionLearning.class.getDeclaredField("statusCondition");
    field.setAccessible(true);
    java.util.concurrent.locks.Condition original =
        (java.util.concurrent.locks.Condition) field.get(learning);
    List<ActionLearningStatus> notifications = java.util.Collections.synchronizedList(new ArrayList<>());
    field.set(learning, Proxy.newProxyInstance(
        java.util.concurrent.locks.Condition.class.getClassLoader(),
        new Class<?>[]{java.util.concurrent.locks.Condition.class},
        (proxy, method, args) -> {
          if (method.getName().equals("signalAll")) {
            notifications.add(learning.getLearningStatus());
          }
          try {
            return method.invoke(original, args);
          } catch (InvocationTargetException e) {
            throw e.getCause();
          }
        }));
    return notifications;
  }

  @Test(timeout = 15000)
  public void delayedActivationCannotRestoreActiveAfterCancellation() throws Exception {
    assertDelayedActivationCannotRestoreActive(false);
  }

  @Test(timeout = 15000)
  public void delayedActivationCannotRestoreActiveAfterFinalization() throws Exception {
    assertDelayedActivationCannotRestoreActive(true);
  }

  private void assertDelayedActivationCannotRestoreActive(boolean finalize) throws Exception {
    ActionLearning learning = new ActionLearning(new ExecutionManager(), false);
    Predicate action = signature("delayedActivation_" + System.nanoTime());
    CountDownLatch activated = new CountDownLatch(1);
    CountDownLatch releaseActivation = new CountDownLatch(1);
    learning.setAfterSessionActivationHookForTest(() -> {
      activated.countDown();
      try {
        await(releaseActivation, "Timed out waiting to release activation.");
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        throw new RuntimeException(e);
      }
    });
    ExecutorService executor = Executors.newSingleThreadExecutor();
    Future<Boolean> learn = executor.submit(() -> learning.learnAction(action));
    try {
      await(activated, "Activation did not reach the post-transition boundary.");
      assertTrue(finalize ? learning.endActionLearning(action) : learning.cancelActionLearning(action));
      releaseActivation.countDown();
      assertTrue("The completed session must not start waiting again.", learn.get(3, TimeUnit.SECONDS));
      assertEquals(finalize ? ActionLearningStatus.END : ActionLearningStatus.NONE, learning.getLearningStatus());
      assertTrue(learning.waitForQueueWorkerStoppedForTest(3000));
    } finally {
      releaseActivation.countDown();
      learning.setAfterSessionActivationHookForTest(null);
      executor.shutdownNow();
    }
  }

  @Test(timeout = 15000)
  public void nestedCancellationDoesNotNotifyTransientInactiveStatus() throws Exception {
    ActionLearning learning = new ActionLearning(new ExecutionManager(), false);
    List<ActionLearningStatus> notifications = recordStatusNotifications(learning);
    Predicate parent = signature("statusParent_" + System.nanoTime());
    Predicate child = signature(parent.getName() + "_child");
    ExecutorService executor = Executors.newFixedThreadPool(2);
    Future<Boolean> parentFuture = executor.submit(() -> learning.learnAction(parent));
    try {
      learning.waitForActionLearningStart(parent);
      Future<Boolean> childFuture = executor.submit(() -> learning.learnAction(child));
      learning.waitForActionLearningStart(child);
      notifications.clear();
      assertTrue(learning.cancelActionLearning(child));
      assertEquals(ActionLearningStatus.ACTIVE, learning.getLearningStatus());
      assertFalse("Nested cancellation must not notify a temporary inactive status.",
          notifications.stream().anyMatch(status -> status != ActionLearningStatus.ACTIVE));
      assertFalse("Parent learning waiter must remain blocked.", parentFuture.isDone());
      assertFalse("Child learning waiter follows the still-active parent lifecycle.", childFuture.isDone());
      assertTrue(learning.endActionLearning(parent));
      assertTrue(parentFuture.get(3, TimeUnit.SECONDS));
      assertTrue(childFuture.get(3, TimeUnit.SECONDS));
      assertTrue(learning.waitForQueueWorkerStoppedForTest(3000));
    } finally {
      executor.shutdownNow();
    }
  }

  @Test(timeout = 15000)
  public void normalLifecycleNotifiesOnlyCommittedStatuses() throws Exception {
    ActionLearning learning = new ActionLearning(new ExecutionManager(), false);
    List<ActionLearningStatus> notifications = recordStatusNotifications(learning);
    Predicate parent = signature("statusLifecycle_" + System.nanoTime());
    Predicate child = signature(parent.getName() + "_child");
    ExecutorService executor = Executors.newSingleThreadExecutor();
    try {
      Future<Boolean> parentFuture = executor.submit(() -> learning.learnAction(parent));
      learning.waitForActionLearningStart(parent);
      assertTrue(learning.pauseActionLearning(parent));
      assertTrue(parentFuture.get(3, TimeUnit.SECONDS));
      assertTrue(learning.waitForQueueWorkerStoppedForTest(3000));

      assertTrue(learning.resumeActionLearning(parent));
      Future<Boolean> childFuture = executor.submit(() -> learning.learnAction(child));
      learning.waitForActionLearningStart(child);
      assertTrue(learning.cancelActionLearning(child));
      assertFalse(childFuture.isDone());
      assertTrue(learning.endActionLearning(parent));
      assertTrue(childFuture.get(3, TimeUnit.SECONDS));
      assertTrue(learning.waitForQueueWorkerStoppedForTest(3000));
      assertEquals(List.of(ActionLearningStatus.ACTIVE, ActionLearningStatus.NONE,
          ActionLearningStatus.ACTIVE, ActionLearningStatus.ACTIVE,
          ActionLearningStatus.ACTIVE, ActionLearningStatus.END), notifications);
    } finally {
      executor.shutdownNow();
    }
  }

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

  @Test(timeout = 20000)
  public void learnActionCannotReactivateClosingSession() throws Exception {
    ExecutionManager manager = new ExecutionManager();
    ActionLearning learning = new ActionLearning(manager, false);
    Predicate action = signature("closingReactivationLearn_" + System.nanoTime());
    ExecutorService executor = Executors.newFixedThreadPool(3);
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
      await(hookEntered, "Timed out waiting for finalization to enter validation.");
      Future<Boolean> reactivationFuture = executor.submit(() -> learning.learnAction(action));
      assertFalse("learnAction must not reactivate a CLOSING session.", reactivationFuture.get(3, TimeUnit.SECONDS));
      releaseHook.countDown();

      assertTrue(endFuture.get(3, TimeUnit.SECONDS));
      assertTrue(learnFuture.get(3, TimeUnit.SECONDS));
    } finally {
      learning.setBeforeFinalizationValidationHookForTest(null);
      executor.shutdownNow();
    }
  }

  @Test(timeout = 20000)
  public void closingParentCannotStartNestedLearningSession() throws Exception {
    ExecutionManager manager = new ExecutionManager();
    ActionLearning learning = new ActionLearning(manager, false);
    String parentName = "closingParent_" + System.nanoTime();
    Predicate parent = signature(parentName);
    Predicate child = signature(parentName + "_child");
    ExecutorService executor = Executors.newFixedThreadPool(2);
    Future<Boolean> parentFuture = executor.submit(() -> learning.learnAction(parent));
    try {
      learning.waitForActionLearningStart(parent);
      CountDownLatch validationEntered = new CountDownLatch(1);
      CountDownLatch releaseValidation = new CountDownLatch(1);
      learning.setBeforeFinalizationValidationHookForTest(() -> {
        validationEntered.countDown();
        try {
          await(releaseValidation, "Timed out waiting to release parent finalization.");
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          throw new RuntimeException(e);
        }
      });

      Future<Boolean> finalizeParent = executor.submit(() -> learning.endActionLearning(parent));
      await(validationEntered, "Timed out waiting for parent to enter finalization.");
      assertFalse("A CLOSING parent must not acquire a nested child.", learning.learnAction(child));
      releaseValidation.countDown();

      assertTrue(finalizeParent.get(3, TimeUnit.SECONDS));
      assertTrue(parentFuture.get(3, TimeUnit.SECONDS));
    } finally {
      learning.setBeforeFinalizationValidationHookForTest(null);
      executor.shutdownNow();
    }

    assertNotNull(Database.getActionDB().getAction(parentName));
    assertNull(Database.getActionDB().getAction(child.getName()));
  }

  @Test(timeout = 20000)
  public void resumeCannotReactivateClosingPausedSession() throws Exception {
    ExecutionManager manager = new ExecutionManager();
    ActionLearning learning = new ActionLearning(manager, false);
    Predicate action = signature("closingReactivationResume_" + System.nanoTime());
    ExecutorService executor = Executors.newFixedThreadPool(2);
    Future<Boolean> learnFuture = executor.submit(() -> learning.learnAction(action));
    try {
      learning.waitForActionLearningStart(action);
      learning.addGoal(taughtGoal("beforePause"));
      assertTrue(learning.pauseActionLearning(action));
      assertTrue(learnFuture.get(3, TimeUnit.SECONDS));

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
      await(hookEntered, "Timed out waiting for paused-session finalization boundary.");
      assertFalse("resumeActionLearning must not reactivate a CLOSING session.",
          learning.resumeActionLearning(action));
      releaseHook.countDown();
      assertTrue(endFuture.get(3, TimeUnit.SECONDS));
    } finally {
      learning.setBeforeFinalizationValidationHookForTest(null);
      executor.shutdownNow();
    }
  }

  @Test(timeout = 20000)
  public void structuralMutationRejectedOnceFinalizationBoundaryIsCrossed() throws Exception {
    ExecutionManager manager = new ExecutionManager();
    ActionLearning learning = new ActionLearning(manager, false);
    Predicate action = signature("structuralClosingBoundary_" + System.nanoTime());
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
      assertFalse("Structural teaching must be rejected after finalization boundary.",
          learning.beginLearnedIf(action, Factory.createPredicate("true")));
      releaseHook.countDown();
      assertTrue(endFuture.get(3, TimeUnit.SECONDS));
      assertTrue(learnFuture.get(3, TimeUnit.SECONDS));
    } finally {
      learning.setBeforeFinalizationValidationHookForTest(null);
      executor.shutdownNow();
    }

    ActionDBEntry learned = Database.getActionDB().getAction(action.getName());
    assertNotNull(learned);
    assertEquals(List.of("beforeClose"),
        learned.getEventSpecs().stream().map(EventSpec::getCommand).toList());
  }

  @Test(timeout = 20000)
  public void cancelingNestedChildRestoresParentAndKeepsParentBlockState() throws Exception {
    ExecutionManager manager = new ExecutionManager();
    ActionLearning learning = new ActionLearning(manager, false);
    String parentName = "nestedCancelParent_" + System.nanoTime();
    String childName = parentName + "_child";
    Predicate parent = signature(parentName);
    Predicate child = signature(childName);

    ExecutorService executor = Executors.newFixedThreadPool(2);
    Future<Boolean> parentFuture = executor.submit(() -> learning.learnAction(parent));
    Future<Boolean> childFuture = null;
    try {
      learning.waitForActionLearningStart(parent);
      assertTrue(learning.beginLearnedIf(parent, Factory.createPredicate("true")));
      learning.addGoal(taughtGoal("parentThen"));

      childFuture = executor.submit(() -> learning.learnAction(child));
      learning.waitForActionLearningStart(child);
      learning.addGoal(taughtGoal("childPending"));
      assertTrue(learning.cancelActionLearning(child));

      assertTrue("Parent should be restored as active after child cancellation.",
          learning.beginLearnedElse(parent));
      learning.addGoal(taughtGoal("parentElse"));
      assertTrue(learning.endLearnedIf(parent));
      assertTrue(learning.endActionLearning(parent));
      assertTrue(parentFuture.get(3, TimeUnit.SECONDS));
      assertTrue(childFuture.get(3, TimeUnit.SECONDS));
    } finally {
      if (childFuture != null && !childFuture.isDone()) {
        childFuture.cancel(true);
      }
      executor.shutdownNow();
    }

    ActionDBEntry parentEntry = Database.getActionDB().getAction(parentName);
    ActionDBEntry childEntry = Database.getActionDB().getAction(childName);
    assertNotNull(parentEntry);
    assertNull("Canceled nested child must not publish.", childEntry);
    assertEquals(List.of("if", "true", "then", "parentThen", "else", "parentElse", "endif"),
        parentEntry.getEventSpecs().stream().map(EventSpec::getCommand).toList());
  }

  @Test(timeout = 20000)
  public void cancelDuringInFlightInstructionPreventsRecordingAndAllowsCleanRestart() throws Exception {
    ExecutionManager manager = new ExecutionManager();
    ActionLearning learning = new ActionLearning(manager, false);
    String actionName = "cancelInFlight_" + System.nanoTime();
    Predicate action = signature(actionName);
    ExecutorService executor = Executors.newSingleThreadExecutor();
    Future<Boolean> learnFuture = executor.submit(() -> learning.learnAction(action));
    try {
      learning.waitForActionLearningStart(action);
      CountDownLatch processingEntered = new CountDownLatch(1);
      CountDownLatch releaseProcessing = new CountDownLatch(1);
      AtomicBoolean first = new AtomicBoolean(true);
      learning.setBeforeQueuedGoalProcessingHookForTest(() -> {
        if (first.compareAndSet(true, false)) {
          processingEntered.countDown();
          try {
            await(releaseProcessing, "Timed out waiting to release queued-goal processing.");
          } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new RuntimeException(e);
          }
        }
      });
      learning.addGoal(taughtGoal("willBeCanceled"));
      await(processingEntered, "Timed out waiting for queued goal to start processing.");
      assertTrue(learning.cancelActionLearning(action));
      releaseProcessing.countDown();
      assertTrue(learnFuture.get(3, TimeUnit.SECONDS));
      assertTrue("Cancel should wake and stop an idle worker.",
          learning.waitForQueueWorkerStoppedForTest(3000));
    } finally {
      learning.setBeforeQueuedGoalProcessingHookForTest(null);
      executor.shutdownNow();
    }

    assertNull("Canceled draft must not publish.", Database.getActionDB().getAction(actionName));

    ExecutorService restartExecutor = Executors.newSingleThreadExecutor();
    Future<Boolean> restartFuture = restartExecutor.submit(() -> learning.learnAction(action));
    try {
      learning.waitForActionLearningStart(action);
      learning.addGoal(taughtGoal("afterRestart"));
      assertTrue(learning.endActionLearning(action));
      assertTrue(restartFuture.get(3, TimeUnit.SECONDS));
    } finally {
      restartExecutor.shutdownNow();
    }
    ActionDBEntry restarted = Database.getActionDB().getAction(actionName);
    assertNotNull(restarted);
    assertEquals(List.of("afterRestart"),
        restarted.getEventSpecs().stream().map(EventSpec::getCommand).toList());
  }

  @Test(timeout = 20000)
  public void cancelPausedSessionDoesNotDisturbOtherActiveSession() throws Exception {
    ExecutionManager manager = new ExecutionManager();
    ActionLearning learning = new ActionLearning(manager, false);
    Predicate actionA = signature("activeDuringPausedCancel_" + System.nanoTime());
    Predicate actionB = signature(actionA.getName() + "_paused");
    ExecutorService executor = Executors.newFixedThreadPool(2);
    Future<Boolean> learnA = executor.submit(() -> learning.learnAction(actionA));
    Future<Boolean> learnB = null;
    try {
      learning.waitForActionLearningStart(actionA);
      assertTrue(learning.pauseActionLearning(actionA));
      assertTrue(learnA.get(3, TimeUnit.SECONDS));

      learnB = executor.submit(() -> learning.learnAction(actionB));
      learning.waitForActionLearningStart(actionB);
      assertTrue(learning.pauseActionLearning(actionB));
      assertTrue(learnB.get(3, TimeUnit.SECONDS));

      assertTrue("Resuming A should activate its paused session.", learning.resumeActionLearning(actionA));
      assertTrue(learning.waitForQueueWorkerRunningForTest(3000));
      Thread activeWorker = learning.getQueueWorkerForTest();
      assertNotNull(activeWorker);
      assertTrue(activeWorker.isAlive());

      assertTrue("Canceling paused B should succeed.", learning.cancelActionLearning(actionB));
      assertEquals("Canceling paused B must leave A active.",
          ActionLearningStatus.ACTIVE, learning.getLearningStatus());
      assertSame("Canceling paused B must not replace A's queue worker.",
          activeWorker, learning.getQueueWorkerForTest());
      assertTrue(activeWorker.isAlive());

      learning.addGoal(taughtGoal("activeAStep"));
      assertTrue(learning.endActionLearning(actionA));
    } finally {
      executor.shutdownNow();
    }

    ActionDBEntry learnedA = Database.getActionDB().getAction(actionA.getName());
    assertNotNull(learnedA);
    assertEquals(List.of("activeAStep"),
        learnedA.getEventSpecs().stream().map(EventSpec::getCommand).toList());
    assertNull(Database.getActionDB().getAction(actionB.getName()));
  }

  @Test(timeout = 20000)
  public void cancellationAtEventSpecInsertionBoundaryPreventsDraftMutation() throws Exception {
    ExecutionManager manager = new ExecutionManager();
    ActionLearning learning = new ActionLearning(manager, false);
    String actionName = "cancelAtInsertion_" + System.nanoTime();
    Predicate action = signature(actionName);
    ExecutorService executor = Executors.newSingleThreadExecutor();
    Future<Boolean> learnFuture = executor.submit(() -> learning.learnAction(action));
    try {
      learning.waitForActionLearningStart(action);
      java.lang.reflect.Field currentStateField = ActionLearning.class.getDeclaredField("currentLearningState");
      currentStateField.setAccessible(true);
      LearningState draft = (LearningState) currentStateField.get(learning);
      CountDownLatch beforeInsertion = new CountDownLatch(1);
      CountDownLatch releaseInsertion = new CountDownLatch(1);
      learning.setBeforeQueuedGoalCreateEventSpecHookForTest(() -> {
        beforeInsertion.countDown();
        try {
          await(releaseInsertion, "Timed out waiting to release pre-insertion hook.");
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
          throw new RuntimeException(e);
        }
      });

      learning.addGoal(taughtGoal("mustNotBeRecorded"));
      await(beforeInsertion, "Timed out waiting for instruction to reach insertion boundary.");
      assertTrue(learning.cancelActionLearning(action));
      releaseInsertion.countDown();
      assertTrue(learnFuture.get(3, TimeUnit.SECONDS));
      assertTrue(learning.waitForQueueWorkerStoppedForTest(3000));

      assertTrue("Canceled instruction must not change the draft EventSpecs.",
          draft.getEventSpecs().isEmpty());
    } finally {
      learning.setBeforeQueuedGoalCreateEventSpecHookForTest(null);
      executor.shutdownNow();
    }
    assertNull(Database.getActionDB().getAction(actionName));
  }

  @Test(timeout = 20000)
  public void acceptedInstructionFailureBlocksFinalizationUntilSessionIsCanceled() throws Exception {
    ExecutionManager manager = new ExecutionManager();
    ActionLearning learning = new ActionLearning(manager, false);
    String actionName = "instructionFailureTracking_" + System.nanoTime();
    Predicate action = signature(actionName);
    ExecutorService executor = Executors.newSingleThreadExecutor();
    Future<Boolean> learnFuture = executor.submit(() -> learning.learnAction(action));
    try {
      learning.waitForActionLearningStart(action);
      AtomicBoolean failOnce = new AtomicBoolean(true);
      learning.setBeforeQueuedGoalCreateEventSpecHookForTest(() -> {
        if (failOnce.compareAndSet(true, false)) {
          throw new IllegalStateException("forced EventSpec creation failure");
        }
      });
      learning.addGoal(taughtGoal("failsToRecord"));
      assertFalse("Finalization must fail when an accepted instruction failed to record.",
          learning.endActionLearning(action));
      assertEquals("Session should remain active/recoverable after failed finalization.",
          ActionLearningStatus.ACTIVE, learning.getLearningStatus());
      assertNull(Database.getActionDB().getAction(actionName));
      assertTrue("Cancel should remain available after instruction-processing failure.",
          learning.cancelActionLearning(action));
      assertTrue(learnFuture.get(3, TimeUnit.SECONDS));
    } finally {
      learning.setBeforeQueuedGoalCreateEventSpecHookForTest(null);
      executor.shutdownNow();
    }
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
