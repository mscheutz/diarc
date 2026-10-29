/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2026.
 */
package edu.tufts.hrilab.action.learning;

import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.manager.ExecutionManager;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import org.junit.Test;

import java.util.List;
import java.lang.reflect.Field;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

import static org.junit.Assert.*;

public class ActionLearningStructuredTeachingTest {
  private static Predicate actionSignature(String name) {
    return Factory.createPredicate(name, "self:agent", "?object");
  }

  private static Goal taughtGoal(String command) {
    return Goal.createGoal(Factory.createPredicate(command, "self:agent", "?object"));
  }

  private static Goal taughtGoalNoArgs(String command) {
    return Goal.createGoal(Factory.createPredicate(command, "self:agent"));
  }

  private static Thread queueThread(ActionLearning learning) throws Exception {
    Field field = ActionLearning.class.getDeclaredField("goalQueueManager");
    field.setAccessible(true);
    return (Thread) field.get(learning);
  }

  @Test(timeout = 15000)
  public void explicitTeachingKeepsStepOrderAcrossBranchTransitions() throws Exception {
    ExecutionManager manager = new ExecutionManager();
    ActionLearning learning = new ActionLearning(manager, false);
    String actionName = "learnedStructured_" + System.nanoTime();
    Predicate signature = actionSignature(actionName);
    ExecutorService executor = Executors.newSingleThreadExecutor();
    Future<Boolean> learnFuture = executor.submit(() -> learning.learnAction(signature));
    try {
      learning.waitForActionLearningStart(signature);
      assertTrue(learning.beginLearnedIf(signature, Factory.createPredicate("true")));
      learning.addGoal(taughtGoal("branchThen"));
      assertTrue(learning.beginLearnedElse(signature));
      learning.addGoal(taughtGoal("branchElse"));
      assertTrue(learning.endLearnedIf(signature));
      learning.addGoal(taughtGoal("tailStep"));
      assertTrue(learning.endActionLearning(signature));
      assertTrue(learnFuture.get(3, TimeUnit.SECONDS));
    } finally {
      executor.shutdownNow();
    }

    ActionDBEntry learned = Database.getActionDB().getAction(actionName);
    assertNotNull("Expected learned action to be published.", learned);
    List<String> commands = learned.getEventSpecs().stream().map(es -> es.getCommand()).toList();
    assertEquals(List.of("if", "true", "then", "branchThen", "else", "branchElse", "endif", "tailStep"), commands);
  }

  @Test(timeout = 15000)
  public void unclosedBlocksPreventPublicationUntilClosed() throws Exception {
    ExecutionManager manager = new ExecutionManager();
    ActionLearning learning = new ActionLearning(manager, false);
    String actionName = "learnedUnclosed_" + System.nanoTime();
    Predicate signature = actionSignature(actionName);
    ExecutorService executor = Executors.newSingleThreadExecutor();
    Future<Boolean> learnFuture = executor.submit(() -> learning.learnAction(signature));
    try {
      learning.waitForActionLearningStart(signature);
      assertTrue(learning.beginLearnedIf(signature, Factory.createPredicate("true")));
      learning.addGoal(taughtGoal("inside"));
      assertFalse("Publishing should fail while IF is still open.", learning.endActionLearning(signature));
      assertEquals(ActionLearningStatus.ACTIVE, learning.getLearningStatus());
      assertTrue(learning.endLearnedIf(signature));
      assertTrue(learning.endActionLearning(signature));
      assertTrue(learnFuture.get(3, TimeUnit.SECONDS));
    } finally {
      executor.shutdownNow();
    }

    ActionDBEntry learned = Database.getActionDB().getAction(actionName);
    assertNotNull("Expected learned action after closing IF and ending learning.", learned);
  }

  @Test(timeout = 15000)
  public void explicitLoopAndLocalTeachingPublishesExpectedMarkers() throws Exception {
    ExecutionManager manager = new ExecutionManager();
    ActionLearning learning = new ActionLearning(manager, false);
    String actionName = "learnedLoops_" + System.nanoTime();
    Predicate signature = Factory.createPredicate(actionName, "self:agent", "?objects");
    ExecutorService executor = Executors.newSingleThreadExecutor();
    Future<Boolean> learnFuture = executor.submit(() -> learning.learnAction(signature));
    try {
      learning.waitForActionLearningStart(signature);
      assertTrue(learning.beginLearnedFor(signature, "!i", "0", "lt", "2", "++"));
      assertTrue(learning.declareLearnedLocal(signature, "!counter", "java.lang.Integer", "", "1"));
      learning.addGoal(taughtGoalNoArgs("forStep"));
      assertTrue(learning.endLearnedFor(signature));

      assertTrue(learning.beginLearnedForeach(signature, "!item", "?objects"));
      learning.addGoal(taughtGoalNoArgs("foreachStep"));
      assertTrue(learning.endLearnedForeach(signature));

      assertTrue(learning.beginLearnedWhile(signature, Factory.createPredicate("true")));
      learning.addGoal(taughtGoalNoArgs("whileStep"));
      assertTrue(learning.endLearnedWhile(signature));

      assertTrue(learning.endActionLearning(signature));
      assertTrue(learnFuture.get(3, TimeUnit.SECONDS));
    } finally {
      executor.shutdownNow();
    }

    ActionDBEntry learned = Database.getActionDB().getAction(actionName);
    assertNotNull(learned);
    List<String> commands = learned.getEventSpecs().stream().map(es -> es.getCommand()).toList();
    assertTrue(commands.contains("for"));
    assertTrue(commands.contains("foreach"));
    assertTrue(commands.contains("while"));
    assertTrue(commands.contains("declare"));
    assertTrue(commands.contains("assign"));
    assertTrue(commands.contains("endfor"));
    assertTrue(commands.contains("endwhile"));
  }

  @Test(timeout = 15000)
  public void pauseResumePreservesOpenStructuredBlockState() throws Exception {
    ExecutionManager manager = new ExecutionManager();
    ActionLearning learning = new ActionLearning(manager, false);
    String actionName = "learnedPauseResume_" + System.nanoTime();
    Predicate signature = actionSignature(actionName);
    ExecutorService executor = Executors.newSingleThreadExecutor();
    Future<Boolean> learnFuture = executor.submit(() -> learning.learnAction(signature));
    try {
      learning.waitForActionLearningStart(signature);
      assertTrue(learning.beginLearnedIf(signature, Factory.createPredicate("true")));
      learning.addGoal(taughtGoal("thenStep"));

      assertTrue(learning.pauseActionLearning(signature));
      assertEquals(ActionLearningStatus.NONE, learning.getLearningStatus());
      assertFalse("Cannot mutate paused learning session.", learning.beginLearnedElse(signature));

      assertTrue(learning.resumeActionLearning(signature));
      assertEquals(ActionLearningStatus.ACTIVE, learning.getLearningStatus());
      assertTrue(learning.beginLearnedElse(signature));
      learning.addGoal(taughtGoal("elseStep"));
      assertTrue(learning.endLearnedIf(signature));
      assertTrue(learning.endActionLearning(signature));
      assertTrue(learnFuture.get(3, TimeUnit.SECONDS));
    } finally {
      executor.shutdownNow();
    }

    ActionDBEntry learned = Database.getActionDB().getAction(actionName);
    assertNotNull(learned);
    List<String> commands = learned.getEventSpecs().stream().map(es -> es.getCommand()).toList();
    assertEquals(List.of("if", "true", "then", "thenStep", "else", "elseStep", "endif"), commands);
  }

  @Test(timeout = 15000)
  public void cancelDiscardsDraftAndDoesNotPublish() throws Exception {
    ExecutionManager manager = new ExecutionManager();
    ActionLearning learning = new ActionLearning(manager, false);
    String actionName = "learnedCancelled_" + System.nanoTime();
    Predicate signature = actionSignature(actionName);
    ExecutorService executor = Executors.newSingleThreadExecutor();
    Future<Boolean> learnFuture = executor.submit(() -> learning.learnAction(signature));
    try {
      learning.waitForActionLearningStart(signature);
      assertTrue(learning.beginLearnedIf(signature, Factory.createPredicate("true")));
      learning.addGoal(taughtGoal("draftStep"));
      assertTrue(learning.cancelActionLearning(signature));
      assertTrue(learnFuture.get(3, TimeUnit.SECONDS));
      assertEquals(ActionLearningStatus.NONE, learning.getLearningStatus());
    } finally {
      executor.shutdownNow();
    }

    assertNull("Canceled draft must not publish an action.", Database.getActionDB().getAction(actionName));
  }

  @Test(timeout = 15000)
  public void wrongSessionSignatureIsRejectedWithoutCorruptingActiveDraft() throws Exception {
    ExecutionManager manager = new ExecutionManager();
    ActionLearning learning = new ActionLearning(manager, false);
    String actionName = "learnedSessionIsolation_" + System.nanoTime();
    Predicate active = actionSignature(actionName);
    Predicate wrong = actionSignature(actionName + "_other");
    ExecutorService executor = Executors.newSingleThreadExecutor();
    Future<Boolean> learnFuture = executor.submit(() -> learning.learnAction(active));
    try {
      learning.waitForActionLearningStart(active);
      assertFalse(learning.beginLearnedIf(wrong, Factory.createPredicate("true")));
      assertTrue(learning.beginLearnedIf(active, Factory.createPredicate("true")));
      learning.addGoal(taughtGoal("stillActive"));
      assertTrue(learning.endLearnedIf(active));
      assertTrue(learning.endActionLearning(active));
      assertTrue(learnFuture.get(3, TimeUnit.SECONDS));
    } finally {
      executor.shutdownNow();
    }

    ActionDBEntry learned = Database.getActionDB().getAction(actionName);
    assertNotNull(learned);
    List<String> commands = learned.getEventSpecs().stream().map(es -> es.getCommand()).toList();
    assertEquals(List.of("if", "true", "then", "stillActive", "endif"), commands);
  }

  @Test(timeout = 15000)
  public void persistenceFailureRollsBackPublicationAndDraftCanStillFinish() throws Exception {
    ExecutionManager manager = new ExecutionManager();
    manager.setLearnedActionsFile("build");
    ActionLearning learning = new ActionLearning(manager, false);
    String actionName = "learnedPersistenceRollback_" + System.nanoTime();
    Predicate signature = actionSignature(actionName);
    ExecutorService executor = Executors.newSingleThreadExecutor();
    Future<Boolean> learnFuture = executor.submit(() -> learning.learnAction(signature));
    try {
      learning.waitForActionLearningStart(signature);
      learning.addGoal(taughtGoal("persistStep"));

      assertFalse("Publishing must fail when learned-actions path is not writable as a file.",
          learning.endActionLearning(signature));
      assertEquals("Draft should remain active after persistence failure.",
          ActionLearningStatus.ACTIVE, learning.getLearningStatus());
      assertNull("Failed persistence must roll back any DB publication.",
          Database.getActionDB().getAction(actionName));

      manager.setLearnedActionsFile(null);
      assertTrue("Draft should still be recoverable after failed persistence.",
          learning.endActionLearning(signature));
      assertTrue(learnFuture.get(3, TimeUnit.SECONDS));
    } finally {
      executor.shutdownNow();
    }

    assertNotNull("Action should publish after retry with valid persistence configuration.",
        Database.getActionDB().getAction(actionName));
  }

  @Test(timeout = 15000)
  public void queuedStepBeforeElseTransitionStaysInThenBranch() throws Exception {
    ExecutionManager manager = new ExecutionManager();
    ActionLearning learning = new ActionLearning(manager, false);
    String actionName = "learnedQueueOrdering_" + System.nanoTime();
    Predicate signature = actionSignature(actionName);
    ExecutorService executor = Executors.newSingleThreadExecutor();
    Future<Boolean> learnFuture = executor.submit(() -> learning.learnAction(signature));
    try {
      learning.waitForActionLearningStart(signature);
      assertTrue(learning.beginLearnedIf(signature, Factory.createPredicate("true")));

      // Submit then-branch taught step and immediately transition branch.
      // beginLearnedElse must drain the queue first, preserving deterministic ordering.
      learning.addGoal(taughtGoal("thenQueued"));
      assertTrue(learning.beginLearnedElse(signature));
      learning.addGoal(taughtGoal("elseStep"));
      assertTrue(learning.endLearnedIf(signature));
      assertTrue(learning.endActionLearning(signature));
      assertTrue(learnFuture.get(3, TimeUnit.SECONDS));
    } finally {
      executor.shutdownNow();
    }

    ActionDBEntry learned = Database.getActionDB().getAction(actionName);
    assertNotNull(learned);
    List<String> commands = learned.getEventSpecs().stream().map(es -> es.getCommand()).toList();
    assertEquals(List.of("if", "true", "then", "thenQueued", "else", "elseStep", "endif"), commands);
  }

  @Test(timeout = 15000)
  public void invalidBranchTransitionsAreRejectedWithoutCorruptingDraft() throws Exception {
    ExecutionManager manager = new ExecutionManager();
    ActionLearning learning = new ActionLearning(manager, false);
    String actionName = "learnedInvalidBranchOps_" + System.nanoTime();
    Predicate signature = actionSignature(actionName);
    ExecutorService executor = Executors.newSingleThreadExecutor();
    Future<Boolean> learnFuture = executor.submit(() -> learning.learnAction(signature));
    try {
      learning.waitForActionLearningStart(signature);

      assertFalse(learning.beginLearnedElse(signature));
      assertFalse(learning.beginLearnedElseIf(signature, Factory.createPredicate("true")));
      assertFalse(learning.endLearnedIf(signature));

      assertTrue(learning.beginLearnedIf(signature, Factory.createPredicate("true")));
      learning.addGoal(taughtGoal("thenStep"));
      assertTrue(learning.beginLearnedElse(signature));
      assertFalse("Duplicate else must be rejected.", learning.beginLearnedElse(signature));
      assertFalse("Else-if after else must be rejected.",
          learning.beginLearnedElseIf(signature, Factory.createPredicate("true")));
      learning.addGoal(taughtGoal("elseStep"));
      assertTrue(learning.endLearnedIf(signature));

      assertTrue(learning.endActionLearning(signature));
      assertTrue(learnFuture.get(3, TimeUnit.SECONDS));
    } finally {
      executor.shutdownNow();
    }

    ActionDBEntry learned = Database.getActionDB().getAction(actionName);
    assertNotNull(learned);
    List<String> commands = learned.getEventSpecs().stream().map(es -> es.getCommand()).toList();
    assertEquals(List.of("if", "true", "then", "thenStep", "else", "elseStep", "endif"), commands);
  }

  @Test(timeout = 20000)
  public void nestedLearningReusesSingleQueueConsumerAndPreservesSessionIsolation() throws Exception {
    ExecutionManager manager = new ExecutionManager();
    ActionLearning learning = new ActionLearning(manager, false);
    String parentName = "learnedParent_" + System.nanoTime();
    String childName = parentName + "_child";
    Predicate parent = actionSignature(parentName);
    Predicate child = actionSignature(childName);

    ExecutorService executor = Executors.newFixedThreadPool(2);
    Future<Boolean> parentFuture = executor.submit(() -> learning.learnAction(parent));
    Future<Boolean> childFuture = null;
    try {
      learning.waitForActionLearningStart(parent);
      learning.addGoal(taughtGoal("parentBeforeChild"));
      Thread consumerBefore = queueThread(learning);
      assertNotNull(consumerBefore);

      childFuture = executor.submit(() -> learning.learnAction(child));
      learning.waitForActionLearningStart(child);
      assertNotNull("Nested learning must keep a queue consumer available.", queueThread(learning));

      learning.addGoal(taughtGoal("childStep"));
      assertTrue(learning.endActionLearning(child));

      learning.addGoal(taughtGoal("parentAfterChild"));
      assertTrue(learning.endActionLearning(parent));
      assertTrue(childFuture.get(3, TimeUnit.SECONDS));
      assertTrue(parentFuture.get(3, TimeUnit.SECONDS));
    } finally {
      if (childFuture != null && !childFuture.isDone()) {
        childFuture.cancel(true);
      }
      executor.shutdownNow();
    }

    ActionDBEntry parentEntry = Database.getActionDB().getAction(parentName);
    ActionDBEntry childEntry = Database.getActionDB().getAction(childName);
    assertNotNull(parentEntry);
    assertNotNull(childEntry);
    List<String> parentCommands = parentEntry.getEventSpecs().stream().map(es -> es.getCommand()).toList();
    List<String> childCommands = childEntry.getEventSpecs().stream().map(es -> es.getCommand()).toList();
    assertEquals(List.of("parentBeforeChild", "parentAfterChild"), parentCommands);
    assertEquals(List.of("childStep"), childCommands);
  }

  @Test(timeout = 15000)
  public void pauseWaitsForPendingQueueWorkBeforeStashingSession() throws Exception {
    ExecutionManager manager = new ExecutionManager();
    ActionLearning learning = new ActionLearning(manager, false);
    String actionName = "learnedPauseDrain_" + System.nanoTime();
    Predicate signature = actionSignature(actionName);
    ExecutorService executor = Executors.newSingleThreadExecutor();
    Future<Boolean> learnFuture = executor.submit(() -> learning.learnAction(signature));
    try {
      learning.waitForActionLearningStart(signature);
      learning.addGoal(taughtGoal("queuedBeforePause"));
      assertTrue("Pause should wait for queued taught steps before stashing.",
          learning.pauseActionLearning(signature));
      assertTrue(learning.resumeActionLearning(signature));
      learning.addGoal(taughtGoal("afterResume"));
      assertTrue(learning.endActionLearning(signature));
      assertTrue(learnFuture.get(3, TimeUnit.SECONDS));
    } finally {
      executor.shutdownNow();
    }

    ActionDBEntry learned = Database.getActionDB().getAction(actionName);
    assertNotNull(learned);
    List<String> commands = learned.getEventSpecs().stream().map(es -> es.getCommand()).toList();
    assertEquals(List.of("queuedBeforePause", "afterResume"), commands);
  }

  @Test(timeout = 15000)
  public void endWithoutActiveSessionReturnsFailure() {
    ExecutionManager manager = new ExecutionManager();
    ActionLearning learning = new ActionLearning(manager, false);
    Predicate unknown = actionSignature("noActiveSession_" + System.nanoTime());
    assertFalse("Ending learning without an active or paused session must fail.",
        learning.endActionLearning(unknown));
  }

  @Test(timeout = 15000)
  public void resumeUnknownSessionReturnsFailureAndKeepsActiveSession() throws Exception {
    ExecutionManager manager = new ExecutionManager();
    ActionLearning learning = new ActionLearning(manager, false);
    String activeName = "activeResumeSession_" + System.nanoTime();
    Predicate active = actionSignature(activeName);
    Predicate unknown = actionSignature(activeName + "_missing");
    ExecutorService executor = Executors.newSingleThreadExecutor();
    Future<Boolean> learnFuture = executor.submit(() -> learning.learnAction(active));
    try {
      learning.waitForActionLearningStart(active);
      assertFalse("Resuming a nonexistent paused action must fail.", learning.resumeActionLearning(unknown));
      learning.addGoal(taughtGoal("stillLearning"));
      assertTrue(learning.endActionLearning(active));
      assertTrue(learnFuture.get(3, TimeUnit.SECONDS));
    } finally {
      executor.shutdownNow();
    }

    ActionDBEntry learned = Database.getActionDB().getAction(activeName);
    assertNotNull(learned);
    List<String> commands = learned.getEventSpecs().stream().map(es -> es.getCommand()).toList();
    assertEquals(List.of("stillLearning"), commands);
  }
}
