package edu.tufts.hrilab.action.manager;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalRecoveryRecord;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.fol.Factory;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.lang.reflect.Field;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.UUID;
import java.util.concurrent.*;

import static org.junit.Assert.*;
import static edu.tufts.hrilab.action.manager.StageFiveExecutionManagerTest.goal;

public class GoalRecoveryTest {
  @Rule
  public TemporaryFolder temporary = new TemporaryFolder();

  private Path path(String name) {
    return temporary.getRoot().toPath().resolve(name);
  }

  private void closeRecovery(ExecutionManager manager) throws Exception {
    Field field = ExecutionManager.class.getDeclaredField("recoveryStore");
    field.setAccessible(true);
    GoalRecoveryStore store = (GoalRecoveryStore) field.get(manager);
    if (store != null) {
      store.close();
    }
  }

  @Test
  public void admissionIntentIsDurableBeforeStartupAndActiveClaimsAreRecorded() throws Exception {
    Path file = path("intent.json");
    StageFiveExecutionManagerTest.TestManager manager = new StageFiveExecutionManagerTest.TestManager() {
      @Override
      protected boolean executeGoal(Goal goal) {
        try {
          List<GoalRecoveryRecord> records = new GoalRecoveryStore(file).load();
          assertEquals(1, records.size());
          assertEquals(goal.getId(), records.get(0).goalId);
          assertEquals(GoalStatus.PENDING, records.get(0).status);
        } catch (IOException e) {
          throw new AssertionError(e);
        }
        return super.executeGoal(goal);
      }
    };
    manager.configureGoalRecovery(file.toString());
    try {
      Resource resource = new Resource(null, Factory.createSymbol("shared"));
      Goal goal = goal(resource, 1);
      manager.submitGoal(goal);
      List<GoalRecoveryRecord> records = new GoalRecoveryStore(file).load();
      assertEquals(GoalStatus.ACTIVE, records.get(0).status);
      assertEquals("shared", records.get(0).claims.get(0).resource);
      assertTrue(records.get(0).claims.get(0).held);
    } finally {
      closeRecovery(manager);
    }
  }

  @Test
  public void restartNeverReplaysAndBlocksAdmissionUntilExactSessionAndIdAreAcknowledged() throws Exception {
    Path original = path("original.json");
    Path restartedFile = path("restarted.json");
    StageFiveExecutionManagerTest.TestManager first = new StageFiveExecutionManagerTest.TestManager();
    StageFiveExecutionManagerTest.TestManager restarted = new StageFiveExecutionManagerTest.TestManager();
    first.configureGoalRecovery(original.toString());
    try {
      Goal old = goal(null, 1);
      first.submitGoal(old);
      Files.copy(original, restartedFile);
      restarted.configureGoalRecovery(restartedFile.toString());
      assertEquals(0, restarted.executions.get());
      assertNull(restarted.getGoal(old.getId()));
      GoalRecoveryRecord record = restarted.getGoalRecoveryRecords().get(0);
      try {
        restarted.submitGoal(goal(null, 1));
        fail("Recovered uncertainty must close admission");
      } catch (RejectedExecutionException expected) {
        assertEquals(0, restarted.executions.get());
      }
      assertFalse(restarted.acknowledgeRecoveredGoal(UUID.randomUUID().toString(), record.goalId));
      assertTrue(restarted.acknowledgeRecoveredGoal(record.sessionId, record.goalId));
      assertFalse(restarted.acknowledgeRecoveredGoal(record.sessionId, record.goalId));
      restarted.submitGoal(goal(null, 1));
      assertEquals(1, restarted.executions.get());
      assertEquals(GoalStatus.ACTIVE, old.getStatus());
    } finally {
      closeRecovery(first);
      closeRecovery(restarted);
    }
  }

  @Test
  public void unresolvedRecordsSurviveAnotherRestartInsteadOfBeingOverwrittenByAnEmptySession() throws Exception {
    Path file = path("carry.json");
    GoalRecoveryRecord expected = new GoalRecoveryRecord(UUID.randomUUID().toString(), 42,
            "test(self)", GoalStatus.ACTIVE, List.of());
    try (GoalRecoveryStore writer = new GoalRecoveryStore(file)) {
      writer.acquireWriter();
      writer.save(() -> List.of(expected), false);
    }
    StageFiveExecutionManagerTest.TestManager manager = new StageFiveExecutionManagerTest.TestManager();
    manager.configureGoalRecovery(file.toString());
    try {
      List<GoalRecoveryRecord> records = new GoalRecoveryStore(file).load();
      assertEquals(1, records.size());
      assertEquals(expected.sessionId, records.get(0).sessionId);
      assertEquals(expected.goalId, records.get(0).goalId);
    } finally {
      closeRecovery(manager);
    }
  }

  @Test
  public void corruptionFailsClosedAndDoesNotOverwriteTheBadFile() throws Exception {
    Path file = path("corrupt.json");
    String corrupt = "{\"version\":2,\"records\":[],\"cleanShutdown\":false}";
    Files.writeString(file, corrupt);
    StageFiveExecutionManagerTest.TestManager manager = new StageFiveExecutionManagerTest.TestManager();
    try {
      manager.configureGoalRecovery(file.toString());
      fail("Expected unsupported checkpoint format");
    } catch (UncheckedIOException expected) {
      assertEquals(corrupt, Files.readString(file));
    }
    try {
      manager.submitGoal(goal(null, 1));
      fail("Recovery failure must block admission");
    } catch (RejectedExecutionException expected) {
      assertEquals(0, manager.executions.get());
    } finally {
      closeRecovery(manager);
    }
  }

  @Test
  public void incompleteCheckpointCannotMasqueradeAsAnEmptyCleanSession() throws Exception {
    Path file = path("incomplete.json");
    Files.writeString(file, "{}");
    try {
      new GoalRecoveryStore(file).load();
      fail("Expected incomplete snapshot rejection");
    } catch (IOException expected) {
      assertTrue(expected.getMessage().contains("Incomplete"));
    }
  }

  @Test
  public void duplicateRecoveryIdentifiersAreRejected() throws Exception {
    Path file = path("duplicate.json");
    GoalRecoveryRecord record = new GoalRecoveryRecord(UUID.randomUUID().toString(), 7,
            "test(self)", GoalStatus.PENDING, List.of());
    try (GoalRecoveryStore writer = new GoalRecoveryStore(file)) {
      writer.acquireWriter();
      writer.save(() -> List.of(record, record), false);
    }
    try {
      new GoalRecoveryStore(file).load();
      fail("Expected duplicate record rejection");
    } catch (IOException expected) {
      assertTrue(expected.getMessage().contains("duplicate"));
    }
  }

  @Test
  public void checkpointFailurePreventsExecutionAndFurtherAdmission() throws Exception {
    Path file = path("unwritable.json");
    StageFiveExecutionManagerTest.TestManager manager = new StageFiveExecutionManagerTest.TestManager();
    manager.configureGoalRecovery(file.toString());
    Files.delete(file);
    Files.createDirectory(file);
    Goal rejected = goal(null, 1);
    try {
      manager.submitGoal(rejected);
      fail("Expected durable-intent write failure");
    } catch (UncheckedIOException expected) {
      assertEquals(GoalStatus.FAILED, rejected.getStatus());
      assertEquals(0, manager.executions.get());
    }
    try {
      manager.submitGoal(goal(null, 1));
      fail("Expected recovery fault admission gate");
    } catch (RejectedExecutionException expected) {
      assertEquals(0, manager.executions.get());
    } finally {
      closeRecovery(manager);
    }
  }

  @Test
  public void aSecondWriterCannotReplaceALiveManagersCheckpoint() throws Exception {
    Path file = path("exclusive.json");
    StageFiveExecutionManagerTest.TestManager first = new StageFiveExecutionManagerTest.TestManager();
    StageFiveExecutionManagerTest.TestManager second = new StageFiveExecutionManagerTest.TestManager();
    first.configureGoalRecovery(file.toString());
    String original = Files.readString(file);
    try {
      second.configureGoalRecovery(file.toString());
      fail("Expected exclusive writer rejection");
    } catch (UncheckedIOException expected) {
      assertEquals(original, Files.readString(file));
    } finally {
      closeRecovery(first);
      closeRecovery(second);
    }
  }

  @Test(timeout = 3000)
  public void snapshotCaptureAndWriteAreSerializedAgainstStaleOverwrites() throws Exception {
    Path file = path("ordered.json");
    GoalRecoveryRecord early = new GoalRecoveryRecord(UUID.randomUUID().toString(), 1,
            "first(self)", GoalStatus.PENDING, List.of());
    GoalRecoveryRecord late = new GoalRecoveryRecord(early.sessionId, 2,
            "second(self)", GoalStatus.ACTIVE, List.of());
    CountDownLatch captured = new CountDownLatch(1);
    CountDownLatch release = new CountDownLatch(1);
    ExecutorService callers = Executors.newFixedThreadPool(2);
    try (GoalRecoveryStore store = new GoalRecoveryStore(file)) {
      store.acquireWriter();
      Future<?> first = callers.submit(() -> {
        store.save(() -> {
          captured.countDown();
          try {
            assertTrue(release.await(2, TimeUnit.SECONDS));
          } catch (InterruptedException e) {
            throw new AssertionError(e);
          }
          return List.of(early);
        }, false);
        return null;
      });
      assertTrue(captured.await(1, TimeUnit.SECONDS));
      Future<?> second = callers.submit(() -> {
        store.save(() -> List.of(late), false);
        return null;
      });
      release.countDown();
      first.get(1, TimeUnit.SECONDS);
      second.get(1, TimeUnit.SECONDS);
      assertEquals(2, store.load().get(0).goalId);
    } finally {
      release.countDown();
      callers.shutdownNow();
    }
  }

  @Test(timeout = 5000)
  public void cleanShutdownClearsFinishedWorkAndReleasesWriterOwnership() throws Exception {
    Path file = path("clean.json");
    StageFiveExecutionManagerTest.TestManager manager = new StageFiveExecutionManagerTest.TestManager();
    StageFiveExecutionManagerTest.installInfrastructure(manager);
    manager.configureGoalRecovery(file.toString());
    manager.submitGoal(goal(null, 1));
    assertEquals(edu.tufts.hrilab.action.goal.GoalManagerShutdownReport.Outcome.COMPLETE,
            manager.shutdownWithTimeout(2000, 0).outcome);
    assertTrue(new GoalRecoveryStore(file).load().isEmpty());
    assertTrue(Files.readString(file).contains("\"cleanShutdown\":true"));
    try (GoalRecoveryStore replacement = new GoalRecoveryStore(file)) {
      replacement.acquireWriter();
    }
  }

  @Test(expected = IllegalStateException.class)
  public void recoveryCannotBeEnabledAfterAdmission() {
    StageFiveExecutionManagerTest.TestManager manager = new StageFiveExecutionManagerTest.TestManager();
    manager.submitGoal(goal(null, 1));
    manager.configureGoalRecovery(path("late.json").toString());
  }

  @Test
  public void configurationAndRecoveryInventoryAreCallableThroughTrade() throws Exception {
    StageFiveExecutionManagerTest.TestManager manager = new StageFiveExecutionManagerTest.TestManager();
    GoalManagerComponent component = new GoalManagerComponent();
    Field field = GoalManagerComponent.class.getDeclaredField("em");
    field.setAccessible(true);
    field.set(component, manager);
    TRADE.registerAllServices(component, List.of());
    try {
      TRADE.getAvailableService(new TRADEServiceConstraints().name("configureGoalLimits")
                      .argTypes(Integer.class, Integer.class)).call(void.class, 1, 2);
      TRADE.getAvailableService(new TRADEServiceConstraints().name("configureGoalRecovery")
                      .argTypes(String.class)).call(void.class, path("trade.json").toString());
      List<?> inventory = TRADE.getAvailableService(new TRADEServiceConstraints()
              .name("getGoalRecoveryRecords")).call(List.class);
      assertTrue(inventory.isEmpty());
      assertFalse(TRADE.getAvailableService(new TRADEServiceConstraints().name("acknowledgeRecoveredGoal")
                      .argTypes(String.class, Long.class)).call(Boolean.class, "unknown", 1L));
    } finally {
      TRADE.deregister(component);
      closeRecovery(manager);
    }
  }
}
