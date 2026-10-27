package edu.tufts.hrilab.action.manager;

import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalInfo;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.goal.schedule.ScheduleInfo;
import edu.tufts.hrilab.action.goal.schedule.ScheduledGoal;
import edu.tufts.hrilab.action.goal.schedule.ScheduledGoalJob;
import edu.tufts.hrilab.action.goal.schedule.ScheduledGoalListener;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import org.junit.Test;
import org.quartz.JobDetail;
import org.quartz.Scheduler;
import org.quartz.TriggerBuilder;

import java.io.ObjectStreamClass;
import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

import static org.junit.Assert.*;

public class CoalescedRecurrenceTest {
  private ScheduledGoal schedule(boolean coalesce) {
    return new ScheduledGoal(Factory.createPredicate("scheduled(test(self),period(1,DAYS)"
            + (coalesce ? ",overlap(coalesce)" : "") + ")"));
  }

  @Test
  public void skipOverlapRemainsDefaultAndDoesNotRememberMissedTicks() {
    ScheduledGoal schedule = schedule(false);
    Goal first = schedule.getNextGoalToSubmit();
    assertNull(schedule.getNextGoalToSubmit());
    first.setStatus(GoalStatus.SUCCEEDED);
    assertNull(schedule.takeCoalescedGoal());
    assertEquals(1, schedule.getSubmittedGoals().size());
    assertNotNull(schedule.getNextGoalToSubmit());
  }

  @Test
  public void manyOverlappingTicksProduceExactlyOneNonoverlappingCatchUp() {
    ScheduledGoal schedule = schedule(true);
    Goal first = schedule.getNextGoalToSubmit();
    for (int i = 0; i < 100; i++) {
      assertNull(schedule.getNextGoalToSubmit());
    }
    assertNull(schedule.takeCoalescedGoal());
    first.setStatus(GoalStatus.SUCCEEDED);
    Goal second = schedule.takeCoalescedGoal();
    assertNotNull(second);
    assertNull(schedule.takeCoalescedGoal());
    assertEquals(List.of(first, second), schedule.getSubmittedGoals());
  }

  @Test
  public void closingRecurrenceDiscardsRememberedTickAndRejectsFutureTicks() {
    ScheduledGoal schedule = schedule(true);
    Goal first = schedule.getNextGoalToSubmit();
    assertNull(schedule.getNextGoalToSubmit());
    schedule.closeRecurrence();
    first.setStatus(GoalStatus.CANCELED);
    assertNull(schedule.takeCoalescedGoal());
    assertNull(schedule.getNextGoalToSubmit());
    assertEquals(1, schedule.getSubmittedGoals().size());
  }

  @Test
  public void endTimeDiscardsCatchUpInsteadOfExecutingLateWork() {
    ScheduledGoal schedule = schedule(true);
    Goal first = schedule.getNextGoalToSubmit();
    assertNull(schedule.getNextGoalToSubmit());
    assertTrue(schedule.update(Factory.createPredicate("end(01-01-2000 00-00-00)")));
    first.setStatus(GoalStatus.SUCCEEDED);
    assertNull(schedule.takeCoalescedGoal());
  }

  @Test
  public void disablingCoalescingDiscardsRememberedTickAndPersistsLegacyFormat() {
    ScheduledGoal schedule = schedule(true);
    Goal first = schedule.getNextGoalToSubmit();
    assertNull(schedule.getNextGoalToSubmit());
    assertTrue(schedule.update(Factory.createPredicate("overlap(skip)")));
    first.setStatus(GoalStatus.FAILED);
    assertNull(schedule.takeCoalescedGoal());
    assertFalse(schedule.getGoalPredicate(true).toString().contains("overlap"));
  }

  @Test
  public void policySurvivesPersistenceRoundTripAndSnapshotsAreDefensive() {
    ScheduledGoal original = schedule(true);
    original.update(Factory.createPredicate("end(1,DAYS)"));
    ScheduledGoal restored = new ScheduledGoal(original.getGoalPredicate(true));
    assertTrue(restored.getInfo().schedulingInfo.coalesce);
    assertTrue(restored.getDisplayPredicate().toString().contains("overlap(coalesce)"));
    GoalInfo snapshot = restored.getInfo();
    snapshot.schedulingInfo.coalesce = false;
    snapshot.schedulingInfo.endTime.setTime(0);
    assertTrue(restored.getInfo().schedulingInfo.coalesce);
    assertTrue(restored.getInfo().schedulingInfo.endTime.after(new Date()));
    assertEquals(7653594451099284288L, ObjectStreamClass.lookup(ScheduleInfo.class).getSerialVersionUID());
  }

  @Test(timeout = 3000)
  public void concurrentTicksReserveOnlyOneOccurrence() throws Exception {
    ScheduledGoal schedule = schedule(true);
    ExecutorService workers = Executors.newFixedThreadPool(4);
    try {
      List<Future<Goal>> ticks = new ArrayList<>();
      for (int i = 0; i < 100; i++) {
        ticks.add(workers.submit(schedule::getNextGoalToSubmit));
      }
      int created = 0;
      for (Future<Goal> tick : ticks) {
        if (tick.get() != null) {
          created++;
        }
      }
      assertEquals(1, created);
      assertEquals(1, schedule.getSubmittedGoals().size());
    } finally {
      workers.shutdownNow();
    }
  }

  @Test(expected = IllegalArgumentException.class)
  public void oneShotCoalescingIsRejected() {
    new ScheduledGoal(Factory.createPredicate("scheduled(test(self),start(1,DAYS),overlap(coalesce))"));
  }

  @Test
  public void invalidOverlapUpdateLeavesExistingPolicyUnchanged() {
    ScheduledGoal schedule = schedule(true);
    assertFalse(schedule.update(Factory.createPredicate("overlap(queue_all)")));
    assertTrue(schedule.getInfo().schedulingInfo.coalesce);
    long period = schedule.getScheduledPeriod();
    assertFalse(schedule.update(Factory.createPredicate("period(0,SECONDS)")));
    assertEquals(period, schedule.getScheduledPeriod());
  }

  @Test(timeout = 3000)
  public void quartzTickAndTerminalListenerWireCatchUpThroughNormalAdmission() throws Exception {
    Symbol self = Factory.createSymbol("self");
    CountDownLatch started = new CountDownLatch(2);
    ExecutionManager execution = new QueueExecutionManager() {
      @Override
      protected boolean executeGoal(Goal goal) {
        goal.setAsStarted();
        started.countDown();
        return true;
      }
    };
    execution.rootAgent = self;
    execution.agentHierarchy = Map.of(self, Set.of(self));
    execution.agentTeams.put(self, new AgentTeam(self, execution));
    ScheduledGoalManager manager = new ScheduledGoalManager(execution);
    Field managerField = ExecutionManager.class.getDeclaredField("scheduledGoalManager");
    managerField.setAccessible(true);
    managerField.set(execution, manager);
    Field schedulerField = ScheduledGoalManager.class.getDeclaredField("goalScheduler");
    schedulerField.setAccessible(true);
    Scheduler original = (Scheduler) schedulerField.get(manager);
    List<JobDetail> jobs = new ArrayList<>();
    Scheduler scheduler = (Scheduler) Proxy.newProxyInstance(getClass().getClassLoader(),
            new Class<?>[]{Scheduler.class}, (proxy, method, args) -> {
              if (method.getName().equals("scheduleJob")) {
                jobs.add((JobDetail) args[0]);
                return new Date();
              }
              throw new AssertionError("Unexpected scheduler call: " + method.getName());
            });
    schedulerField.set(manager, scheduler);
    try {
      ScheduledGoal schedule = schedule(true);
      assertTrue(schedule.update(Factory.createPredicate("end(1,DAYS)")));
      execution.submitGoal(schedule);
      assertTrue(manager.updateScheduledGoal(schedule.getId(), Factory.createPredicate("overlap(skip)")));
      assertTrue(manager.updateScheduledGoal(schedule.getId(), Factory.createPredicate("overlap(coalesce)")));
      Runnable tick = (Runnable) jobs.get(0).getJobDataMap().get(ScheduledGoalJob.RUNNABLE_KEY);
      tick.run();
      Goal first = schedule.getLastSubmittedGoal();
      assertEquals(GoalStatus.ACTIVE, first.getStatus());
      tick.run();
      tick.run();
      first.setStatus(GoalStatus.SUCCEEDED);
      assertTrue(started.await(1, TimeUnit.SECONDS));
      assertEquals(2, schedule.getSubmittedGoals().size());
      Goal second = schedule.getLastSubmittedGoal();
      assertEquals(GoalStatus.ACTIVE, second.getStatus());
      assertEquals(schedule.getEndDate().getTime(), second.getInfo().startDeadline);
      assertSame(second, execution.getGoal(second.getId()));
      second.setStatus(GoalStatus.SUCCEEDED);
      assertEquals(2, schedule.getSubmittedGoals().size());
    } finally {
      original.shutdown();
    }
  }

  @Test
  public void occurrenceReservedBeforeCancellationCannotBeAdmittedAfterward() {
    ExecutionManager manager = new ExecutionManager();
    ScheduledGoal schedule = schedule(true);
    Goal reserved = schedule.getNextGoalToSubmit();
    schedule.closeRecurrence();
    manager.submitScheduledOccurrence(schedule, reserved);
    assertEquals(GoalStatus.CANCELED, reserved.getStatus());
    assertNull(manager.getGoal(reserved.getId()));
  }

  @Test
  public void finalizationAlsoWaitsForCatchUpCreatedDuringTheLastCompletion() {
    ScheduledGoal schedule = schedule(true);
    Goal first = schedule.getNextGoalToSubmit();
    assertNull(schedule.getNextGoalToSubmit());
    List<Goal> joined = new ArrayList<>();
    ExecutionManager manager = new ExecutionManager() {
      @Override
      public ScheduledGoal getScheduledGoal(long id) {
        return schedule;
      }

      @Override
      public GoalStatus joinOnGoal(long id) {
        Goal current = schedule.getSubmittedGoals().stream()
                .filter(goal -> goal.getId() == id).findFirst().get();
        joined.add(current);
        current.setStatus(GoalStatus.SUCCEEDED);
        if (current == first) {
          schedule.takeCoalescedGoal();
        }
        return current.getStatus();
      }
    };
    new ScheduledGoalListener(manager).triggerFinalized(TriggerBuilder.newTrigger()
            .forJob(Long.toString(schedule.getId())).build());
    assertEquals(2, schedule.getSubmittedGoals().size());
    assertTrue(joined.contains(schedule.getLastSubmittedGoal()));
    assertEquals(GoalStatus.SUCCEEDED, schedule.getLastSubmittedGoal().getStatus());
    assertEquals(GoalStatus.SUCCEEDED, schedule.getStatus());
  }
}
