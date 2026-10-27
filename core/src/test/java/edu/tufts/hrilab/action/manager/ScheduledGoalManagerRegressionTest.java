package edu.tufts.hrilab.action.manager;

import edu.tufts.hrilab.action.goal.schedule.ScheduledGoal;
import edu.tufts.hrilab.fol.Factory;
import org.junit.Test;
import org.quartz.JobKey;
import org.quartz.ListenerManager;
import org.quartz.Scheduler;
import org.quartz.SchedulerListener;

import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.concurrent.TimeUnit;

import static org.junit.Assert.*;

public class ScheduledGoalManagerRegressionTest {
  @Test(timeout = 1000)
  public void catchUpPreservesLongPeriods() {
    long period = TimeUnit.DAYS.toMillis(30);
    Date now = new Date(TimeUnit.DAYS.toMillis(100));
    Date start = new Date(now.getTime() - TimeUnit.DAYS.toMillis(1));
    assertEquals(new Date(start.getTime() + period),
            ScheduledGoalManager.nextScheduledStart(start, now, period));
  }

  @Test(timeout = 1000)
  public void catchUpDoesNotIterateOverMissedOccurrences() {
    assertEquals(new Date(1_000_000_000_000L),
            ScheduledGoalManager.nextScheduledStart(new Date(0), new Date(1_000_000_000_000L), 1));
  }

  @Test
  public void catchUpPreservesIntervalAlignmentAndFutureStarts() {
    assertEquals(new Date(130), ScheduledGoalManager.nextScheduledStart(new Date(100), new Date(121), 10));
    assertEquals(new Date(120), ScheduledGoalManager.nextScheduledStart(new Date(100), new Date(120), 10));
    assertEquals(new Date(200), ScheduledGoalManager.nextScheduledStart(new Date(200), new Date(120), 10));
  }

  @Test(expected = IllegalArgumentException.class)
  public void catchUpRejectsZeroPeriod() {
    ScheduledGoalManager.nextScheduledStart(new Date(0), new Date(100), 0);
  }

  @Test(expected = ArithmeticException.class)
  public void catchUpReportsUnrepresentableDate() {
    ScheduledGoalManager.nextScheduledStart(new Date(Long.MAX_VALUE - 10), new Date(Long.MAX_VALUE - 1), 20);
  }

  @Test(timeout = 3000)
  public void joinsHandleDeletionBeforeRegistration() throws Exception {
    assertJoinHandlesDeletion(false);
  }

  @Test(timeout = 3000)
  public void joinsHandleDeletionBetweenExistenceCheckAndWait() throws Exception {
    assertJoinHandlesDeletion(true);
  }

  private void assertJoinHandlesDeletion(boolean deleteAfterCheck) throws Exception {
    ScheduledGoalManager manager = new ScheduledGoalManager(new ExecutionManager());
    Field schedulerField = ScheduledGoalManager.class.getDeclaredField("goalScheduler");
    schedulerField.setAccessible(true);
    Scheduler original = (Scheduler) schedulerField.get(manager);
    try {
      List<SchedulerListener> registered = new ArrayList<>();
      ListenerManager listeners = (ListenerManager) Proxy.newProxyInstance(
              getClass().getClassLoader(), new Class<?>[]{ListenerManager.class}, (proxy, method, args) -> {
                switch (method.getName()) {
                  case "addSchedulerListener":
                    registered.add((SchedulerListener) args[0]);
                    return null;
                  case "removeSchedulerListener":
                    return registered.remove(args[0]);
                  default:
                    throw new AssertionError("Unexpected listener operation: " + method.getName());
                }
              });
      Scheduler scheduler = (Scheduler) Proxy.newProxyInstance(
              getClass().getClassLoader(), new Class<?>[]{Scheduler.class}, (proxy, method, args) -> {
                switch (method.getName()) {
                  case "scheduleJob":
                    return new Date();
                  case "getListenerManager":
                    return listeners;
                  case "checkExists":
                    if (deleteAfterCheck) {
                      for (SchedulerListener listener : registered) {
                        listener.jobDeleted((JobKey) args[0]);
                      }
                      return true;
                    }
                    return false;
                  default:
                    throw new AssertionError("Unexpected scheduler operation: " + method.getName());
                }
              });
      schedulerField.set(manager, scheduler);
      ScheduledGoal goal = new ScheduledGoal(
              Factory.createPredicate("scheduled(regression(self),start(1,DAYS))"));
      manager.executeScheduledGoal(goal);
      manager.joinOnScheduledGoal(goal.getId());
      assertTrue(registered.isEmpty());
      manager.joinOnScheduledGoal(goal.getId(), 20);
      assertTrue(registered.isEmpty());
    } finally {
      original.shutdown();
    }
  }
}
