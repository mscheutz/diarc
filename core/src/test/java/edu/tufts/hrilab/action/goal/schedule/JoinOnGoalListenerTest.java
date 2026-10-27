package edu.tufts.hrilab.action.goal.schedule;

import org.junit.Test;
import org.quartz.JobKey;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

import static org.junit.Assert.*;

public class JoinOnGoalListenerTest {
  @Test(timeout = 1000)
  public void deletionBeforeWaitIsRemembered() {
    JobKey key = new JobKey("regression");
    JoinOnGoalListener listener = new JoinOnGoalListener(key);
    listener.jobDeleted(key);
    assertTrue(listener.joinOnGoal());
    assertTrue(listener.joinOnGoal(0));
  }

  @Test(timeout = 1000)
  public void unrelatedDeletionDoesNotCompleteWait() {
    JoinOnGoalListener listener = new JoinOnGoalListener(new JobKey("regression"));
    listener.jobDeleted(new JobKey("unrelated"));
    assertFalse(listener.joinOnGoal(10));
  }

  @Test(timeout = 2000)
  public void deletionCompletesMultipleWaiters() throws Exception {
    JobKey key = new JobKey("regression");
    JoinOnGoalListener listener = new JoinOnGoalListener(key);
    ExecutorService executor = Executors.newFixedThreadPool(2);
    try {
      Future<Boolean> first = executor.submit(() -> listener.joinOnGoal());
      Future<Boolean> second = executor.submit(() -> listener.joinOnGoal(1000));
      listener.jobDeleted(key);
      assertTrue(first.get(1, TimeUnit.SECONDS));
      assertTrue(second.get(1, TimeUnit.SECONDS));
    } finally {
      executor.shutdownNow();
    }
  }
}
