package edu.tufts.hrilab.action.goal;

import edu.tufts.hrilab.fol.Factory;
import org.junit.Test;

import java.lang.reflect.Field;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;

import static org.junit.Assert.*;

public class PendingGoalWaitTest {
  private PendingGoal pending() {
    return new PendingGoal(Goal.createGoal(Factory.createPredicate("test(self)")));
  }

  @Test(timeout = 3000)
  public void spuriousSignalsDoNotEndTimedOrIndefiniteWaits() throws Exception {
    for (boolean timed : new boolean[]{true, false}) {
      PendingGoal pending = pending();
      ExecutorService executor = Executors.newSingleThreadExecutor();
      try {
        Future<Boolean> wait = executor.submit(() -> {
          if (timed) {
            return pending.waitForNoLongerPending(1000);
          }
          pending.waitForNoLongerPending();
          return true;
        });
        signalWithoutTransition(pending);
        try {
          wait.get(20, TimeUnit.MILLISECONDS);
          fail("Spurious signal completed the pending wait");
        } catch (TimeoutException expected) {
          assertFalse(wait.isDone());
        }
        pending.notifyOfNoLongerPending();
        assertTrue(wait.get(1, TimeUnit.SECONDS));
      } finally {
        executor.shutdownNow();
      }
    }
  }

  @Test(timeout = 2000)
  public void spuriousWakeupCannotProduceSuccessfulTimedWait() throws Exception {
    PendingGoal pending = pending();
    ExecutorService executor = Executors.newSingleThreadExecutor();
    try {
      Future<Boolean> wait = executor.submit(() -> pending.waitForNoLongerPending(150));
      signalWithoutTransition(pending);
      assertFalse(wait.get(1, TimeUnit.SECONDS));
    } finally {
      executor.shutdownNow();
    }
  }

  @Test
  public void completionBeforeRegistrationAndZeroTimeoutHaveExplicitPredicates() {
    PendingGoal pending = pending();
    assertFalse(pending.waitForNoLongerPending(0));
    pending.notifyOfNoLongerPending();
    assertTrue(pending.waitForNoLongerPending(0));
  }

  @Test
  public void interruptedWaitsPreserveTheThreadFlag() {
    for (boolean timed : new boolean[]{false, true}) {
      Thread.currentThread().interrupt();
      try {
        if (timed) {
          assertFalse(pending().waitForNoLongerPending(100));
        } else {
          pending().waitForNoLongerPending();
        }
        assertTrue(Thread.currentThread().isInterrupted());
      } finally {
        Thread.interrupted();
      }
    }
  }

  private void signalWithoutTransition(PendingGoal pending) throws Exception {
    Field lockField = PendingGoal.class.getDeclaredField("pendingLock");
    lockField.setAccessible(true);
    Lock lock = (Lock) lockField.get(pending);
    Field conditionField = PendingGoal.class.getDeclaredField("pendingLockCondition");
    conditionField.setAccessible(true);
    Condition condition = (Condition) conditionField.get(pending);
    long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(1);
    while (System.nanoTime() < deadline) {
      lock.lock();
      try {
        if (((java.util.concurrent.locks.ReentrantLock) lock).hasWaiters(condition)) {
          condition.signalAll();
          return;
        }
      } finally {
        lock.unlock();
      }
      Thread.yield();
    }
    fail("Pending waiter did not register");
  }
}
