package edu.tufts.hrilab.action.manager;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.RejectedExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.Assert.*;

public class ManagerCoordinatorTest {
  @Test(timeout = 5000)
  public void concurrentCommandsHaveOneOwnerAndNeverOverlap() throws Exception {
    try (ManagerCoordinator coordinator = new ManagerCoordinator()) {
      ExecutorService callers = Executors.newFixedThreadPool(8);
      AtomicInteger active = new AtomicInteger();
      AtomicInteger count = new AtomicInteger();
      Set<Thread> owners = ConcurrentHashMap.newKeySet();
      try {
        List<Future<?>> futures = new ArrayList<>();
        for (int i = 0; i < 100; i++) {
          futures.add(callers.submit(() -> coordinator.run(() -> {
            assertTrue(coordinator.isOwner());
            assertEquals(1, active.incrementAndGet());
            owners.add(Thread.currentThread());
            count.incrementAndGet();
            assertEquals(0, active.decrementAndGet());
          })));
        }
        for (Future<?> future : futures) {
          future.get(2, TimeUnit.SECONDS);
        }
        assertEquals(100, count.get());
        assertEquals(1, owners.size());
      } finally {
        callers.shutdownNow();
      }
    }
  }

  @Test(timeout = 3000)
  public void effectsRunAfterCommitAndCanWaitForAReentrantCommandOnAnotherThread() {
    try (ManagerCoordinator coordinator = new ManagerCoordinator()) {
      ExecutorService caller = Executors.newSingleThreadExecutor();
      AtomicInteger value = new AtomicInteger();
      try {
        coordinator.run(() -> {
          value.set(1);
          coordinator.defer(() -> {
            assertFalse(coordinator.isOwner());
            try {
              assertEquals(Integer.valueOf(2), caller.submit(
                      () -> coordinator.call(value::incrementAndGet)).get(1, TimeUnit.SECONDS));
            } catch (Exception e) {
              throw new AssertionError(e);
            }
          });
        });
        assertEquals(2, value.get());
      } finally {
        caller.shutdownNow();
      }
    }
  }

  @Test
  public void failuresDoNotDropCommittedEffectsOrPoisonLaterCommands() {
    try (ManagerCoordinator coordinator = new ManagerCoordinator()) {
      AtomicInteger effects = new AtomicInteger();
      try {
        coordinator.run(() -> {
          coordinator.defer(() -> {
            effects.incrementAndGet();
            throw new IllegalArgumentException("effect");
          });
          coordinator.defer(effects::incrementAndGet);
          throw new IllegalStateException("command");
        });
        fail("Expected command failure");
      } catch (IllegalStateException expected) {
        assertEquals("command", expected.getMessage());
        assertEquals("effect", expected.getSuppressed()[0].getMessage());
      }
      assertEquals(2, effects.get());
      assertEquals(Integer.valueOf(3), coordinator.call(effects::incrementAndGet));
    }
  }

  @Test(timeout = 3000)
  public void interruptedCallerFinishesItsCommittedEffectsAndPreservesTheFlag() throws Exception {
    try (ManagerCoordinator coordinator = new ManagerCoordinator()) {
      CountDownLatch committed = new CountDownLatch(1);
      CountDownLatch release = new CountDownLatch(1);
      AtomicBoolean effected = new AtomicBoolean();
      AtomicBoolean interrupted = new AtomicBoolean();
      Thread caller = new Thread(() -> {
        coordinator.run(() -> {
          coordinator.defer(() -> effected.set(true));
          committed.countDown();
          try {
            assertTrue(release.await(1, TimeUnit.SECONDS));
          } catch (InterruptedException e) {
            throw new AssertionError(e);
          }
        });
        interrupted.set(Thread.currentThread().isInterrupted());
      });
      caller.start();
      try {
        assertTrue(committed.await(1, TimeUnit.SECONDS));
        caller.interrupt();
        release.countDown();
        caller.join(1000);
        assertFalse(caller.isAlive());
        assertTrue(effected.get());
        assertTrue(interrupted.get());
      } finally {
        release.countDown();
        caller.interrupt();
      }
    }
  }

  @Test(expected = RejectedExecutionException.class)
  public void closedCoordinatorRejectsNewCommandsExplicitly() {
    ManagerCoordinator coordinator = new ManagerCoordinator();
    coordinator.close();
    coordinator.run(() -> fail("Closed command executed"));
  }
}
