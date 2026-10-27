package edu.tufts.hrilab.action.manager;

import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.SynchronousQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

/** Interrupts workers without falsely completing their execution futures. */
final class GoalManagerExecutor extends ThreadPoolExecutor {
  private final Set<Thread> running = ConcurrentHashMap.newKeySet();
  private volatile boolean interruptionRequested;

  GoalManagerExecutor() {
    super(0, Integer.MAX_VALUE, 60, TimeUnit.SECONDS, new SynchronousQueue<>());
  }

  @Override
  protected void beforeExecute(Thread thread, Runnable task) {
    super.beforeExecute(thread, task);
    running.add(thread);
    if (interruptionRequested) {
      thread.interrupt();
    }
  }

  @Override
  protected void afterExecute(Runnable task, Throwable failure) {
    running.remove(Thread.currentThread());
    super.afterExecute(task, failure);
  }

  void requestInterruption() {
    interruptionRequested = true;
    running.forEach(Thread::interrupt);
  }
}
