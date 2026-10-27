/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.goal.schedule;

import org.quartz.JobKey;
import org.quartz.listeners.SchedulerListenerSupport;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;

/**
 * This is a helper class to enable waiting for a scheduled goal to finish.
 */
public class JoinOnGoalListener extends SchedulerListenerSupport {
  private Logger log = LoggerFactory.getLogger(JoinOnGoalListener.class);
  private JobKey jobKey;
  private ReentrantLock lock = new ReentrantLock();
  private Condition condition = lock.newCondition();
  private boolean deleted;

  public JoinOnGoalListener(JobKey jobKey) {
    this.jobKey = jobKey;
  }

  public boolean joinOnGoal(long millis) {
    lock.lock();
    try {
      long remaining = TimeUnit.MILLISECONDS.toNanos(millis);
      while (!deleted) {
        if (remaining <= 0) {
          return false;
        }
        remaining = condition.awaitNanos(remaining);
      }
      return true;
    } catch (InterruptedException e) {
      log.error("Interrupted while waiting for goal to finish.", e);
      Thread.currentThread().interrupt();
      return false;
    } finally {
      lock.unlock();
    }
  }

  public boolean joinOnGoal() {
    lock.lock();
    try {
      while (!deleted) {
        condition.await();
      }
    } catch (InterruptedException e) {
      log.error("Interrupted while waiting for goal to finish.", e);
      Thread.currentThread().interrupt();
      return false;
    } finally {
      lock.unlock();
    }

    return true;
  }

  @Override
  public void jobDeleted(JobKey jobKey) {
    log.debug("Job deleted. Job key: {}", jobKey);

    if (jobKey.equals(this.jobKey)) {
      lock.lock();
      try {
        deleted = true;
        condition.signalAll();
      } finally {
        lock.unlock();
      }
    }
  }
}
