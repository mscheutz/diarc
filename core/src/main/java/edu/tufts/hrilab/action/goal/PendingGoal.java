/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.goal;

import edu.tufts.hrilab.action.execution.ExecutionType;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class PendingGoal {
  private static final Logger log = LoggerFactory.getLogger(PendingGoal.class);
  private final Goal goal;
  private final ExecutionType executionType;
  private volatile boolean isPending = true;
  private final Lock pendingLock = new ReentrantLock();
  private final Condition pendingLockCondition = pendingLock.newCondition();
  //(Assuming we are keeping old AIs) Need way to keep track of superseded goals' original futures
  private Future previousAIFuture = null;
  private GoalStatus previousAIStatus;

  public PendingGoal(Goal goal, ExecutionType executionType) {
    this.goal = goal;
    this.executionType = executionType;
  }

  public Goal getGoal() {
    return goal;
  }

  public ExecutionType getExecutionType() {
    return executionType;
  }

  /**
   * Wait indefinitely for goal to no longer be pending.
   *
   */
  public void waitForNoLongerPending() {
    pendingLock.lock();
    try {
      if (isPending) {
        pendingLockCondition.await();
      }
    } catch (InterruptedException e) {
      log.error("[waitForNoLongerPending] Exception while waiting.", e);
    } finally {
      pendingLock.unlock();
    }
  }

  /**
   * Wait (up to specified milliseconds) for goal to no longer be pending.
   *
   * @param millis
   * @return true: goal is no longer pending. false: goal is pending
   */
  public boolean waitForNoLongerPending(long millis) {
    pendingLock.lock();
    try {
      if (isPending) {
        return pendingLockCondition.await(millis, TimeUnit.MILLISECONDS);
      } else {
        return true;
      }
    } catch (InterruptedException e) {
      log.error("[waitForNoLongerPending] Exception while waiting.", e);
      return false;
    } finally {
      pendingLock.unlock();
    }
  }

  /**
   * This method will notify all waiting threads (i.e., callers of waitForNoLongerPending).
   */
  public void notifyOfNoLongerPending() {
    pendingLock.lock();
    try {
      isPending = false;
      pendingLockCondition.signalAll();
    } finally {
      pendingLock.unlock();
    }
  }

  public Future getPreviousAIFuture() {
    return previousAIFuture;
  }

  public void setPreviousAIFuture(Future previousAIFuture) {
    this.previousAIFuture = previousAIFuture;
  }

  public void setPreviousStatus(GoalStatus previousAIStatus) {
    this.previousAIStatus = previousAIStatus;
  }

  public GoalStatus getPreviousStatus() {
    return previousAIStatus;
  }
}
