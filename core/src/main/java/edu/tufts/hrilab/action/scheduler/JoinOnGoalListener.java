package edu.tufts.hrilab.action.scheduler;

import org.quartz.JobKey;
import org.quartz.Trigger;
import org.quartz.listeners.SchedulerListenerSupport;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;

public class JoinOnGoalListener extends SchedulerListenerSupport {
  private Logger log = LoggerFactory.getLogger(JoinOnGoalListener.class);
  private JobKey jobKey;
  private ReentrantLock lock = new ReentrantLock();
  private Condition condition = lock.newCondition();

  public JoinOnGoalListener(JobKey jobKey) {
    this.jobKey = jobKey;
  }

  public boolean joinOnGoal(long millis) {
    lock.lock();
    try {
      return condition.await(millis, TimeUnit.MILLISECONDS);
    } catch (InterruptedException e) {
      log.error("Interrupted while waiting for goal to finish.");
      return false;
    } finally {
      lock.unlock();
    }
  }

  public boolean joinOnGoal() {
    lock.lock();
    try {
      condition.await();
    } catch (InterruptedException e) {
      log.error("Interrupted while waiting for goal to finish.");
      return false;
    } finally {
      lock.unlock();
    }

    return true;
  }

  @Override
  public void jobDeleted(JobKey jobKey) {
    log.info("Job deleted. Job key: {}", jobKey);
    if (jobKey.equals(this.jobKey)) {
      lock.lock();
      try {
        condition.signalAll();
      } finally {
        lock.unlock();
      }
    }
  }

//  @Override
//  public void triggerFinalized(Trigger trigger) {
//    log.info("Trigger finalized. Job key: {}", trigger.getJobKey());
//    if (jobKey.equals(trigger.getJobKey())) {
//      lock.lock();
//      try {
//        condition.signalAll();
//      } finally {
//        lock.unlock();
//      }
//    }
//  }
}
