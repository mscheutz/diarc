package edu.tufts.hrilab.action.scheduler;

import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.goal.ScheduledGoal;
import edu.tufts.hrilab.action.manager.ExecutionManager;
import org.quartz.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class ScheduledGoalListener implements SchedulerListener {
  private Logger log = LoggerFactory.getLogger(ScheduledGoalListener.class);
  private ExecutionManager em;

  public ScheduledGoalListener(ExecutionManager executionManager) {
    this.em = executionManager;
  }

  @Override
  public void jobScheduled(Trigger trigger) {

  }

  @Override
  public void jobUnscheduled(TriggerKey triggerKey) {

  }

  @Override
  public void triggerFinalized(Trigger trigger) {
    log.info("Trigger finalized. Job key: {}", trigger.getJobKey());
    long goalId = Long.parseLong(trigger.getJobKey().getName()); // jobKey name should always be the goal ID
    ScheduledGoal goal = em.getScheduledGoal(goalId);

    // assume succeeded -- end time can only be specified for periodic goals, so can't be based on single
    // "normal" goal status (e.g., what to do if 2 out of 5 failed)
    goal.setStatus(GoalStatus.SUCCEEDED);

  }

  @Override
  public void triggerPaused(TriggerKey triggerKey) {

  }

  @Override
  public void triggersPaused(String s) {

  }

  @Override
  public void triggerResumed(TriggerKey triggerKey) {

  }

  @Override
  public void triggersResumed(String s) {

  }

  @Override
  public void jobAdded(JobDetail jobDetail) {

  }

  @Override
  public void jobDeleted(JobKey jobKey) {
    log.info("Job deleted. Job key: {}", jobKey);

  }

  @Override
  public void jobPaused(JobKey jobKey) {

  }

  @Override
  public void jobsPaused(String s) {

  }

  @Override
  public void jobResumed(JobKey jobKey) {

  }

  @Override
  public void jobsResumed(String s) {

  }

  @Override
  public void schedulerError(String s, SchedulerException e) {

  }

  @Override
  public void schedulerInStandbyMode() {

  }

  @Override
  public void schedulerStarted() {

  }

  @Override
  public void schedulerStarting() {

  }

  @Override
  public void schedulerShutdown() {

  }

  @Override
  public void schedulerShuttingdown() {

  }

  @Override
  public void schedulingDataCleared() {

  }
}
