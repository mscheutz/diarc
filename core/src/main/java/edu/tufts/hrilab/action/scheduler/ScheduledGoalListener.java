package edu.tufts.hrilab.action.scheduler;

import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.goal.ScheduledGoal;
import edu.tufts.hrilab.action.manager.ExecutionManager;
import org.quartz.*;
import org.quartz.listeners.SchedulerListenerSupport;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class ScheduledGoalListener extends SchedulerListenerSupport {
  private Logger log = LoggerFactory.getLogger(ScheduledGoalListener.class);
  private ExecutionManager em;

  public ScheduledGoalListener(ExecutionManager executionManager) {
    this.em = executionManager;
  }

  @Override
  public void triggerFinalized(Trigger trigger) {
    log.info("Trigger finalized. Job key: {}", trigger.getJobKey());
    long goalId = Long.parseLong(trigger.getJobKey().getName()); // jobKey name should always be the goal ID
    ScheduledGoal goal = em.getScheduledGoal(goalId);

    // wait for all submitted goals to finish
    goal.getSubmittedGoals().forEach(g -> em.joinOnGoal(g.getId()));

    // assume succeeded -- end time can only be specified for periodic goals, so can't be based on single
    // "normal" goal status (e.g., what to do if 2 out of 5 failed)
    if (!goal.getStatus().isTerminated()) {
      goal.setStatus(GoalStatus.SUCCEEDED);
    }
  }

  @Override
  public void jobDeleted(JobKey jobKey) {
    log.info("Job deleted. Job key: {}", jobKey);
  }
}
