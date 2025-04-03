package edu.tufts.hrilab.action.goal;

import org.quartz.Job;
import org.quartz.JobExecutionContext;

public class ScheduledGoalJob implements Job {
  public static final String RUNNABLE_KEY = "RUNNABLE_KEY";

  public ScheduledGoalJob() {
    // Explicit constructor is required by Quartz.
  }

  @Override
  public void execute(JobExecutionContext jobExecutionContext) {
    final var runnable = (Runnable) jobExecutionContext.getJobDetail().getJobDataMap().get(RUNNABLE_KEY);
    runnable.run();
  }
}
