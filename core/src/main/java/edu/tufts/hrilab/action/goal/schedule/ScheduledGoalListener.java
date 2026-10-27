/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.goal.schedule;

import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.manager.ExecutionManager;
import org.quartz.Trigger;
import org.quartz.listeners.SchedulerListenerSupport;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.List;

public class ScheduledGoalListener extends SchedulerListenerSupport {
  private Logger log = LoggerFactory.getLogger(ScheduledGoalListener.class);
  private ExecutionManager em;

  public ScheduledGoalListener(ExecutionManager executionManager) {
    this.em = executionManager;
  }

  /**
   * This gets called when a trigger (for a scheduled goal) will never be fired again.
   * This is the hook to clean up a terminated scheduled goal. Note that setting the goal status
   * here will cause the scheduled goal to be moved to past goals in the EM.
   *
   * @param trigger
   */
  @Override
  public void triggerFinalized(Trigger trigger) {
    log.debug("Trigger finalized. Job key: {}", trigger.getJobKey());

    long goalId = Long.parseLong(trigger.getJobKey().getName()); // jobKey name should always be the goal ID
    ScheduledGoal goal = em.getScheduledGoal(goalId);
    if (goal == null) {
      log.debug("Ignoring finalized trigger for removed goal {}", goalId);
      return;
    }

    // Completion can submit a coalesced occurrence before the previous future finishes.
    List<Goal> submitted;
    do {
      submitted = goal.getSubmittedGoals();
      submitted.forEach(g -> em.joinOnGoal(g.getId()));
    } while (!submitted.equals(goal.getSubmittedGoals()));

    // set scheduled goal status
    if (!goal.getStatus().isTerminated()) {
      // assume succeeded
      // TODO: should this reflect the submitted goal(s) status? (e.g., what to do if 2 out of 5 failed)
      goal.setStatus(GoalStatus.SUCCEEDED);
    }
  }
}
