package edu.tufts.hrilab.action.manager;

import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.goal.schedule.ScheduledGoal;
import edu.tufts.hrilab.action.goal.schedule.JoinOnGoalListener;
import edu.tufts.hrilab.action.goal.schedule.ScheduledGoalJob;
import edu.tufts.hrilab.action.goal.schedule.ScheduledGoalListener;
import edu.tufts.hrilab.fol.Predicate;
import org.quartz.*;
import org.quartz.impl.StdSchedulerFactory;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;

public class ScheduledGoalManager {
  private Logger log = LoggerFactory.getLogger(ScheduledGoalManager.class);

  /**
   * Instance of quartz schedule for managing execution of scheduled goals.
   */
  private Scheduler goalScheduler;

  /**
   * Scheduled goals that are "active". One a scheduled goal is finished executing,
   * it's moved into pastGoals.
   * TODO: move this into AgentTeam?
   * TODO: should active goals include scheduled goals? currently they do not
   */
  private final Map<ScheduledGoal, JobKey> scheduledGoals = new HashMap<>();

  private ExecutionManager em;

  public ScheduledGoalManager(ExecutionManager executionManager) {
    this.em = executionManager;

    // instantiate and start scheduled goal manager
    try {
      goalScheduler = StdSchedulerFactory.getDefaultScheduler();
      goalScheduler.getListenerManager().addSchedulerListener(new ScheduledGoalListener(this.em));
      goalScheduler.start();
    } catch (SchedulerException e) {
      log.error("Error getting goal schedule. Scheduled goals not available at this time.");
    }
  }

  /**
   * Submit a new job for the schedule to handle execution of the scheduled goal.
   * Also, add the ScheduledGoal to the scheduledGoals map.
   *
   * @param goal ScheduledGoal to execute
   */
  public void executeScheduledGoal(ScheduledGoal goal) {
    // set to active goal status
    goal.setStatus(GoalStatus.ACTIVE);

    JobBuilder jobBuilder = JobBuilder.newJob(ScheduledGoalJob.class)
            .withIdentity(Long.toString(goal.getInfo().gid), "scheduledGoals")
            .setJobData(new JobDataMap(Map.of(ScheduledGoalJob.RUNNABLE_KEY,
                    (Runnable) () -> {
                      Goal newGoal = goal.getNextGoalToSubmit();
                      if (newGoal != null) {
                        em.submitGoal(newGoal);
                      }
                    })));

    // schedule the goal
    JobDetail jobDetail = jobBuilder.build();
    Trigger trigger = createTrigger(goal, jobDetail.getKey());
    try {
      goalScheduler.scheduleJob(jobDetail, trigger);
    } catch (SchedulerException e) {
      log.error("Error scheduling goal: {}", goal, e);
    }

    // add scheduled goals
    scheduledGoals.put(goal, jobDetail.getKey());
  }

  /**
   * Create a new scheduled goal job trigger based on the goal's scheduling information. This is used by the
   * quartz schedule for firing scheduled events.
   *
   * @param goal
   * @param jobKey
   * @return
   */
  private Trigger createTrigger(ScheduledGoal goal, JobKey jobKey) {
    TriggerBuilder<Trigger> triggerBuilder = TriggerBuilder.newTrigger();

    // set periodic schedule
    if (goal.isPeriodic()) {
      triggerBuilder.withSchedule(SimpleScheduleBuilder.simpleSchedule().withIntervalInMilliseconds(goal.getScheduledPeriod()).repeatForever());
    }

    // set start time
    if (goal.hasStartDate()) {
      triggerBuilder.startAt(goal.getStartDate());
    }

    // set end time
    if (goal.hasEndDate()) {
      triggerBuilder.endAt(goal.getEndDate());
    }

    triggerBuilder.forJob(jobKey);
    return triggerBuilder.build();
  }

  /**
   * Get Date of next scheduled goal execution. Returns null if goalId is
   * for a non-scheduled goal, or if scheduled goal will not execute again.
   *
   * @param goalId scheduled goal ID
   * @return Date or null
   */
  public Date getNextScheduledExecutionDate(long goalId) {
    ScheduledGoal goal = getScheduledGoal(goalId);
    if (goal == null) {
      return null;
    }

    // get goal's jobKey
    JobKey jobKey = scheduledGoals.get(goal);
    if (jobKey == null) {
      return null;
    }

    // get goal's triggers
    List<? extends Trigger> triggers = null;
    try {
      triggers = goalScheduler.getTriggersOfJob(jobKey);
    } catch (SchedulerException e) {
      log.error("Error looking up goal trigger. Goal: {}", goal.getPredicate());
    }
    if (triggers == null || triggers.size() != 1) {
      return null;
    }
    Trigger trigger = triggers.get(0);
    return trigger.getNextFireTime();
  }

  /**
   * Wait for the matching scheduled goal to terminate. Returns
   * immediately if the goal is not found or is not a scheduled goal.
   */
  public void joinOnScheduledGoal(long gid) {
    ScheduledGoal goal = getScheduledGoal(gid);
    if (goal == null) {
      return;
    }
    JobKey jobKey = scheduledGoals.get(goal);
    JoinOnGoalListener listener = new JoinOnGoalListener(jobKey);
    try {
      goalScheduler.getListenerManager().addSchedulerListener(listener);
      if (listener.joinOnGoal()) {
        Goal lastGoal = goal.getLastSubmittedGoal();
        em.joinOnGoal(lastGoal.getId());
      }
      goalScheduler.getListenerManager().removeSchedulerListener(listener);
    } catch (SchedulerException e) {
      log.error("Error trying to register new schedule listener.");
    }
  }

  /**
   * Wait (up to specified milliseconds) for the matching scheduled goal to terminate. Returns
   * immediately if the goal is not found or is not a scheduled goal.
   */
  public void joinOnScheduledGoal(long gid, long millis) {
    ScheduledGoal goal = getScheduledGoal(gid);
    if (goal == null) {
      return;
    }
    JobKey jobKey = scheduledGoals.get(goal);
    JoinOnGoalListener listener = new JoinOnGoalListener(jobKey);
    try {
      goalScheduler.getListenerManager().addSchedulerListener(listener);
      long startTime = System.currentTimeMillis();
      if (listener.joinOnGoal(millis)) {
        millis = millis - (System.currentTimeMillis() - startTime);
        Goal lastGoal = goal.getLastSubmittedGoal();
        em.joinOnGoal(lastGoal.getId(), millis);
      }
      goalScheduler.getListenerManager().removeSchedulerListener(listener);
    } catch (SchedulerException e) {
      log.error("Error trying to register new schedule listener.");
    }
  }

  /**
   * Get ScheduledGoal with goal ID, or null.
   *
   * @param goalId
   * @return
   */
  public ScheduledGoal getScheduledGoal(long goalId) {
    return scheduledGoals.keySet().stream()
            .filter(goal -> goal.getInfo().gid == goalId)
            .findFirst()
            .orElse(null);
  }

  /**
   * Get a copied list of all scheduled goals which have either yet to be submitted, or are currently executing.
   *
   * @return shallow copy of scheduled goals
   */
  public List<ScheduledGoal> getScheduledGoals() {
    synchronized (scheduledGoals) {
      return new ArrayList<>(scheduledGoals.keySet());
    }
  }

  /**
   * Modify existing scheduled goal based on update predicate.
   *
   * @param goalId ID of scheduled goal
   * @param update update predicate
   * @return if update was successful
   */
  public boolean updateScheduledGoal(long goalId, Predicate update) {
    ScheduledGoal goal = getScheduledGoal(goalId);
    if (goal == null) {
      return false;
    }

    if (!goal.update(update)) {
      return false;
    }

    // reschedule goal with new trigger
    JobKey jobKey = scheduledGoals.get(goal);
    Trigger newTrigger = createTrigger(goal, jobKey);
    try {
      // get trigger key for job (should be one for our use cases)
      List<? extends Trigger> jobTriggers = goalScheduler.getTriggersOfJob(jobKey);
      if (jobTriggers == null || jobTriggers.size() != 1) {
        log.error("Expecting exactly one trigger per job.");
        return false;
      }

      goalScheduler.rescheduleJob(jobTriggers.get(0).getKey(), newTrigger);
    } catch (SchedulerException e) {
      log.error("Error getting triggers for job key: {}", jobKey);
      return false;
    }

    return true;
  }

  /**
   * Cancel ScheduledGoal. This also cancels any active goals
   * that are executing as part of this scheduled goal.
   *
   * @param goal
   * @return
   */
  public boolean cancelScheduledGoal(ScheduledGoal goal) {
    // cancel scheduling more tasks for this goal
    JobKey key = scheduledGoals.get(goal);
    try {
      if (!goalScheduler.deleteJob(key)) {
        return false;
      }
    } catch (SchedulerException e) {
      log.error("Error looking up scheduled goal: {}", goal);
      return false;
    }

    // cancel goals that have been submitted as part of scheduled goal
    for (Goal submittedGoal : goal.getSubmittedGoals()) {
      if (!submittedGoal.getStatus().isTerminated()) {
        em.cancelGoal(submittedGoal.getId());
      }
    }

    goal.setStatus(GoalStatus.CANCELED);
    return true;
  }

  /**
   * Cancel all scheduled goals.
   */
  public void cancelAllScheduledGoals() {
    Set<ScheduledGoal> goals;
    synchronized (scheduledGoals) {
      goals = new HashSet<>(scheduledGoals.keySet());
    }

    goals.forEach(g -> cancelScheduledGoal(g));
  }

  /**
   * Remove a ScheduledGoal from the local container. This does not initiate canceling, and is purely
   * for bookkeeping purposes.
   *
   * @param goal
   * @return
   */
  public boolean removeGoal(ScheduledGoal goal) {
    synchronized (scheduledGoals) {
      return (scheduledGoals.remove(goal) == null) ? false : true;
    }
  }
}
