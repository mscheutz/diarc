/**
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.manager;

import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.action.goal.GoalStatus;
import edu.tufts.hrilab.action.justification.ConditionJustification;
import edu.tufts.hrilab.action.goal.schedule.ScheduledGoal;
import edu.tufts.hrilab.action.goal.schedule.JoinOnGoalListener;
import edu.tufts.hrilab.action.goal.schedule.ScheduledGoalJob;
import edu.tufts.hrilab.action.goal.schedule.ScheduledGoalListener;
import edu.tufts.hrilab.action.notification.GoalManagerNotifier;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import org.quartz.*;
import org.quartz.impl.StdSchedulerFactory;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.util.*;
import java.util.concurrent.TimeUnit;

public class ScheduledGoalManager {
  private final Logger log = LoggerFactory.getLogger(ScheduledGoalManager.class);

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
  private final Map<ScheduledGoal, JobKey> scheduledGoals = Collections.synchronizedMap(new LinkedHashMap<>());

  private final ExecutionManager em;

  private String scheduledGoalsFile = null;

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
    JobBuilder jobBuilder = JobBuilder.newJob(ScheduledGoalJob.class)
            .withIdentity(Long.toString(goal.getInfo().gid), "scheduledGoals")
            .setJobData(new JobDataMap(Map.of(ScheduledGoalJob.RUNNABLE_KEY,
                    (Runnable) () -> {
                      Goal newGoal = goal.getNextGoalToSubmit();
                      if (newGoal != null) {
                        em.submitScheduledOccurrence(goal, newGoal);
                      }
                    })));

    // schedule the goal
    JobDetail jobDetail = jobBuilder.build();
    Trigger trigger = createTrigger(goal, jobDetail.getKey());
    scheduledGoals.put(goal, jobDetail.getKey());
    try {
      goalScheduler.scheduleJob(jobDetail, trigger);
      goal.setStatus(GoalStatus.ACTIVE);
    } catch (SchedulerException e) {
      scheduledGoals.remove(goal);
      log.error("Error scheduling goal: {}", goal, e);
      goal.setFailureStatus(GoalStatus.FAILED, new ConditionJustification(false,
              Factory.createPredicate("scheduleRejected", goal.getGoalPredicate())));
    }
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

    //TODO: discuss before merging what the accepted behavior here should be
    //Periodic goal is scheduled with a start time in the past. Moving new start time to the next future time matching
    // the periodic interval. Doing this now mainly alongside persisting scheduled tasks across diarc shutdown/startup
    // so things don't always occur immediately on startup (ideally configure timings through calendar function)
    if (goal.isPeriodic() && goal.hasStartDate()) {
      Date startDate = goal.getStartDate();
      Date currentDate = new Date();
      //Find first matching future start date on periodic interval
      if (startDate.before(currentDate)) {
        log.warn("[createTrigger] scheduled goal's start date is in the past. Finding first matching future time based on periodic interval");
        Date nextStart = nextScheduledStart(startDate, currentDate, goal.getScheduledPeriod());
        if (goal.hasEndDate() && nextStart.after(goal.getEndDate())) {
          log.warn("[createTrigger] next matching scheduled interval time is past the configured end date");
        } else {
          triggerBuilder.startAt(nextStart);
        }
      }
    }

    triggerBuilder.forJob(jobKey);
    return triggerBuilder.build();
  }

  static Date nextScheduledStart(Date start, Date now, long period) {
    if (period <= 0) {
      throw new IllegalArgumentException("Scheduled period must be positive");
    }
    if (!start.before(now)) {
      return start;
    }
    long elapsed = Math.subtractExact(now.getTime(), start.getTime());
    long remainder = elapsed % period;
    return new Date(Math.addExact(now.getTime(), remainder == 0 ? 0 : period - remainder));
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
      log.error("Error looking up goal trigger. Goal: {}", goal.getGoalStatePredicate());
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
    if (jobKey == null) {
      return;
    }
    JoinOnGoalListener listener = new JoinOnGoalListener(jobKey);
    try {
      ListenerManager listeners = goalScheduler.getListenerManager();
      listeners.addSchedulerListener(listener);
      try {
        if (!goalScheduler.checkExists(jobKey)) {
          listener.jobDeleted(jobKey);
        }
        if (listener.joinOnGoal()) {
          Goal lastGoal = goal.getLastSubmittedGoal();
          if (lastGoal != null) {
            em.joinOnGoal(lastGoal.getId());
          }
        }
      } finally {
        listeners.removeSchedulerListener(listener);
      }
    } catch (SchedulerException e) {
      log.error("Error waiting for scheduled goal {}", gid, e);
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
    if (jobKey == null) {
      return;
    }
    JoinOnGoalListener listener = new JoinOnGoalListener(jobKey);
    try {
      ListenerManager listeners = goalScheduler.getListenerManager();
      listeners.addSchedulerListener(listener);
      long startTime = System.nanoTime();
      try {
        if (!goalScheduler.checkExists(jobKey)) {
          listener.jobDeleted(jobKey);
        }
        if (listener.joinOnGoal(millis)) {
          long remaining = Math.max(0, millis - TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - startTime));
          Goal lastGoal = goal.getLastSubmittedGoal();
          if (lastGoal != null) {
            em.joinOnGoal(lastGoal.getId(), remaining);
          }
        }
      } finally {
        listeners.removeSchedulerListener(listener);
      }
    } catch (SchedulerException e) {
      log.error("Error waiting for scheduled goal {}", gid, e);
    }
  }

  /**
   * Get ScheduledGoal with goal ID, or null.
   *
   * @param goalId
   * @return
   */
  public ScheduledGoal getScheduledGoal(long goalId) {
    synchronized (scheduledGoals) {
      return scheduledGoals.keySet().stream()
            .filter(goal -> goal.getInfo().gid == goalId)
            .findFirst()
            .orElse(null);
    }
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
   * Get a copied list of all scheduled goals which have either yet to be submitted, or are currently executing.
   *
   * @return shallow copy of scheduled goals
   */
  public List<ScheduledGoal> getScheduledGoals(Symbol actor) {
    AgentTeam agentTeam = em.getAgentTeam(actor);
    if (agentTeam == null) {
      return new ArrayList<>();
    }

    synchronized (scheduledGoals) {
      return new ArrayList<>(scheduledGoals.keySet()).stream().filter(sg -> agentTeam.getMember(sg.getActor()) != null).toList();
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
  public void cancelScheduledGoals() {
    Set<ScheduledGoal> goals;
    synchronized (scheduledGoals) {
      goals = new HashSet<>(scheduledGoals.keySet());
    }

    goals.forEach(this::cancelScheduledGoal);
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
      return scheduledGoals.remove(goal) != null;
    }
  }

  public void setScheduledGoalsFile(String filepath) {
    scheduledGoalsFile = filepath;
  }

  public void setGoalManagerNotifier(GoalManagerNotifier notifier) {
    em.setGoalManagerNotifier(notifier);
  }

  public void writeScheduledGoalsToFile() {
    if (scheduledGoalsFile != null) {
      List<Predicate> goalPredicates = new ArrayList<>();
      synchronized (scheduledGoals) {
        for (ScheduledGoal sg : scheduledGoals.keySet()) {
          goalPredicates.add(sg.getGoalPredicate(true));
        }
      }

      //Same TODO as MapComponent read/write of location references
      File f = new File(scheduledGoalsFile);
      try {
        if (!f.exists()) {
          f.getParentFile().mkdirs();
          f.createNewFile();
        }
        ObjectOutputStream os = new ObjectOutputStream(new FileOutputStream(f));
        os.writeObject(goalPredicates);
        os.flush();
        os.close();
      } catch (IOException e) {
        log.error("[writeScheduledGoalsToFile] error caching current scheduled goals", e);
      }
    }
  }

  public void shutdown() {
    try {
      goalScheduler.shutdown();
    } catch (SchedulerException e) {
      log.error("[shutdown] a scheduled goal was not correctly canceled during shutdown", e);
    }
  }
}
