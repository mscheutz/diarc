package edu.tufts.hrilab.action.goal;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class ScheduledGoal extends Goal {

  private static final Logger log = LoggerFactory.getLogger(ScheduledGoal.class);
  private Predicate start;
  private Predicate period;
  private Predicate end;

  /**
   * Schedule information, extracted from the start, period, and end predicates.
   */
  public ScheduleInfo scheduleInfo = new ScheduleInfo();

  private List<Goal> submittedGoals = new ArrayList<>();

  public ScheduledGoal(Predicate goal, Predicate start, Predicate period, Predicate end) {
    super(goal);
    setScheduleInfo(start, period, end);
  }

  private void setScheduleInfo(Predicate start, Predicate period, Predicate end) {
    this.start = start;
    this.period = period;
    this.end = end;

    // TODO: set scheduleInfo
    if (start != null) {
      scheduleInfo.initialDelay = 10;
    }

    if (period != null) {
      scheduleInfo.period = 100;
    }

    if (end != null) {
      scheduleInfo.endTime = 1000;
    }
  }

  public Goal getNextGoalToExecute() {

  }

  @Override
  public boolean isScheduled() {
    return true;
  }

  public boolean isPeriodic() {
    return scheduleInfo.period >= 0;
  }

  public long getScheduledDelay() {
  }

  /**
   * Only applicable to Persistently Scheduled goals; Temporarily modifies the delay until the next execution of this
   * goal without affecting the regular interval. This method can be used to shift execution timing without affecting
   * the interval.
   */
  public boolean updateNextScheduledTime(long delay, TimeUnit timeUnit) {
  }

  /**
   * Only applicable to Persistently Scheduled goals; Temporarily modifies the delay until the next execution of this
   * goal without affecting the regular interval. This method can be used to shift execution timing without affecting
   * the interval.
   */
  public boolean updateNextScheduledTime(Date date) {
  }

  /**
   * Updates the scheduled delay of this goal. If this goal is persistently scheduled, the supplied delay will
   * be the interval at which this task now executes
   */
  public boolean setScheduledDelay(Long delay, TimeUnit timeUnit) {
  }

  /**
   * Updates the scheduled date of this goal. If this goal is persistently scheduled, the supplied date will
   * contain the time of day this task executes every day.
   */
  public boolean setScheduledDate(Date scheduledDate) {
  }

  public TimeUnit getScheduledTimeUnit() {
    return info.scheduleInfo.scheduleTimeUnit;
  }

  public Date getScheduledDate() {
  }

  /**
   * Get the duration in milliseconds until this goal's next scheduled execution.
   * Returns -1 if the goal is not a scheduled goal.
   */
  public long getTimeUntilScheduledExecution() {
  }

  public Date getNextScheduledExecutionDate() {
  }

  /**
   * Utility function to get the re-wrapped predicate this goal was originally submitted with.
   * Practically, this method will return the base goal predicate and optionally 'persistent' and 'scheduled' wrappers
   * along with any scheduled timing information if applicable.
   */
  public Predicate getWrappedGoalSubmissionPredicate() {
    Predicate goalPred;
    if (info.goal == null) {
      goalPred = ((Predicate) info.getUnboundGoal()).clone();
    } else {
      goalPred = info.goal.clone();
    }
    if (info.isScheduled()) {
      if (info.scheduledDelay != null) {
        Symbol delaySymbol = Factory.createSymbol(info.scheduledDelay.toString());
        Symbol timeUnitSymbol = Factory.createSymbol(info.scheduledDelayTimeUnit.toString());
        goalPred = Factory.createPredicate("scheduled", info.actor, goalPred, delaySymbol, timeUnitSymbol);
      } else {
        String pattern = "HH-mm-ss";
        DateFormat df = new SimpleDateFormat(pattern);
        Symbol dateSymbol = Factory.createSymbol(df.format(info.scheduledDate));
        goalPred = Factory.createPredicate("scheduled", info.actor, goalPred, dateSymbol);
      }
    }
    if (info.isPersistent()) {
      goalPred = Factory.createPredicate("persistent", info.actor, goalPred);
    }
    if (info.goalType == GoalType.SCHEDULEDPERSISTENTSCHEDULED) {
      if (info.getScheduledSubmissionDelay() != null) {
        Symbol delaySymbol = Factory.createSymbol(info.scheduledSubmissionDelay.toString());
        Symbol timeUnitSymbol = Factory.createSymbol(info.scheduledSubmissionDelayTimeUnit.toString());
        goalPred = Factory.createPredicate("scheduled", info.actor, goalPred, delaySymbol, timeUnitSymbol);
      } else {
        String pattern = "HH-mm-ss";
        DateFormat df = new SimpleDateFormat(pattern);
        Symbol dateSymbol = Factory.createSymbol(df.format(info.scheduledSubmissionDate));
        goalPred = Factory.createPredicate("scheduled", info.actor, goalPred, dateSymbol);
      }
    }
    return goalPred;
  }

  /**
   * Utility function to get a wrapped predicate for this goal including all current scheduling information. Behaves
   * like {@link #getWrappedGoalSubmissionPredicate()} except that any timing information is relative to now
   * rather than submission
   */
  public Predicate getWrappedGoalPredicate() {
    Predicate goalPred;
    if (info.status == GoalStatus.PENDING || info.status == GoalStatus.UNKNOWN || info.status.isTerminated()) {
      log.warn("[getWrappedGoalPredicate] not sure how to handle goal status {}, returning submission predicate", info.status);
      return getWrappedGoalSubmissionPredicate();
    }
    if (info.goal == null) {
      goalPred = ((Predicate) info.getUnboundGoal()).clone();
    } else {
      goalPred = info.goal.clone();
    }
    if (info.isScheduled() && info.isPersistent()) {
      if (info.scheduledDelay != null) {
        Symbol delaySymbol = Factory.createSymbol(info.scheduledDelay.toString());
        Symbol timeUnitSymbol = Factory.createSymbol(info.scheduledDelayTimeUnit.toString());
        goalPred = Factory.createPredicate("scheduled", info.actor, goalPred, delaySymbol, timeUnitSymbol);
      } else {
        String pattern = "HH-mm-ss";
        DateFormat df = new SimpleDateFormat(pattern);
        Symbol dateSymbol = Factory.createSymbol(df.format(info.scheduledDate));
        goalPred = Factory.createPredicate("scheduled", info.actor, goalPred, dateSymbol);
      }
    }
    if (info.isPersistent()) {
      goalPred = Factory.createPredicate("persistent", info.actor, goalPred);
    }
    //TODO: should this always be a date or should logic be written to get the "best" representation of delay
    //  duration and timeunit from a general time.
    if (info.isScheduled()) {
      ScheduledContext scheduledContext = getScheduledContext();
      if (scheduledContext != null) {
        if (isPersistent() && scheduledContext.scheduledByDate() && !scheduledContext.hasOffsetSchedule() && !scheduledContext.nextScheduleIntervalOffset()) {
          return goalPred;
        }
        Date date = scheduledContext.getDateOfNextExecution();
        String pattern = "HH-mm-ss";
        DateFormat df = new SimpleDateFormat(pattern);
        Symbol dateSymbol = Factory.createSymbol(df.format(date));
        goalPred = Factory.createPredicate("scheduled", info.actor, goalPred, dateSymbol);
      }
    }

    return goalPred;
  }
}
