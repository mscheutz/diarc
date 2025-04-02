package edu.tufts.hrilab.action.goal;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.text.DateFormat;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.*;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

public class ScheduledGoal extends Goal {

  private static final Logger log = LoggerFactory.getLogger(ScheduledGoal.class);
  private Predicate start;
  private Predicate period;
  private Predicate end;

  /**
   * Schedule information, extracted from the start, period, and end predicates.
   */
  private ScheduleInfo scheduleInfo = new ScheduleInfo();

  private List<Goal> submittedGoals = new ArrayList<>();

  /**
   * Scheduled goal form: scheduled(goalPred, startPred, periodPred, endPred)
   * - only goalPred and startPred OR periodPred are required
   * - no startTime means start right away
   * - endPred only valid if periodPred is specified
   * - startPred forms: start(number, timeunit), start(date-format)
   * - periodPred forms: period(number,timeunit)
   * - endPred forms: end(number, timeunit), end(date-format)
   *
   * @param goalState
   */
  protected ScheduledGoal(Predicate goalState) {
    super((Predicate) goalState.get(0));

    // parse scheduled goal args
    Predicate start = null;
    Predicate period = null;
    Predicate end = null;
    for (int i = 1; i < goalState.size(); ++i) {
      Predicate arg = (Predicate) goalState.get(i);
      switch (arg.getName()) {
        case "start":
          start = arg;
          break;
        case "period":
          period = arg;
        case "end":
          end = arg;
        default:
          log.error("Invalid argument in scheduled goal predicate: {}", arg);
      }
    }

    // Error checks
    if (start == null && period == null) {
      log.error("Invalid scheduled goal predicate. Must define start or period. Goal: {}", goalState);
    }
    if (end != null && period == null) {
      log.error("Invalid scheduled goal predicate. End time only valid with periodic goals. Goal: {}", goalState);
    }

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

  @Override
  public Predicate getPredicate() {
    List<Symbol> args = new ArrayList<>();
    args.add(super.getPredicate()); // goal state
    if (start != null) {
      args.add(start);
    }
    if (period != null) {
      args.add(period);
    }
    if (end != null) {
      args.add(end);
    }

    return Factory.createPredicate("scheduled", args);
  }

  public Goal getNextGoalToSubmit() {
    Goal nextGoal = createGoal(super.getPredicate());
    submittedGoals.add(nextGoal);
    return nextGoal;
  }

  @Override
  public boolean isScheduled() {
    return true;
  }

  public boolean isPeriodic() {
    return scheduleInfo.period >= 0;
  }

  public long getScheduledDelay() {
    return scheduleInfo.initialDelay;
  }

  public long getScheduledPeriod() {
    return scheduleInfo.period;
  }

  public TimeUnit getScheduledTimeUnit() {
    return scheduleInfo.timeUnit;
  }

  public void updateStartTime(Predicate start) {

  }

  public void updatePeriod(Predicate period) {

  }

  public void updateEndTime(Predicate end) {

  }

  public static Date parseDateString(String dateString) {
    SimpleDateFormat formatter = new SimpleDateFormat("HH-mm-ss", Locale.ENGLISH);
    Date date = null;
    //TODO: what date format do we expect? Is only the LLM able to target this flexibly?
    try {
      date = formatter.parse(dateString);
      Calendar calendar = Calendar.getInstance();
      calendar.set(Calendar.HOUR_OF_DAY, date.getHours());
      calendar.set(Calendar.MINUTE, date.getMinutes());
      calendar.set(Calendar.SECOND, date.getSeconds());
      date = calendar.getTime();
    } catch (ParseException e) {
      // intentionally empty -- will try again with a different format
    }

    if (date == null) {
      try {
        formatter = new SimpleDateFormat("MM-dd-yyyy HH-mm-ss", Locale.ENGLISH);
        date = formatter.parse(dateString);
      } catch (ParseException e) {
        log.error("Error while parsing date arguments for scheduled goal. Date: {}", dateString, e);
      }
    }

    return date;
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
