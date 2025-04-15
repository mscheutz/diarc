package edu.tufts.hrilab.action.goal;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.*;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

public class ScheduledGoal extends Goal {

  private static final Logger log = LoggerFactory.getLogger(ScheduledGoal.class);
  private Predicate start;
  private Predicate period;
  private Predicate end;
  private final long instantiatedTime = System.currentTimeMillis();

  /**
   * Schedule information, extracted from the start, period, and end predicates.
   */
  private final ScheduleInfo scheduleInfo = new ScheduleInfo();

  public List<Goal> getSubmittedGoals() {
    return submittedGoals;
  }

  public ScheduleInfo getScheduleInfo() {
    return scheduleInfo;
  }

  private final List<Goal> submittedGoals = new ArrayList<>();

  /**
   * Scheduled goal form: scheduled(goalPred, startPred, periodPred, endPred)
   * - only goalPred and startPred OR periodPred are required
   * - no startTime means start right away
   * - endPred only valid if periodPred is specified
   * - startPred forms: start(number, timeunit), start(date-format)
   * - periodPred forms: period(number,timeunit), period(date-format)
   * - endPred forms: end(number, timeunit), end(date-format)
   *
   * @param goalState scheduled goal predicate: scheduled(goalPred, [startPred], [periodPred], [endPred])
   */
  protected ScheduledGoal(Predicate goalState) {
    super((Predicate) goalState.get(0));

    // parse scheduled goal args
    for (int i = 1; i < goalState.size(); ++i) {
      Predicate arg = (Predicate) goalState.get(i);
      update(arg);
    }

    // Error checks
    if (start == null && period == null) {
      log.error("Invalid scheduled goal predicate. Must define start or period. Goal: {}", goalState);
    }
    if (end != null && period == null) {
      log.error("Invalid scheduled goal predicate. End time only valid with periodic goals. Goal: {}", goalState);
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
    // if last goal is not terminated yet, return null so overlapping goals aren't submitted
    if (!submittedGoals.isEmpty()) {
      Goal lastGoal = submittedGoals.get(submittedGoals.size()-1);
      if (!lastGoal.getStatus().isTerminated()) {
        return null;
      }
    }

    Goal nextGoal = createGoal(super.getPredicate());
    submittedGoals.add(nextGoal);
    return nextGoal;
  }

  public Goal getLastSubmittedGoal() {
    return submittedGoals.get(submittedGoals.size()-1);
  }

  @Override
  public boolean isScheduled() {
    return true;
  }

  public boolean isPeriodic() {
    return scheduleInfo.period >= 0;
  }

  public long getScheduledPeriod() {
    return scheduleInfo.period;
  }

  public boolean hasStartDate() {
    return scheduleInfo.startTime != null;
  }

  public Date getStartDate() {
    return scheduleInfo.startTime;
  }

  public boolean hasEndDate() {
    return scheduleInfo.endTime != null;
  }

  public Date getEndDate() {
    return scheduleInfo.endTime;
  }

  public boolean update(Predicate update) {
    return switch (update.getName()) {
      case "and" -> {
        AtomicBoolean result = new AtomicBoolean(true);
        update.getArgs().forEach(u -> result.set(result.get() & update((Predicate) u)));
        yield result.get();
      }
      case "start" -> updateStartDate(update);
      case "period" -> updatePeriod(update);
      case "end" -> updateEndDate(update);
      default -> {
        log.error("Invalid scheduled goal predicate: {}", update);
        yield false;
      }
    };
  }

  private boolean updateStartDate(Predicate start) {
    if (start == null) {
      return false;
    }

    if (start.size() == 1) {
      // "start(date-string)"
      scheduleInfo.startTime = parseDateString(start.get(0).toString());
    } else if (start.size() == 2) {
      // "start(num, time-unit)"
      long delay = Long.parseLong(start.get(0).toString());
      long millis = TimeUnit.valueOf(start.get(1).toString().toUpperCase()).toMillis(delay);
      scheduleInfo.startTime = new Date(System.currentTimeMillis() + millis);
    } else {
      log.error("Invalid start date predicate: {}", start);
      return false;
    }

    this.start = start;
    return true;
  }

  private boolean updatePeriod(Predicate period) {
    if (period == null) {
      return false;
    }

    if (period.size() == 2) {
      // "period(num, time-unit)"
      long delay = Long.parseLong(period.get(0).toString());
      scheduleInfo.period = TimeUnit.valueOf(period.get(1).toString().toUpperCase()).toMillis(delay);
    } else {
      log.error("Invalid periodicity predicate: {}", period);
      return false;
    }

    this.period = period;
    return true;
  }

  private boolean updateEndDate(Predicate end) {
    if (end == null) {
      return false;
    }

    this.end = end;
    if (end.size() == 1) {
      // "end(date-string)"
      scheduleInfo.endTime = parseDateString(end.get(0).toString());
    } else if (start.size() == 2) {
      // "end(num, time-unit)"
      long delay = Long.parseLong(end.get(0).toString());
      long millis = TimeUnit.valueOf(end.get(1).toString().toUpperCase()).toMillis(delay);
      scheduleInfo.endTime = new Date(System.currentTimeMillis() + millis);
    } else {
      log.error("Invalid end date predicate: {}", end);
      return false;
    }
    return true;
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

  public Date getNextScheduledExecutionDate() {
    // TODO: implement this !!
    return new Date();
  }

}
