package edu.tufts.hrilab.action.goal;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.action.annotations.OnInterrupt;
import edu.tufts.hrilab.action.execution.ExecutionType;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.util.Util;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.Locale;

import static org.junit.Assert.*;

//**DISCLAIMER**: Reduced the timing windows in these tests after writing them, wary that different CPUs under different load
//  may not consistently pass all tests.
//Unfortunately, due to the nature of the functionality this is testing, we need to wait some amount of time within each
//  test. Tried to make the windows as small as possible while still being consistent
//TODO: best solution would be to extend submission/parsing to handle millisecond precision and use that to avoid the
//  need to work around potential 999ms delta based on submission time.
public class ScheduledGoalTest {
  private static GoalManagerComponent gm;
  private long testExecutedTime = -1;
  private static Logger log = LoggerFactory.getLogger(ScheduledGoalTest.class);
  private long sleepTime = 100;
  private boolean testCanceled = false;

  @BeforeClass
  public static void setUpClass() {
    gm = DiarcComponent.createInstance(GoalManagerComponent.class, "-beliefinitfile agents/agents.pl", true);
  }

  @AfterClass
  public static void destroy(){
    gm.shutdownComponent();
    gm = null;
  }

  @Before
  public void registerThis() {
    try {
      log.info("registerThis called.");
      TRADE.registerAllServices(this,new ArrayList<>());
      Util.Sleep(1000); // give time for actions to get into action DB
    } catch (TRADEException e) {
      log.error("Error registering with TRADE.", e);
    }
  }

  @After
  public void deregisterThis() {
    try {
      log.info("deregisterThis called.");
      TRADE.deregister(this);
    } catch (TRADEException e) {
      log.error("Error deregistering with TRADE.", e);
    }
    gm.cancelAllCurrentGoals();
  }

  private String getDateString(int delay, int timeUnit) {
    Calendar calendar = Calendar.getInstance();
    calendar.add(timeUnit, delay);
    Date date = calendar.getTime();
    SimpleDateFormat formatter = new SimpleDateFormat("HH-mm-ss", Locale.ENGLISH);
    return formatter.format(date);
  }

  /**
   * Test submission and execution of a scheduled goal.
   * Execution occurs at the specified Date.
   */
  @Test
  public void testScheduledExecutionByDate() {
    log.info("[testScheduledExecutionByDate]");
    String dateString = getDateString(2, Calendar.SECOND);
    long expectedExecutionTime = System.currentTimeMillis() + 2000;
    Predicate goalPred = Factory.createPredicate("scheduled(test(self:agent),start("+dateString+"))");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    assertEquals(GoalStatus.ACTIVE,gm.joinOnGoal(gid,1000));
    assertEquals(GoalStatus.SUCCEEDED,gm.joinOnGoal(gid,2000));
    Goal goal = gm.getGoal(gid);
    assertTrue(goal.isScheduled());
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
  }

  /**
   * Test submission and execution of a scheduled goal.
   * Execution occurs after the specified delay.
   */
  @Test
  public void testScheduledExecutionByDelay() {
    log.info("[testScheduledExecutionByDelay]");
    Predicate goalPred = Factory.createPredicate("scheduled(test(self:agent),start(2,SECONDS))");
    long expectedExecutionTime = System.currentTimeMillis() + 2000;
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    assertEquals(GoalStatus.ACTIVE,gm.joinOnGoal(gid,1000));
    assertEquals(GoalStatus.SUCCEEDED,gm.joinOnGoal(gid,2000));
    Goal goal = gm.getGoal(gid);
    assertTrue(goal.isScheduled());
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
  }

  /**
   * Tests submission and execution of a persistently scheduled goal.
   * Scheduled by Date. The first execution is at the supplied Date. The interval thereafter is 1 day.
   */
  @Test
  public void testPersistentlyScheduledExecutionByDate() throws InterruptedException {
    log.info("[testPersistentlyScheduledExecutionByDate]");
    Calendar calendar = Calendar.getInstance();
    calendar.add(Calendar.SECOND, 2);
    Date date = calendar.getTime();
    SimpleDateFormat formatter = new SimpleDateFormat("HH-mm-ss", Locale.ENGLISH);
    String dateString = formatter.format(date);
    String startPredStr = "start(" + dateString + ")";
    String periodPredStr = "period(1,DAYS)";
    Predicate goalPred = Factory.createPredicate("scheduled","test(self:agent)", startPredStr, periodPredStr);
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    long expectedExecutionTime = calendar.getTime().getTime();
    Goal goal = gm.getGoal(gid);
    assertTrue(goal.isScheduled());
    assertTrue(goal.isPersistent());
    Thread.sleep(2500);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    calendar.add(Calendar.DAY_OF_MONTH,1);
    assertEquals(goal.getNextScheduledExecutionDate().getTime(),calendar.getTime().getTime(), 1000);
  }

  /**
   * Tests submission and execution of a persistently scheduled goal.
   * Scheduled by delay. First execution is immediate. The interval is defined by a delay duration and timeunit.
   */
  @Test
  public void testPersistentlyScheduledExecutionByDelay() throws InterruptedException {
    log.info("[testPersistentlyScheduledExecutionByDelay]");
    Predicate goalPred = Factory.createPredicate("scheduled(test(self:agent),period(3,SECONDS))");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    long expectedExecutionTime = System.currentTimeMillis();
    Goal goal = gm.getGoal(gid);
    assertTrue(goal.isScheduled());
    assertTrue(goal.isPersistent());
    Thread.sleep(1000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    expectedExecutionTime += 3000;
    Thread.sleep(3000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
  }

  /**
   * Tests submission and execution of a persistently scheduled goal with a separately scheduled first execution.
   * First execution occurs after the outer delay and thereafter on an interval defined by the inner delay.
   */
  @Test
  public void testScheduledPersistentlyScheduledExecutionByDelays() throws InterruptedException {
    log.info("[testScheduledPersistentlyScheduledExecutionByDelays]");
    Predicate goalPred = Factory.createPredicate("scheduled(test(self:agent),start(2,SECONDS),period(3,SECONDS))");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    long expectedExecutionTime = System.currentTimeMillis() + 2000;
    Goal goal = gm.getGoal(gid);
    assertTrue(goal.isScheduled());
    assertTrue(goal.isPersistent());
    Thread.sleep(3000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    expectedExecutionTime += 3000;
    Thread.sleep(3000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    expectedExecutionTime += 3000;
    Thread.sleep(3000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
  }

  //Not sure if this case should be allowed to submit
  /**
   * Tests submission and execution of a persistently scheduled goal with a separately scheduled first execution.
   * First execution occurs at the specified outer date and thereafter every day at the time contained within the inner date.
   */
  @Test
  public void testScheduledPersistentlyScheduledExecutionByDates() throws InterruptedException {
    log.info("[testScheduledPersistentlyScheduledExecutionByDates]");
    Calendar calendar = Calendar.getInstance();
    calendar.add(Calendar.SECOND, 2);
    Date date = calendar.getTime();
    long expectedExecutionTime = date.getTime();
    SimpleDateFormat formatter = new SimpleDateFormat("HH-mm-ss", Locale.ENGLISH);
    String startPredStr = "start(" + formatter.format(date) + ")";
    calendar.add(Calendar.SECOND, 3);
    date = calendar.getTime();
    long expectedExecutionTime2 = date.getTime();
    String periodPredStr = "period(" +  formatter.format(date) + ")";
    Predicate goalPred = Factory.createPredicate("scheduled", "test(self:agent)", startPredStr, periodPredStr);
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    Goal goal = gm.getGoal(gid);
    assertTrue(goal.isScheduled());
    assertTrue(goal.isPersistent());
    Thread.sleep(2500);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    Thread.sleep(3000);
    assertEquals(expectedExecutionTime2, testExecutedTime, 999);
    calendar.add(Calendar.DAY_OF_MONTH,1);
    assertEquals(goal.getNextScheduledExecutionDate().getTime(),calendar.getTime().getTime(), 1000);
  }

  //Again, not sure if this submission makes much sense
  /**
   * Tests submission and execution of a persistently scheduled goal with a separately scheduled first execution.
   * First execution occurs after the outer delay and thereafter every day at the time contained within the inner date.
   */
  @Test
  public void testScheduledPersistentlyScheduledExecutionByDelayAndDate() throws InterruptedException {
    log.info("[testScheduledPersistentlyScheduledExecutionByDelayAndDate]");
    Calendar calendar = Calendar.getInstance();
    calendar.add(Calendar.SECOND, 3);
    Date date = calendar.getTime();
    long expectedExecutionTime2 = date.getTime();
    SimpleDateFormat formatter = new SimpleDateFormat("HH-mm-ss", Locale.ENGLISH);
    String periodPredStr = "period(" + formatter.format(date) + ")";
    Predicate goalPred = Factory.createPredicate("scheduled", "test(self:agent)", "start(1,SECONDS)", periodPredStr);
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    long expectedExecutionTime = System.currentTimeMillis() + 1000;
    Goal goal = gm.getGoal(gid);
    assertTrue(goal.isScheduled());
    assertTrue(goal.isPersistent());
    Thread.sleep(1500);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    Thread.sleep(2000);
    assertEquals(expectedExecutionTime2, testExecutedTime, 999);
    calendar.add(Calendar.DAY_OF_MONTH,1);
    assertEquals(goal.getNextScheduledExecutionDate().getTime(),calendar.getTime().getTime(), 1000);
  }

  /**
   * Tests submission and execution of a persistently scheduled goal with a separately scheduled first execution.
   * First execution occurs at the specified outer date and thereafter on an interval defined by the inner delay.
   */
  @Test
  public void testScheduledPersistentlyScheduledExecutionByDateAndDelay() throws InterruptedException {
    log.info("[testScheduledPersistentlyScheduledExecutionByDateAndDelay]");
    Calendar calendar = Calendar.getInstance();
    calendar.add(Calendar.SECOND, 2);
    Date date = calendar.getTime();
    long expectedExecutionTime = date.getTime();
    SimpleDateFormat formatter = new SimpleDateFormat("HH-mm-ss", Locale.ENGLISH);
    String startPredStr = "start(" + formatter.format(date) + ")";
    Predicate goalPred = Factory.createPredicate("scheduled", "test(self:agent)", startPredStr, "period(3,SECONDS)");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    Goal goal = gm.getGoal(gid);
    assertTrue(goal.isScheduled());
    assertTrue(goal.isPersistent());
    Thread.sleep(2500);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    expectedExecutionTime += 3000;
    Thread.sleep(3000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    expectedExecutionTime += 3000;
    Thread.sleep(3000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
  }

  /**
   * Tests modifying the scheduled time of a scheduled goal.
   * Update a scheduled date with another date.
   */
  @Test
  public void testScheduleModificationDate() throws InterruptedException {
    log.info("[testScheduleModificationDate]");
    Calendar calendar = Calendar.getInstance();
    calendar.add(Calendar.SECOND, 2);
    Date date = calendar.getTime();
    SimpleDateFormat formatter = new SimpleDateFormat("HH-mm-ss", Locale.ENGLISH);
    String dateString = formatter.format(date);
    Predicate goalPred = Factory.createPredicate("scheduled", "test(self:agent)", "start("+dateString+")");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    calendar.add(Calendar.SECOND, 2);
    Thread.sleep(300);
    date = calendar.getTime();
    dateString = formatter.format(date);
    long expectedExecutionTime = date.getTime();
    gm.updateScheduledGoal(gid,Factory.createPredicate("start", dateString));
    assertEquals(GoalStatus.ACTIVE,gm.joinOnGoal(gid,2000));
    assertEquals(GoalStatus.SUCCEEDED,gm.joinOnGoal(gid,3000));
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
  }

  /**
   * Tests modifying the scheduled time of a scheduled goal.
   * Update a delay with another delay.
   */
  @Test
  public void testScheduleModificationDelay() throws InterruptedException {
    log.info("[testScheduleModificationDelay]");
    Predicate goalPred = Factory.createPredicate("scheduled","test(self:agent)","start(10,SECONDS)");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    Thread.sleep(500);
    gm.updateScheduledGoal(gid,Factory.createPredicate("start(2,SECONDS)"));
    long expectedExecutionTime = System.currentTimeMillis() + 2000;
    assertEquals(GoalStatus.SUCCEEDED,gm.joinOnGoal(gid,3000));
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
  }

  /**
   * Tests modifying the scheduled time of a scheduled goal.
   * Replace a scheduled date with a delay.
   */
  @Test
  public void testScheduleModificationDateToDelay() throws InterruptedException {
    log.info("[testScheduleModificationDateToDelay]");
    Calendar calendar = Calendar.getInstance();
    calendar.add(Calendar.SECOND, 2);
    Date date = calendar.getTime();
    SimpleDateFormat formatter = new SimpleDateFormat("HH-mm-ss", Locale.ENGLISH);
    String dateString = formatter.format(date);
    Predicate goalPred = Factory.createPredicate("scheduled", "test(self:agent)","start("+dateString+")");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    Thread.sleep(500);
    gm.updateScheduledGoal(gid, Factory.createPredicate("start(3, SECONDS)"));
    long expectedExecutionTime = System.currentTimeMillis() + 3000;
    assertEquals(GoalStatus.SUCCEEDED,gm.joinOnGoal(gid,4000));
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
  }

  /**
   * Tests modifying the scheduled time of a scheduled goal.
   * Replace a delay with a date.
   */
  @Test
  public void testScheduleModificationDelayToDate() throws InterruptedException {
    log.info("[testScheduleModificationDelayToDate]");
    Predicate goalPred = Factory.createPredicate("scheduled","test(self:agent)", "start(10,SECONDS)");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    Thread.sleep(1000);
    Calendar calendar = Calendar.getInstance();
    calendar.add(Calendar.SECOND, 2);
    Date date = calendar.getTime();
    SimpleDateFormat formatter = new SimpleDateFormat("HH-mm-ss", Locale.ENGLISH);
    String dateString = formatter.format(date);
    long expectedExecutionTime = date.getTime();
    gm.updateScheduledGoal(gid,Factory.createPredicate("start", dateString));
    assertEquals(GoalStatus.ACTIVE,gm.joinOnGoal(gid,1000));
    assertEquals(GoalStatus.SUCCEEDED,gm.joinOnGoal(gid,2000));
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
  }

  //TODO: Only one of this or modifying the 'next execution' should actually be reached when desiring this functionality.
  /**
   * Tests modifying the scheduled interval of a persistently scheduled goal.
   * Replace a scheduled date with a different date. The goal will execute every day at the time contained within the new date.
   */
  @Test
  public void testPersistentlyScheduledIntervalModificationDate() throws InterruptedException {
    log.info("[testPersistentlyScheduledIntervalModificationDate]");
    Calendar calendar = Calendar.getInstance();
    calendar.add(Calendar.SECOND, 2);
    Date date = calendar.getTime();
    SimpleDateFormat formatter = new SimpleDateFormat("HH-mm-ss", Locale.ENGLISH);
    String dateString = formatter.format(date);
    Predicate goalPred = Factory.createPredicate("scheduled","test(self:agent)", "period(" + dateString+")");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    long expectedExecutionTime = calendar.getTime().getTime();
    Goal goal = gm.getGoal(gid);
    assertTrue(goal.isScheduled());
    assertTrue(goal.isPersistent());
    Thread.sleep(2500);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    calendar.add(Calendar.HOUR, 2);
    date = calendar.getTime();
    dateString = formatter.format(date);
    gm.updateScheduledGoal(gid,Factory.createPredicate("period", dateString));
    assertEquals(goal.getNextScheduledExecutionDate().getTime(),calendar.getTime().getTime(), 1000);
  }

  /**
   * Tests modifying the scheduled interval of a persistently scheduled goal.
   * Replace a scheduled date with a delay. The goal will execute on the new interval relative to the last execution time
   */
  @Test
  public void testPersistentlyScheduledIntervalModificationDateToDelay() throws InterruptedException {
    log.info("[testPersistentlyScheduledIntervalModificationDateToDelay]");
    Calendar calendar = Calendar.getInstance();
    calendar.add(Calendar.SECOND, 2);
    Date date = calendar.getTime();
    SimpleDateFormat formatter = new SimpleDateFormat("HH-mm-ss", Locale.ENGLISH);
    String dateString = formatter.format(date);
    Predicate goalPred = Factory.createPredicate("scheduled","test(self:agent)", "period(" + dateString + ")");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    long expectedExecutionTime = calendar.getTime().getTime();
    Goal goal = gm.getGoal(gid);
    assertTrue(goal.isScheduled());
    assertTrue(goal.isPersistent());
    Thread.sleep(2500);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    gm.updateScheduledGoal(gid,Factory.createPredicate("period(2,SECONDS)"));
    expectedExecutionTime += 2000;
    Thread.sleep(2500);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    expectedExecutionTime += 2000;
    Thread.sleep(2000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
  }

  /**
   * Tests modifying the scheduled interval of a persistently scheduled goal.
   * Replace a scheduled date with a delay. The goal will execute on the new interval relative to the last execution time.
   * If more time has passed since the last execution than the specified interval, execute at the next aligning time. NOT
   * right away.
   */
  @Test
  public void testPersistentlyScheduledIntervalModificationDateToDelayV2() throws InterruptedException {
    log.info("[testPersistentlyScheduledIntervalModificationDateToDelayV2]");
    Calendar calendar = Calendar.getInstance();
    calendar.add(Calendar.SECOND, 1);
    Date date = calendar.getTime();
    SimpleDateFormat formatter = new SimpleDateFormat("HH-mm-ss", Locale.ENGLISH);
    String dateString = formatter.format(date);
    Predicate goalPred = Factory.createPredicate("scheduled", "test(self:agent)", "period(" + dateString + ")");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    long expectedExecutionTime = calendar.getTime().getTime();
    Goal goal = gm.getGoal(gid);
    assertTrue(goal.isScheduled());
    assertTrue(goal.isPersistent());
    Thread.sleep(3500);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    gm.updateScheduledGoal(gid,Factory.createPredicate("period(2,SECONDS)"));
    expectedExecutionTime += 4000;
    Thread.sleep(3000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    expectedExecutionTime += 2000;
    Thread.sleep(2000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
  }

  /**
   * Tests modifying the scheduled interval of a persistently scheduled goal.
   * Update a scheduled delay. The goal will execute on the new interval relative to the last execution time.
   */
  @Test
  public void testPersistentlyScheduledIntervalModificationDelay() throws InterruptedException {
    log.info("[testPersistentlyScheduledIntervalModificationDelay]");
    Predicate goalPred = Factory.createPredicate("scheduled", "test(self:agent)","period(3,SECONDS)");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    long expectedExecutionTime = System.currentTimeMillis();
    Goal goal = gm.getGoal(gid);
    assertTrue(goal.isScheduled());
    assertTrue(goal.isPersistent());
    Thread.sleep(1000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    expectedExecutionTime += 3000;
    Thread.sleep(3000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);

    gm.updateScheduledGoal(gid,Factory.createPredicate("period(2,SECONDS)"));
    expectedExecutionTime += 2000;
    Thread.sleep(1500);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    expectedExecutionTime += 2000;
    Thread.sleep(2500);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
  }

  /**
   * Tests modifying the scheduled interval of a persistently scheduled goal.
   * Update a scheduled delay. The goal will execute on the new interval relative to the last execution time.
   * If more time has passed since the last execution than the specified interval, execute at the next aligning time. NOT
   * right away.
   */
  @Test
  public void testPersistentlyScheduledIntervalModificationDelayV2() throws InterruptedException {
    log.info("[testPersistentlyScheduledIntervalModificationDelayV2]");
    Predicate goalPred = Factory.createPredicate("scheduled","test(self:agent)", "period(3,SECONDS)");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    long expectedExecutionTime = System.currentTimeMillis();
    Goal goal = gm.getGoal(gid);
    assertTrue(goal.isScheduled());
    assertTrue(goal.isPersistent());
    Thread.sleep(1000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    expectedExecutionTime += 3000;
    Thread.sleep(5000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);

    gm.updateScheduledGoal(gid,Factory.createPredicate("period(2,SECONDS)"));
    expectedExecutionTime += 4000;
    Thread.sleep(2000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
  }

  //Only one of this or "next execution" modification actually happens
  /**
   * Tests modifying the scheduled interval of a persistently scheduled goal.
   * Replace a scheduled delay with a date. The goal will execute at the specified Date and every day thereafter at the contained time
   */
  @Test
  public void testPersistentlyScheduledIntervalModificationDelayToDate() throws InterruptedException {
    log.info("[testPersistentlyScheduledIntervalModificationDelayToDate]");
    Predicate goalPred = Factory.createPredicate("scheduled", "test(self:agent)", "period(2,SECONDS)");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    long expectedExecutionTime = System.currentTimeMillis();
    Goal goal = gm.getGoal(gid);
    assertTrue(goal.isScheduled());
    assertTrue(goal.isPersistent());
    Thread.sleep(500);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    expectedExecutionTime += 2000;
    Thread.sleep(2000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);

    Calendar calendar = Calendar.getInstance();
    calendar.add(Calendar.SECOND, 1);
    Date date = calendar.getTime();
    SimpleDateFormat formatter = new SimpleDateFormat("HH-mm-ss", Locale.ENGLISH);
    String dateString = formatter.format(date);
    gm.updateScheduledGoal(gid,Factory.createPredicate("period", dateString));
    expectedExecutionTime = calendar.getTime().getTime();
    Thread.sleep(1500);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    calendar.add(Calendar.DAY_OF_MONTH,1);
    assertEquals(goal.getNextScheduledExecutionDate().getTime(),calendar.getTime().getTime(), 1000);
  }

  //Only one of this or "interval" modification actually happens
  /**
   * Tests modifying the next execution of a persistently scheduled goal (essentially 'shifting' the execution window).
   * Shift to a date. The goal will execute at the specified Date and every day thereafter at the contained time
   */
  @Test
  public void testPersistentlyScheduledExecutionModificationDateByDate() throws InterruptedException {
    log.info("[testPersistentlyScheduledExecutionModificationDateByDate]");
    Calendar calendar = Calendar.getInstance();
    calendar.add(Calendar.SECOND, 1);
    Date date = calendar.getTime();
    SimpleDateFormat formatter = new SimpleDateFormat("HH-mm-ss", Locale.ENGLISH);
    String dateString = formatter.format(date);
    Predicate goalPred = Factory.createPredicate("scheduled", "test(self:agent)", "start(" + dateString + ")", "period(1,DAYS)");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    long expectedExecutionTime = calendar.getTime().getTime();
    Goal goal = gm.getGoal(gid);
    assertTrue(goal.isScheduled());
    assertTrue(goal.isPersistent());
    Thread.sleep(1500);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    calendar.add(Calendar.HOUR, 2);
    date = calendar.getTime();
    dateString = formatter.format(date);
    gm.updateScheduledGoal(gid,Factory.createPredicate("start", dateString));
    assertEquals(goal.getNextScheduledExecutionDate().getTime(),calendar.getTime().getTime(), 1000);
  }

  //TODO: don't think this method should ever be hit in this case. If it does, what should the behavior be?
  @Ignore
  @Test
  public void testPersistentlyScheduledExecutionModificationDateByDelay() throws InterruptedException {
    log.info("[testPersistentlyScheduledExecutionModificationDateByDelay]");
    Calendar calendar = Calendar.getInstance();
    calendar.add(Calendar.SECOND, 1);
    Date date = calendar.getTime();
    SimpleDateFormat formatter = new SimpleDateFormat("HH-mm-ss", Locale.ENGLISH);
    String dateString = formatter.format(date);
    Predicate goalPred = Factory.createPredicate("persistent(self:agent,scheduled(self:agent,test(self:agent),"+dateString+"))");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    long expectedExecutionTime = calendar.getTime().getTime();
    Goal goal = gm.getGoal(gid);
    assertTrue(goal.isScheduled());
    assertTrue(goal.isPersistent());
    Thread.sleep(1500);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    gm.updateScheduledGoal(gid,Factory.createPredicate("start","2","SECONDS"));
    expectedExecutionTime = System.currentTimeMillis() + 2000;
    Thread.sleep(2500);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    calendar.add(Calendar.SECOND,3);
    calendar.add(Calendar.DAY_OF_MONTH,1);
    assertEquals(goal.getNextScheduledExecutionDate().getTime(),calendar.getTime().getTime(), 1000);
  }

  /**
   * Tests modifying the next execution of a persistently scheduled goal (essentially 'shifting' the execution window).
   * Shift by a delay. The goal will next execute after the newly specified delay, then continue on the original interval
   */
  @Test
  public void testPersistentlyScheduledExecutionModificationDelayByDelay() throws InterruptedException {
    log.info("[testPersistentlyScheduledExecutionModificationDelayByDelay]");
    Predicate goalPred = Factory.createPredicate("persistent(self:agent,scheduled(self:agent,test(self:agent),3,SECONDS))");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    long expectedExecutionTime = System.currentTimeMillis();
    Goal goal = gm.getGoal(gid);
    assertTrue(goal.isScheduled());
    assertTrue(goal.isPersistent());
    Thread.sleep(1000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    expectedExecutionTime += 3000;
    Thread.sleep(3000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);

    gm.updateScheduledGoal(gid,Factory.createPredicate("period","2", "SECONDS"));
    expectedExecutionTime = System.currentTimeMillis() + 2000;
    Thread.sleep(2500);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    expectedExecutionTime += 3000;
    Thread.sleep(3500);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
  }

  /**
   * Tests modifying the next execution of a persistently scheduled goal (essentially 'shifting' the execution window).
   * Shift to a date. The goal will next execute at the specified date, then continue on the original interval
   */
  @Test
  public void testPersistentlyScheduledExecutionModificationDelayByDate() throws InterruptedException {
    log.info("[testPersistentlyScheduledExecutionModificationDelayByDate]");
    Predicate goalPred = Factory.createPredicate("persistent(self:agent,scheduled(self:agent,test(self:agent),2,SECONDS))");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    long expectedExecutionTime = System.currentTimeMillis();
    Goal goal = gm.getGoal(gid);
    assertTrue(goal.isScheduled());
    assertTrue(goal.isPersistent());
    Thread.sleep(500);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    expectedExecutionTime += 2000;
    Thread.sleep(2000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);

    Calendar calendar = Calendar.getInstance();
    calendar.add(Calendar.SECOND, 3);
    Date date = calendar.getTime();
    SimpleDateFormat formatter = new SimpleDateFormat("HH-mm-ss", Locale.ENGLISH);
    String dateString = formatter.format(date);
    gm.updateScheduledGoal(gid,Factory.createPredicate("start", dateString));
    expectedExecutionTime = calendar.getTime().getTime();
    Thread.sleep(3500);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    expectedExecutionTime += 2000;
    Thread.sleep(2000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
  }

  //TODO: write additional test cases for this or assume SPS -> PS works the same way once the initial S has occurred?

  /**
   * Tests modifying the first execution of a scheduled persistently scheduled goal.
   * Update first execution to occur after a new delay.
   */
  @Test
  public void testScheduledPersistentlyScheduledFirstExecutionModificationDelay() throws InterruptedException {
    log.info("[testScheduledPersistentlyScheduledFirstExecutionModificationDelay]");
    Predicate goalPred = Factory.createPredicate("scheduled(self:agent,persistent(self:agent,scheduled(self:agent,test(self:agent),2,SECONDS)),5,SECONDS)");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    Thread.sleep(500);
    gm.updateScheduledGoal(gid,Factory.createPredicate("period",  "3","SECONDS"));
    long expectedExecutionTime = System.currentTimeMillis() + 3000;
    Goal goal = gm.getGoal(gid);
    assertTrue(goal.isScheduled());
    assertTrue(goal.isPersistent());
    Thread.sleep(3500);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    expectedExecutionTime += 2000;
    Thread.sleep(2000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    expectedExecutionTime += 2000;
    Thread.sleep(2000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
  }

  /**
   * Tests modifying the first execution of a scheduled persistently scheduled goal.
   * Replace first execution delay with a scheduled date.
   */
  @Test
  public void testScheduledPersistentlyScheduledFirstExecutionModificationDelayToDate() throws InterruptedException {
    log.info("[testScheduledPersistentlyScheduledFirstExecutionModificationDelayToDate]");
    Predicate goalPred = Factory.createPredicate("scheduled(self:agent,persistent(self:agent,scheduled(self:agent,test(self:agent),2,SECONDS)),5,SECONDS)");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    Thread.sleep(500);
    Calendar calendar = Calendar.getInstance();
    calendar.add(Calendar.SECOND, 2);
    Date date = calendar.getTime();
    long expectedExecutionTime = date.getTime();
    SimpleDateFormat formatter = new SimpleDateFormat("HH-mm-ss", Locale.ENGLISH);
    String dateString = formatter.format(date);
    gm.updateScheduledGoal(gid,Factory.createPredicate("start", dateString));
    Goal goal = gm.getGoal(gid);
    assertTrue(goal.isScheduled());
    assertTrue(goal.isPersistent());
    Thread.sleep(2500);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    expectedExecutionTime += 2000;
    Thread.sleep(2000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    expectedExecutionTime += 2000;
    Thread.sleep(2000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
  }

  //Should this case ever be reached either?
  /**
   * Tests modifying the first execution of a scheduled persistently scheduled goal.
   * Update the first execution date.
   */
  @Test
  public void testScheduledPersistentlyScheduledFirstExecutionModificationDate() throws InterruptedException {
    log.info("[testScheduledPersistentlyScheduledFirstExecutionModificationDate]");
    Calendar calendar = Calendar.getInstance();
    calendar.add(Calendar.SECOND, 2);
    Date date = calendar.getTime();
    SimpleDateFormat formatter = new SimpleDateFormat("HH-mm-ss", Locale.ENGLISH);
    String dateString = formatter.format(date);
    calendar.add(Calendar.SECOND, 2);
    date = calendar.getTime();
    long expectedExecutionTime = date.getTime();
    String dateString1 = formatter.format(date);
    calendar.add(Calendar.SECOND, 3);
    date = calendar.getTime();
    long expectedExecutionTime2 = date.getTime();
    String dateString2 = formatter.format(date);
    Predicate goalPred = Factory.createPredicate("scheduled(self:agent,persistent(self:agent,scheduled(self:agent,test(self:agent),"+dateString2+")),"+dateString+")");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    Thread.sleep(500);
    gm.updateScheduledGoal(gid,Factory.createPredicate("start", dateString1));
    Goal goal = gm.getGoal(gid);
    assertTrue(goal.isScheduled());
    assertTrue(goal.isPersistent());
    Thread.sleep(4500);
    assertEquals(expectedExecutionTime, testExecutedTime, 1000);
    Thread.sleep(3000);
    assertEquals(expectedExecutionTime2, testExecutedTime, 1000);
    calendar.add(Calendar.DAY_OF_MONTH,1);
    assertEquals(goal.getNextScheduledExecutionDate().getTime(),calendar.getTime().getTime(), 1000);
  }

  /**
   * Tests modifying the first execution of a scheduled persistently scheduled goal.
   * Replace original first execution date with a delay.
   */
  @Test
  public void testScheduledPersistentlyScheduledFirstExecutionModificationDateToDelay() throws InterruptedException {
    log.info("[testScheduledPersistentlyScheduledFirstExecutionModificationDateToDelay]");
    Calendar calendar = Calendar.getInstance();
    calendar.add(Calendar.SECOND, 2);
    Date date = calendar.getTime();
    SimpleDateFormat formatter = new SimpleDateFormat("HH-mm-ss", Locale.ENGLISH);
    String dateString = formatter.format(date);
    calendar.add(Calendar.SECOND, 4);
    date = calendar.getTime();
    long expectedExecutionTime2 = date.getTime();
    String dateString2 = formatter.format(date);
    Predicate goalPred = Factory.createPredicate("scheduled(self:agent,persistent(self:agent,scheduled(self:agent,test(self:agent),"+dateString2+")),"+dateString+")");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    Thread.sleep(500);
    gm.updateScheduledGoal(gid,Factory.createPredicate("start",  "3","SECONDS"));
    long expectedExecutionTime = System.currentTimeMillis() + 3000;
    Goal goal = gm.getGoal(gid);
    assertTrue(goal.isScheduled());
    assertTrue(goal.isPersistent());
    Thread.sleep(3500);
    assertEquals(expectedExecutionTime, testExecutedTime, 1000);
    Thread.sleep(3000);
    assertEquals(expectedExecutionTime2, testExecutedTime, 1000);
    calendar.add(Calendar.DAY_OF_MONTH,1);
    assertEquals(goal.getNextScheduledExecutionDate().getTime(),calendar.getTime().getTime(), 1000);
  }

  /**
   * Tests modifying the interval of a inner persistently scheduled goal within a scheduled persistently scheduled goal.
   * Update the interval delay.
   */
  @Test
  public void testScheduledPersistentlyScheduledIntervalModificationDelay() throws InterruptedException {
    log.info("[testScheduledPersistentlyScheduledIntervalModificationDelay]");
    Predicate goalPred = Factory.createPredicate("scheduled(self:agent,persistent(self:agent,scheduled(self:agent,test(self:agent),3,SECONDS)),2,SECONDS)");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    long expectedExecutionTime = System.currentTimeMillis() + 2000;
    Thread.sleep(500);
    gm.updateScheduledGoal(gid,Factory.createPredicate("period","2","SECONDS"));
    Goal goal = gm.getGoal(gid);
    assertTrue(goal.isScheduled());
    assertTrue(goal.isPersistent());
    Thread.sleep(2000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    expectedExecutionTime += 2000;
    Thread.sleep(2000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    expectedExecutionTime += 2000;
    Thread.sleep(2000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
  }

  /**
   * Tests modifying the interval of an inner persistently scheduled goal within a scheduled persistently scheduled goal.
   * Replace the interval delay with a scheduled date
   */
  @Test
  public void testScheduledPersistentlyScheduledIntervalModificationDelayToDate() throws InterruptedException {
    log.info("[testScheduledPersistentlyScheduledIntervalModificationDelayToDate]");
    Predicate goalPred = Factory.createPredicate("scheduled(self:agent,persistent(self:agent,scheduled(self:agent,test(self:agent),3,SECONDS)),2,SECONDS)");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    long expectedExecutionTime = System.currentTimeMillis() + 2000;
    Thread.sleep(500);
    Calendar calendar = Calendar.getInstance();
    calendar.add(Calendar.SECOND, 3);
    Date date = calendar.getTime();
    SimpleDateFormat formatter = new SimpleDateFormat("HH-mm-ss", Locale.ENGLISH);
    String dateString = formatter.format(date);
    gm.updateScheduledGoal(gid,Factory.createPredicate("period", dateString));
    Goal goal = gm.getGoal(gid);
    assertTrue(goal.isScheduled());
    assertTrue(goal.isPersistent());
    Thread.sleep(2000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    expectedExecutionTime = date.getTime();
    Thread.sleep(3000);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    calendar.add(Calendar.DAY_OF_MONTH,1);
    assertEquals(goal.getNextScheduledExecutionDate().getTime(),calendar.getTime().getTime(), 1000);
  }

  /**
   * Tests modifying the interval of an inner persistently scheduled goal within a scheduled persistently scheduled goal.
   * Update the persistently scheduled date
   */
  @Test
  public void testScheduledPersistentlyScheduledIntervalModificationDate() throws InterruptedException {
    log.info("[testScheduledPersistentlyScheduledIntervalModificationDate]");
    Calendar calendar = Calendar.getInstance();
    calendar.add(Calendar.SECOND, 2);
    Date date = calendar.getTime();
    long expectedExecutionTime = date.getTime();
    SimpleDateFormat formatter = new SimpleDateFormat("HH-mm-ss", Locale.ENGLISH);
    String dateString = formatter.format(date);
    calendar.add(Calendar.SECOND, 2);
    date = calendar.getTime();
    long expectedExecutionTime2 = date.getTime();
    String dateString1 = formatter.format(date);
    calendar.add(Calendar.DAY_OF_MONTH,1);
    Date expectedNextExecutionDate = calendar.getTime();
    calendar.add(Calendar.SECOND, 4);
    date = calendar.getTime();
    String dateString2 = formatter.format(date);
    Predicate goalPred = Factory.createPredicate("scheduled(self:agent,persistent(self:agent,scheduled(self:agent,test(self:agent),"+dateString2+")),"+dateString+")");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    Thread.sleep(500);
    gm.updateScheduledGoal(gid,Factory.createPredicate("period", dateString1));
    Goal goal = gm.getGoal(gid);
    assertTrue(goal.isScheduled());
    assertTrue(goal.isPersistent());
    Thread.sleep(2000);
    assertEquals(expectedExecutionTime, testExecutedTime, 1000);
    Thread.sleep(3000);
    assertEquals(expectedExecutionTime2, testExecutedTime, 1000);
    assertEquals(goal.getNextScheduledExecutionDate().getTime(),expectedNextExecutionDate.getTime(), 1000);
  }

  /**
   * Tests modifying the interval of an inner persistently scheduled goal within a scheduled persistently scheduled goal.
   * Replace the persistently scheduled date with a delay
   */
  @Test
  public void testScheduledPersistentlyScheduledIntervalModificationDateToDelay() throws InterruptedException {
    log.info("[testScheduledPersistentlyScheduledIntervalModificationDateToDelay]");
    Calendar calendar = Calendar.getInstance();
    calendar.add(Calendar.SECOND, 3);
    Date date = calendar.getTime();
    long expectedExecutionTime = date.getTime();
    SimpleDateFormat formatter = new SimpleDateFormat("HH-mm-ss", Locale.ENGLISH);
    String dateString = formatter.format(date);
    calendar.add(Calendar.SECOND, 7);
    date = calendar.getTime();
    String dateString2 = formatter.format(date);
    Predicate goalPred = Factory.createPredicate("scheduled(self:agent,persistent(self:agent,scheduled(self:agent,test(self:agent),"+dateString2+")),"+dateString+")");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    Thread.sleep(500);
    gm.updateScheduledGoal(gid,Factory.createPredicate("period","2","SECONDS"));
    Goal goal = gm.getGoal(gid);
    assertTrue(goal.isScheduled());
    assertTrue(goal.isPersistent());
    Thread.sleep(3000);
    assertEquals(expectedExecutionTime, testExecutedTime, 1000);
    expectedExecutionTime += 2000;
    Thread.sleep(2000);
    assertEquals(expectedExecutionTime, testExecutedTime, 1000);
    expectedExecutionTime += 2000;
    Thread.sleep(2000);
    assertEquals(expectedExecutionTime, testExecutedTime, 1000);
  }

  /**
   * Tests recovering the original predicate a scheduled goal was submitted with after processing and stripping of details
   * Scheduled by date
   */
  @Test
  public void testGetWrappedSubmissionPredScheduled() throws InterruptedException {
    log.info("[testGetWrappedSubmissionPredScheduled]");
    String dateString = getDateString(3, Calendar.SECOND);
    Predicate goalPred = Factory.createPredicate("scheduled(self:agent,test(self:agent),"+dateString+")");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    Thread.sleep(500);
    assertEquals(gm.getWrappedScheduledGoalSubmissionPredicate(gid),goalPred);
  }

  /**
   * Tests recovering the original predicate a scheduled goal was submitted with after processing and stripping of details.
   * Scheduled by delay
   */
  @Test
  public void testGetWrappedSubmissionPredScheduledDelay() throws InterruptedException {
    log.info("[testGetWrappedSubmissionPredScheduledDelay]");
    Predicate goalPred = Factory.createPredicate("scheduled(self:agent,test(self:agent),3,SECONDS)");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    Thread.sleep(500);
    assertEquals(gm.getWrappedScheduledGoalSubmissionPredicate(gid).toString(),goalPred.toString());
  }

  /**
   * Tests recovering the original predicate a persistently scheduled goal was submitted with after processing and stripping of details.
   * Scheduled by delay
   */
  @Test
  public void testGetWrappedSubmissionPredPersistentlyScheduled() throws InterruptedException {
    log.info("[testGetWrappedSubmissionPredPersistentlyScheduled]");
    Predicate goalPred = Factory.createPredicate("persistent(self:agent,scheduled(self:agent,test(self:agent),3,SECONDS))");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    Thread.sleep(500);
    assertEquals(gm.getWrappedScheduledGoalSubmissionPredicate(gid).toString(), goalPred.toString());
  }

  /**
   * Tests recovering the original predicate a scheduled persistently scheduled goal was submitted with after processing and stripping of details.
   * Scheduled by date and delay
   */
  @Test
  public void testGetWrappedSubmissionPredScheduledPersistentlyScheduled() throws InterruptedException {
    log.info("[testGetWrappedSubmissionPredScheduledPersistentlyScheduled]");
    Calendar calendar = Calendar.getInstance();
    calendar.add(Calendar.SECOND, 5);
    Date date = calendar.getTime();
    SimpleDateFormat formatter = new SimpleDateFormat("HH-mm-ss", Locale.ENGLISH);
    String dateString = formatter.format(date);
    Predicate goalPred = Factory.createPredicate("scheduled(self:agent,persistent(self:agent,scheduled(self:agent,test(self:agent),"+dateString+")),1,SECONDS)");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    Thread.sleep(500);
    assertEquals(gm.getWrappedScheduledGoalSubmissionPredicate(gid).toString(), goalPred.toString());
    gm.joinOnGoal(gid,1000);
    assertEquals(gm.getWrappedScheduledGoalSubmissionPredicate(gid).toString(), goalPred.toString());
  }

  /**
   * Tests generating a wrapped predicate representing the current scheduled/persistent state of a scheduled goal.
   * Scheduled by date
   */
  @Test
  public void testGetWrappedPredScheduled() throws InterruptedException {
    log.info("[testGetWrappedPredScheduled]");
    String dateString = getDateString(3, Calendar.SECOND);
    Predicate goalPred = Factory.createPredicate("scheduled(self:agent,test(self:agent),"+dateString+")");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    Thread.sleep(500);
    assertEquals(gm.getWrappedScheduledGoalPredicate(gid),goalPred);
  }

  /**
   * Tests generating a wrapped predicate representing the current scheduled/persistent state of a scheduled goal.
   * Scheduled by delay
   */
  @Test
  public void testGetWrappedPredScheduledDelay() throws InterruptedException, ParseException {
    log.info("[testGetWrappedPredScheduledDelay]");
    Predicate goalPred = Factory.createPredicate("scheduled(self:agent,test(self:agent),5,SECONDS)");
    long expectedTime = System.currentTimeMillis() + 5000;
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    Thread.sleep(1500);
    SimpleDateFormat formatter = new SimpleDateFormat("HH-mm-ss", Locale.ENGLISH);
    String dateString = gm.getWrappedScheduledGoalPredicate(gid).get(2).toString();
    Date date = formatter.parse(dateString);
    Calendar calendar = Calendar.getInstance();
    calendar.set(Calendar.HOUR_OF_DAY,date.getHours());
    calendar.set(Calendar.MINUTE,date.getMinutes());
    calendar.set(Calendar.SECOND,date.getSeconds());
    date = calendar.getTime();
    assertEquals(date.getTime(),expectedTime,1000);
  }

  /**
   * Tests generating a wrapped predicate representing the current scheduled/persistent state of a persistently scheduled goal.
   * Scheduled by delay
   */
  @Test
  public void testGetWrappedPredPersistentlyScheduled() throws InterruptedException, ParseException {
    log.info("[testGetWrappedPredPersistentlyScheduled]");
    Predicate goalPred = Factory.createPredicate("persistent(self:agent,scheduled(self:agent,test(self:agent),5,SECONDS))");
    long expectedTime = System.currentTimeMillis() + 5000;
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    Thread.sleep(1500);
    assertEquals(gm.getWrappedScheduledGoalPredicate(gid).get(1).toString(), goalPred.toString());
    SimpleDateFormat formatter = new SimpleDateFormat("HH-mm-ss", Locale.ENGLISH);
    String dateString = gm.getWrappedScheduledGoalPredicate(gid).get(2).toString();
    Date date = formatter.parse(dateString);
    Calendar calendar = Calendar.getInstance();
    calendar.set(Calendar.HOUR_OF_DAY,date.getHours());
    calendar.set(Calendar.MINUTE,date.getMinutes());
    calendar.set(Calendar.SECOND,date.getSeconds());
    date = calendar.getTime();
    assertEquals(date.getTime(),expectedTime,1000);
  }

  /**
   * Tests generating a wrapped predicate representing the current scheduled/persistent state of a scheduled persistently scheduled goal.
   * Scheduled after a delay every date
   */
  @Test
  public void testGetWrappedPredScheduledPersistentlyScheduled() throws InterruptedException, ParseException {
    log.info("[testGetWrappedPredScheduledPersistentlyScheduled]");
    Calendar calendar = Calendar.getInstance();
    calendar.add(Calendar.SECOND, 11);
    Date date = calendar.getTime();
    SimpleDateFormat formatter = new SimpleDateFormat("HH-mm-ss", Locale.ENGLISH);
    String dateString = formatter.format(date);
    Predicate goalPred = Factory.createPredicate("scheduled(self:agent,persistent(self:agent,scheduled(self:agent,test(self:agent),"+dateString+")),6,SECONDS)");
    long expectedTime = System.currentTimeMillis() + 6000;
    long expectedTime2 = System.currentTimeMillis() + 12000;
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    Thread.sleep(2000);
    assertEquals(gm.getWrappedScheduledGoalPredicate(gid).get(1).toString(), goalPred.get(1).toString());
    dateString = gm.getWrappedScheduledGoalPredicate(gid).get(2).toString();
    date = formatter.parse(dateString);
    calendar = Calendar.getInstance();
    calendar.set(Calendar.HOUR_OF_DAY,date.getHours());
    calendar.set(Calendar.MINUTE,date.getMinutes());
    calendar.set(Calendar.SECOND,date.getSeconds());
    date = calendar.getTime();
    assertEquals(date.getTime(),expectedTime,1000);
    gm.joinOnGoal(gid,5000);
    assertEquals(gm.getWrappedScheduledGoalPredicate(gid).toString(), goalPred.get(1).toString());
  }

  /**
   * Tests recovering the original predicate a persistent goal was submitted with after processing and stripping of details
   */
  @Test
  public void testGetWrappedSubmissionPredPersistent() throws InterruptedException {
    log.info("[testGetWrappedSubmissionPredPersistent]");
    Predicate goalPred = Factory.createPredicate("persistent(self:agent,test(self:agent))");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    Thread.sleep(500);
    assertEquals(gm.getWrappedScheduledGoalSubmissionPredicate(gid).toString(), goalPred.toString());
  }

  /**
   * Tests generating a wrapped predicate representing the current scheduled/persistent state of a persistent goal.
   */
  @Test
  public void testGetWrappedPredPersistent() throws InterruptedException {
    log.info("[testGetWrappedPredPersistent]");
    Predicate goalPred = Factory.createPredicate("persistent(self:agent,test(self:agent))");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    assertEquals(gm.getWrappedScheduledGoalPredicate(gid).toString(), goalPred.toString());
  }

  /**
   * Tests canceling a scheduled goal before execution
   */
  @Test
  public void testCancelScheduled() throws InterruptedException {
    log.info("[testCancelScheduled]");
    String dateString = getDateString(2, Calendar.SECOND);
    Predicate goalPred = Factory.createPredicate("scheduled(self:agent,test(self:agent),"+dateString+")");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    Thread.sleep(500);
    gm.cancelGoal(gid);
    assertEquals(GoalStatus.CANCELED,gm.joinOnGoal(gid,500));
    Thread.sleep(2000);
    assertEquals(-1, testExecutedTime);
  }

  /**
   * Tests canceling a persistently scheduled goal between executions
   */
  @Test
  public void testCancelPersistentlyScheduled() throws InterruptedException {
    log.info("[testCancelPersistentlyScheduled]");
    Calendar calendar = Calendar.getInstance();
    calendar.add(Calendar.SECOND, 2);
    Date date = calendar.getTime();
    SimpleDateFormat formatter = new SimpleDateFormat("HH-mm-ss", Locale.ENGLISH);
    String dateString = formatter.format(date);
    Predicate goalPred = Factory.createPredicate("persistent(self:agent,scheduled(self:agent,test(self:agent),"+dateString+"))");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    Thread.sleep(500);
    gm.cancelGoal(gid);
    assertEquals(GoalStatus.CANCELED,gm.joinOnGoal(gid,500));
    Thread.sleep(2000);
    assertEquals(-1, testExecutedTime);
  }

  /**
   * Tests canceling a scheduled persistently scheduled goal before execution
   */
  @Test
  public void testCancelScheduledPersistentlyScheduled() throws InterruptedException {
    log.info("[testCancelScheduledPersistentlyScheduled]");
    Predicate goalPred = Factory.createPredicate("scheduled(self:agent,persistent(self:agent,scheduled(self:agent,test(self:agent),3,SECONDS)),2,SECONDS)");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    Thread.sleep(500);
    gm.cancelGoal(gid);
    assertEquals(GoalStatus.CANCELED,gm.joinOnGoal(gid,500));
    Thread.sleep(2000);
    assertEquals(-1, testExecutedTime);
  }

  /**
   * Tests canceling a scheduled persistently scheduled goal after first execution
   */
  @Test
  public void testCancelScheduledPersistentlyScheduledV2() throws InterruptedException {
    log.info("[testCancelScheduledPersistentlyScheduledV2]");
    Predicate goalPred = Factory.createPredicate("scheduled(self:agent,persistent(self:agent,scheduled(self:agent,test(self:agent),3,SECONDS)),2,SECONDS)");
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    long expectedExecutionTime = System.currentTimeMillis() + 2000;
    Goal goal = gm.getGoal(gid);
    assertTrue(goal.isScheduled());
    assertTrue(goal.isPersistent());
    Thread.sleep(2500);
    assertEquals(expectedExecutionTime, testExecutedTime, 999);
    gm.cancelGoal(gid);
    assertEquals(GoalStatus.CANCELED,gm.joinOnGoal(gid,500));
    Thread.sleep(3000);
    assertEquals(expectedExecutionTime, testExecutedTime,999);
  }

  /**
   * Tests canceling a scheduled goal during execution of an interruptible goal
   */
  @Test
  public void testCancelInterruptibleScheduled() throws InterruptedException {
    log.info("[testCancelInterruptibleScheduled]");
    String dateString = getDateString(1, Calendar.SECOND);
    Predicate goalPred = Factory.createPredicate("scheduled(self:agent,test(self:agent),"+dateString+")");
    sleepTime = 3000;
    long gid = gm.submitGoal(goalPred, ExecutionType.ACT);
    Thread.sleep(1500);
    gm.cancelGoal(gid);
    assertEquals(GoalStatus.CANCELED,gm.joinOnGoal(gid,500));
    assertTrue(testCanceled);
  }

  /**
   * Tests when a persistently scheduled goal's execution takes longer than the scheduled delay interval. In this case
   * we expect to 'skip' a scheduled interval and execute on the next matching time (in 2x2=4seconds), rather than
   * execute again immediately.
   * Doubles as a test to make sure there is no 'drift' in the interval delay implementation.
   * Q: Does presence of drift depend on phrasing of command?
   */
  @Test
  public void testExecutionDurationLongerThanScheduledDelay() throws InterruptedException {
    log.info("[testExecutionDurationLongerThanScheduledDelay]");
    Predicate goalPred = Factory.createPredicate("persistent(self:agent,scheduled(self:agent,test(self:agent),2,SECONDS))");
    sleepTime = 3000;
    gm.submitGoal(goalPred, ExecutionType.ACT);
    long expectedExecutionTime = System.currentTimeMillis();
    Thread.sleep(3000);
    assertEquals(testExecutedTime, expectedExecutionTime, 999);
    expectedExecutionTime += 4000;
    sleepTime = 100;
    Thread.sleep(2000);
    assertEquals(testExecutedTime, expectedExecutionTime, 999);
  }

  @Action
  @TRADEService
  @OnInterrupt(onCancelServiceCall = "cancelTest()")
  public void test() {
    log.info("test called.");
    testExecutedTime = System.currentTimeMillis();
    Util.Sleep(sleepTime);
  }

  @Action
  @TRADEService
  public void cancelTest() {
    log.info("test canceled");
    testCanceled = true;
  }
}
