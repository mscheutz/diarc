package edu.tufts.hrilab.action.goal;

import java.util.concurrent.TimeUnit;

public class ScheduleInfo {
  /**
   * Delay before executing the first scheduled goal. 0 meaning, do it right now.
   */
  public long initialDelay = 0;
  /**
   * For repeated/period goals. Value < 0 indicated no repetition (i.e., just do it once).
   */
  public long period = -1;
  /**
   * Set the time to stop executing this goal. Value < 0 indicates no end time (i.e., run forever).
   * This only applies for repeated goals (i.e., period > 0).
   */
  public long endTime = -1;
  /**
   * The TimeUnit relating to both initialDelay and period timings.
   */
  public TimeUnit scheduleTimeUnit = null;
}
